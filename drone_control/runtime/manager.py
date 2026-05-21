from __future__ import annotations

import itertools
import queue
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Any

from drone_control.config import DroneConfig
from drone_control.controllers.base import DisabledController, SafetyConstraints
from drone_control.controllers.local_vla import LocalVLAClient, LocalVLAConfig
from drone_control.controllers.manual import ManualController
from drone_control.controllers.scripted import scripted_controller
from drone_control.controllers.text_command import TextCommandController
from drone_control.controllers.vla import VLAController
from drone_control.coordinator.tasks import Assignment, MissionProgress
from drone_control.protocols import make_protocol
from drone_control.transport import make_drone_link

from .drone_runtime import DroneRuntimeSnapshot
from .drone_runtime import DroneRuntime
from .events import RuntimeEvent


@dataclass(slots=True)
class RuntimeManagerConfig:
    control_hz: float = 20.0
    dry_run: bool = True
    enable_io: bool = False
    local_vla_command: list[str] | None = None
    local_vla_timeout_seconds: float = 0.25


class RuntimeManager:
    def __init__(self, *, config: RuntimeManagerConfig | None = None) -> None:
        self.config = config or RuntimeManagerConfig()
        self._runtimes: dict[str, DroneRuntime] = {}
        self._manual: dict[str, ManualController] = {}
        self._events: deque[dict[str, Any]] = deque(maxlen=1000)
        self._seq = itertools.count(1)
        self._lock = threading.RLock()

    def configure_drones(self, configs: list[DroneConfig]) -> None:
        with self._lock:
            existing = set(self._runtimes)
            wanted = {cfg.id for cfg in configs}
            for drone_id in existing - wanted:
                self._runtimes.pop(drone_id).stop()
                self._manual.pop(drone_id, None)
            for cfg in configs:
                if cfg.id in self._runtimes:
                    continue
                manual = ManualController()
                link = None
                dry_run = self.config.dry_run or not self.config.enable_io
                if not dry_run:
                    link = make_drone_link(cfg)
                runtime = DroneRuntime(
                    drone_id=cfg.id,
                    protocol=make_protocol(cfg.protocol),
                    link=link,
                    controller=DisabledController(),
                    constraints=SafetyConstraints(command_hz=self.config.control_hz),
                    control_hz=self.config.control_hz,
                    dry_run=dry_run,
                    link_type=cfg.link_type,
                )
                self._runtimes[cfg.id] = runtime
                self._manual[cfg.id] = manual

    def start_all(self) -> None:
        with self._lock:
            for runtime in self._runtimes.values():
                runtime.start()

    def stop_all(self) -> None:
        with self._lock:
            runtimes = list(self._runtimes.values())
        for runtime in runtimes:
            runtime.stop()

    def set_controller(self, drone_id: str, mode: str) -> None:
        runtime = self._get(drone_id)
        key = mode.strip().lower()
        if key == "manual":
            runtime.set_controller(self._manual[drone_id])
            return
        if key == "vla":
            if not self.config.local_vla_command:
                runtime.set_controller(VLAController(model_step=None))
                return
            client = LocalVLAClient(
                LocalVLAConfig(
                    command=self.config.local_vla_command,
                    timeout_seconds=self.config.local_vla_timeout_seconds,
                )
            )
            runtime.set_controller(VLAController(model_step=client.step))
            return
        if key in {"disabled", "off"}:
            runtime.set_controller(DisabledController())
            return
        try:
            runtime.set_controller(scripted_controller(key))
        except ValueError:
            runtime.set_controller(TextCommandController(mode))

    def set_all_controllers(self, mode: str) -> None:
        for drone_id in self.runtime_ids():
            self.set_controller(drone_id, mode)

    def arm(self, drone_id: str) -> None:
        self._get(drone_id).arm()

    def arm_all(self) -> None:
        for drone_id in self.runtime_ids():
            self.arm(drone_id)

    def disarm(self, drone_id: str) -> None:
        self._get(drone_id).disarm()

    def heartbeat(self, drone_id: str) -> None:
        self._get(drone_id).heartbeat()

    def heartbeat_all(self) -> None:
        for drone_id in self.runtime_ids():
            self.heartbeat(drone_id)

    def clear_fault(self, drone_id: str) -> None:
        self._get(drone_id).clear_fault()

    def set_manual_axes(self, drone_id: str, axes: dict[str, object]) -> None:
        self._manual[drone_id].set_axes(
            roll=_optional_number(axes.get("roll")),
            pitch=_optional_number(axes.get("pitch")),
            throttle=_optional_number(axes.get("throttle")),
            yaw=_optional_number(axes.get("yaw")),
        )

    def stop_manual(self, drone_id: str) -> None:
        self._manual[drone_id].stop()

    def apply_mission_progress(self, progress: MissionProgress) -> None:
        for assignment in progress.assignments:
            self.apply_assignment(assignment)

    def apply_assignment(self, assignment: Assignment) -> None:
        constraints = assignment.constraints
        if constraints is None:
            return
        runtime = self._get(assignment.drone_id)
        current = runtime.controller.constraints
        runtime.update_constraints(
            SafetyConstraints(
                armed=runtime.controller.state.armed,
                max_throttle=current.max_throttle if constraints.max_throttle is None else constraints.max_throttle,
                require_heartbeat=current.require_heartbeat
                if constraints.require_heartbeat is None
                else constraints.require_heartbeat,
                heartbeat_timeout_seconds=current.heartbeat_timeout_seconds,
                throttle_slew_per_second=current.throttle_slew_per_second,
                command_hz=current.command_hz,
                metadata={
                    **current.metadata,
                    "assignmentRole": assignment.role,
                    "assignmentTask": assignment.task,
                    "minConfidence": constraints.min_confidence,
                },
            )
        )

    def snapshots(self) -> dict[str, Any]:
        self._collect_events()
        drones = [snapshot.as_dict() for snapshot in self.snapshot_objects()]
        return {
            "running": any(item["running"] for item in drones),
            "dryRun": self.config.dry_run or not self.config.enable_io,
            "enableIo": self.config.enable_io,
            "localVlaConfigured": bool(self.config.local_vla_command),
            "drones": drones,
            "events": list(self._events)[-50:],
        }

    def snapshot_objects(self) -> list[DroneRuntimeSnapshot]:
        with self._lock:
            return [runtime.snapshot() for runtime in self._runtimes.values()]

    def runtime_ids(self) -> list[str]:
        with self._lock:
            return list(self._runtimes)

    def event_stream(self, *, since: int = 0) -> dict[str, Any]:
        self._collect_events()
        events = [event for event in self._events if int(event["seq"]) > since]
        return {"events": events, "latestSeq": events[-1]["seq"] if events else since}

    def _collect_events(self) -> None:
        with self._lock:
            runtimes = list(self._runtimes.values())
        for runtime in runtimes:
            for event in runtime.drain_events():
                payload = event.as_dict() if isinstance(event, RuntimeEvent) else dict(event)
                payload["seq"] = next(self._seq)
                self._events.append(payload)

    def _get(self, drone_id: str) -> DroneRuntime:
        with self._lock:
            runtime = self._runtimes.get(drone_id)
        if runtime is None:
            raise KeyError(f"unknown drone runtime: {drone_id}")
        return runtime


def _optional_number(value: object) -> int | float | None:
    if value is None or value == "":
        return None
    if isinstance(value, (int, float)):
        return value
    return float(value)
