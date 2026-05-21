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
from drone_control.controllers.manual import ManualController
from drone_control.controllers.scripted import scripted_controller
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
        if key in {"disabled", "off"}:
            runtime.set_controller(DisabledController())
            return
        runtime.set_controller(scripted_controller(key))

    def arm(self, drone_id: str) -> None:
        self._get(drone_id).arm()

    def disarm(self, drone_id: str) -> None:
        self._get(drone_id).disarm()

    def heartbeat(self, drone_id: str) -> None:
        self._get(drone_id).heartbeat()

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

    def snapshots(self) -> dict[str, Any]:
        self._collect_events()
        drones = [snapshot.as_dict() for snapshot in self.snapshot_objects()]
        return {
            "running": any(item["running"] for item in drones),
            "dryRun": self.config.dry_run or not self.config.enable_io,
            "enableIo": self.config.enable_io,
            "drones": drones,
            "events": list(self._events)[-50:],
        }

    def snapshot_objects(self) -> list[DroneRuntimeSnapshot]:
        with self._lock:
            return [runtime.snapshot() for runtime in self._runtimes.values()]

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
