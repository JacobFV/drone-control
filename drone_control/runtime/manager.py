from __future__ import annotations

import itertools
import json
import queue
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from drone_control.config import DroneConfig
from drone_control.controllers.autonomy import BoundedAutonomyController
from drone_control.controllers.base import DisabledController, SafetyConstraints
from drone_control.controllers.batched_vla import BatchedVLAController, BatchedVLAHub
from drone_control.controllers.local_vla import BatchLocalVLAClient, LocalVLAClient, LocalVLAConfig
from drone_control.controllers.manual import ManualController
from drone_control.controllers.scripted import scripted_controller
from drone_control.controllers.text_command import TextCommandController
from drone_control.controllers.vla import VLAController
from drone_control.coordinator.tasks import Assignment, MissionProgress
from drone_control.perception.frame_registry import LiveFrameRegistry
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
    batched_vla_command: list[str] | None = None
    batched_vla_timeout_seconds: float = 0.25
    batch_max_wait_seconds: float = 0.025
    vla_log_path: str | None = None


class RuntimeManager:
    def __init__(self, *, config: RuntimeManagerConfig | None = None) -> None:
        self.config = config or RuntimeManagerConfig()
        self._runtimes: dict[str, DroneRuntime] = {}
        self._manual: dict[str, ManualController] = {}
        self._events: deque[dict[str, Any]] = deque(maxlen=1000)
        self._seq = itertools.count(1)
        self._lock = threading.RLock()
        self.frame_registry = LiveFrameRegistry()
        self._vla_hub: BatchedVLAHub | None = None
        self._vla_client: BatchLocalVLAClient | None = None
        self._vla_log_lock = threading.Lock()
        self._world_model: Any | None = None
        self._ingestors: dict[str, Any] = {}

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
        self.stop_ingestion()
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
        if key in {"autonomy", "builtin_vla", "builtin-autonomy"}:
            runtime.set_controller(BoundedAutonomyController())
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
        if key in {"batched_vla", "batched-vla", "batch_vla"}:
            hub = self._ensure_vla_hub()
            wait = self.config.batched_vla_timeout_seconds + 0.05
            runtime.set_controller(
                BatchedVLAController(
                    hub=hub,
                    drone_id=drone_id,
                    registry=self.frame_registry,
                    wait_seconds=wait,
                )
            )
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

    def _ensure_vla_hub(self) -> BatchedVLAHub:
        with self._lock:
            if self._vla_hub is None:
                self._vla_hub = BatchedVLAHub(
                    self._batch_model,
                    max_wait_seconds=self.config.batch_max_wait_seconds,
                )
            return self._vla_hub

    def _batch_model(self, payloads: list[dict[str, Any]]) -> list[dict[str, Any]]:
        if self.config.batched_vla_command:
            if self._vla_client is None:
                self._vla_client = BatchLocalVLAClient(
                    LocalVLAConfig(
                        command=self.config.batched_vla_command,
                        timeout_seconds=self.config.batched_vla_timeout_seconds,
                    )
                )
            results = self._vla_client.step_batch(payloads)
        else:
            results = _neutral_batch(payloads)
        self._log_transitions(payloads, results)
        return results

    def _log_transitions(self, payloads: list[dict[str, Any]], results: list[dict[str, Any]]) -> None:
        if not self.config.vla_log_path:
            return
        results_by_id = {r.get("droneId"): r for r in results if isinstance(r, dict)}
        lines = []
        for index, payload in enumerate(payloads):
            result = results_by_id.get(payload.get("droneId"))
            if result is None and index < len(results) and isinstance(results[index], dict):
                result = results[index]
            action = result.get("action") if isinstance(result, dict) else None
            if not isinstance(action, dict):
                continue
            lines.append(
                json.dumps(
                    {
                        "droneId": payload.get("droneId"),
                        "observation": payload.get("observation"),
                        "frameJpegB64": payload.get("frameJpegB64"),
                        "recentActions": payload.get("recentActions"),
                        "action": action,
                    },
                    separators=(",", ":"),
                )
            )
        if not lines:
            return
        with self._vla_log_lock:
            path = Path(self.config.vla_log_path)
            path.parent.mkdir(parents=True, exist_ok=True)
            with path.open("a", encoding="utf-8") as handle:
                handle.write("\n".join(lines) + "\n")

    # ------------------------------------------------------------------ #
    # Live cross-drone Gaussian-splat world model
    # ------------------------------------------------------------------ #

    def attach_frame_source(self, drone_id: str, source: Any, *, pose_provider: Any | None = None) -> None:
        """Stream a live camera FrameSource for a drone into ingest_frame.

        Defaults the pose provider to the drone's latest runtime observation pose
        so frames carry a pose for the world model without extra wiring.
        """

        from drone_control.perception.ingestion import FrameIngestor

        if pose_provider is None:
            pose_provider = lambda: self._latest_pose(drone_id)  # noqa: E731
        with self._lock:
            existing = self._ingestors.pop(drone_id, None)
        if existing is not None:
            existing.stop()
        ingestor = FrameIngestor(drone_id, source, self.ingest_frame, pose_provider=pose_provider)
        ingestor.start()
        with self._lock:
            self._ingestors[drone_id] = ingestor

    def detach_frame_source(self, drone_id: str) -> None:
        with self._lock:
            ingestor = self._ingestors.pop(drone_id, None)
        if ingestor is not None:
            ingestor.stop()

    def stop_ingestion(self) -> None:
        with self._lock:
            ingestors = list(self._ingestors.values())
            self._ingestors.clear()
        for ingestor in ingestors:
            ingestor.stop()

    def ingestion_status(self) -> list[dict[str, Any]]:
        with self._lock:
            return [ingestor.status() for ingestor in self._ingestors.values()]

    def _latest_pose(self, drone_id: str) -> dict[str, Any] | None:
        with self._lock:
            runtime = self._runtimes.get(drone_id)
        if runtime is None:
            return None
        observation = runtime._last_observation
        if observation is None or observation.pose is None:
            return None
        return observation.pose.as_dict()

    def ingest_frame(self, drone_id: str, jpeg: bytes, pose: dict[str, Any] | None = None) -> None:
        """Publish a camera frame for both the batched VLA hub and the world model.

        Any live-video producer should call this; a single decode feeds both the
        diffusion policy (via the frame registry) and the splat engine.
        """

        self.frame_registry.publish(drone_id, jpeg)
        with self._lock:
            world = self._world_model
        if world is not None:
            try:
                world.ingest(drone_id, jpeg, pose)
            except Exception:
                pass

    def start_world_model(self) -> dict[str, Any]:
        from drone_control.perception import live_splat

        if not live_splat.available():
            return {"available": False, "reason": live_splat.unavailable_reason()}
        with self._lock:
            if self._world_model is None:
                self._world_model = live_splat.LiveSplatEngine()
            self._world_model.start()
            return self._world_model.snapshot()

    def stop_world_model(self) -> dict[str, Any]:
        with self._lock:
            world = self._world_model
        if world is None:
            return {"available": False, "running": False}
        world.stop()
        return world.snapshot()

    def world_model_status(self) -> dict[str, Any]:
        from drone_control.perception import live_splat

        with self._lock:
            world = self._world_model
        if world is None:
            return {"available": live_splat.available(), "running": False, "reason": live_splat.unavailable_reason()}
        return world.snapshot()

    def export_world_model(self, path: str | Path) -> str | None:
        with self._lock:
            world = self._world_model
        if world is None:
            return None
        return str(world.export_ply(Path(path)))

    def set_world_transform(self, drone_id: str, transform: list[list[float]]) -> None:
        import numpy as np

        with self._lock:
            world = self._world_model
        if world is not None:
            world.set_drone_transform(drone_id, np.asarray(transform, dtype=float))

    def bootstrap_world_model(
        self,
        drone_frames: dict[str, list[Path]],
        work_dir: str | Path,
        *,
        vo_centers_by_image: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        """Run COLMAP-union cross-drone co-registration and seed the world model."""

        from drone_control.perception import cross_drone, live_splat

        if not live_splat.available():
            return {"available": False, "reason": live_splat.unavailable_reason()}
        if not cross_drone.colmap_available():
            return {"available": True, "error": "colmap CLI not found on PATH"}
        with self._lock:
            if self._world_model is None:
                self._world_model = live_splat.LiveSplatEngine()
            world = self._world_model
        result = cross_drone.bootstrap_world_model(
            world,
            {k: [Path(p) for p in v] for k, v in drone_frames.items()},
            Path(work_dir),
            vo_centers_by_image=vo_centers_by_image,
        )
        world.start()
        return result.as_status() | world.snapshot()

    def close_vla(self) -> None:
        with self._lock:
            hub = self._vla_hub
            client = self._vla_client
            self._vla_hub = None
            self._vla_client = None
        if hub is not None:
            hub.close()
        if client is not None:
            client.close()

    def snapshots(self) -> dict[str, Any]:
        self._collect_events()
        drones = [snapshot.as_dict() for snapshot in self.snapshot_objects()]
        return {
            "running": any(item["running"] for item in drones),
            "dryRun": self.config.dry_run or not self.config.enable_io,
            "enableIo": self.config.enable_io,
            "localVlaConfigured": bool(self.config.local_vla_command),
            "batchedVlaConfigured": bool(self.config.batched_vla_command),
            "batchedVla": self._batched_vla_status(),
            "drones": drones,
            "events": list(self._events)[-50:],
        }

    def _batched_vla_status(self) -> dict[str, Any]:
        hub = self._vla_hub
        return {
            "active": hub is not None,
            "command": self.config.batched_vla_command,
            "batches": hub.batches if hub else 0,
            "lastBatchSize": hub.last_batch_size if hub else 0,
            "maxWaitSeconds": self.config.batch_max_wait_seconds,
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


def _neutral_batch(payloads: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """In-process fallback when no batched VLA command is configured.

    Emits a neutral, low-confidence action per drone so the batched loop still
    runs end-to-end (and the safety wrapper still clamps it). Point
    ``batched_vla_command`` at ``tools/diffusion_vla_policy.py`` for the real
    reverse-diffusion model.
    """

    return [
        {
            "droneId": payload.get("droneId"),
            "action": {"roll": 128, "pitch": 128, "throttle": 128, "yaw": 128},
            "confidence": 0.02,
            "reason": "batched_vla_neutral_fallback",
        }
        for payload in payloads
    ]


def _optional_number(value: object) -> int | float | None:
    if value is None or value == "":
        return None
    if isinstance(value, (int, float)):
        return value
    return float(value)
