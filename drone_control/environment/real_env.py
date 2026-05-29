from __future__ import annotations

from typing import Any

from drone_control.runtime.manager import RuntimeManager


class RealEnvironment:
    """Environment backed by the live runtime (physical drones / replay).

    Thin adapter over the existing ``RuntimeManager`` (per-drone control loops,
    shared frame registry, guidance bus, live splat world model).
    """

    kind = "real"

    def __init__(self, runtime: RuntimeManager, *, world_model: bool = True) -> None:
        self.runtime = runtime
        self._world_model = world_model

    # -- lifecycle ---------------------------------------------------------

    def start(self) -> None:
        self.runtime.start_all()
        if self._world_model:
            try:
                self.runtime.start_world_model()
            except Exception:
                # World model is best-effort (needs torch/gsplat/CUDA).
                pass

    def stop(self) -> None:
        self.runtime.stop_all()

    # -- drones / streams --------------------------------------------------

    def drone_ids(self) -> list[str]:
        return list(self.runtime.runtime_ids())

    def latest_frame(self, drone_id: str) -> bytes | None:
        frame = self.runtime.frame_registry.latest(drone_id)
        return frame.jpeg if frame is not None else None

    def latest_pose(self, drone_id: str) -> dict[str, Any] | None:
        for traj in self.runtime.trajectories(limit=1):
            if traj.get("droneId") == drone_id and traj.get("poses"):
                return dict(traj["poses"][-1])
        return None

    def trajectories(self) -> list[dict[str, Any]]:
        return self.runtime.trajectories()

    def world_model_status(self) -> dict[str, Any]:
        return self.runtime.world_model_status()

    def set_speed(self, mode: str) -> None:
        # Real hardware runs at wall-clock speed; nothing to do.
        return None

    def status(self) -> dict[str, Any]:
        snap = self.runtime.snapshots()
        snap["speed"] = "realtime"
        return snap
