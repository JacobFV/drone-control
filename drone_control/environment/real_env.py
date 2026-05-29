from __future__ import annotations

from typing import Any

from drone_control.runtime.manager import RuntimeManager


class RealEnvironment:
    """Environment backed by the live runtime (physical drones / replay).

    Thin adapter over the existing ``RuntimeManager`` (per-drone control loops,
    shared frame registry, guidance bus, live splat world model).
    """

    kind = "real"

    def __init__(self, runtime: RuntimeManager) -> None:
        self.runtime = runtime

    # -- lifecycle ---------------------------------------------------------

    def start(self) -> None:
        # The splat is owned + fed by the SessionService perception loop (same as
        # sim), so we do NOT also spin up the runtime's own world model here.
        self.runtime.start_all()

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

    def camera_pose(self, drone_id: str) -> dict[str, Any] | None:
        # The visual-odometry pose estimate is already a camera-frame pose; pass
        # it straight through (translation + whatever rotation it carries). The
        # perception stack treats this exactly like the sim's calibrated pose.
        return self.latest_pose(drone_id)

    def trajectories(self) -> list[dict[str, Any]]:
        return self.runtime.trajectories()

    def world_model_status(self) -> dict[str, Any]:
        return {"available": False, "running": False, "reason": "splat owned by session"}

    def set_speed(self, mode: str) -> None:
        # Real hardware runs at wall-clock speed; nothing to do.
        return None

    def status(self) -> dict[str, Any]:
        snap = self.runtime.snapshots()
        snap["speed"] = "realtime"
        return snap
