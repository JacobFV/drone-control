from __future__ import annotations

from typing import Any

import numpy as np

from drone_control.perception.segmentation import _pose_rotation
from drone_control.sim.session import SimSession, SimSessionConfig


def _index(drone_id: str) -> int | None:
    if drone_id.startswith("sim-"):
        try:
            return int(drone_id.split("-", 1)[1])
        except ValueError:
            return None
    return None


class SimEnvironment:
    """Environment backed by the in-process swarm simulator.

    Wraps ``SimSession`` (analytic-expert swarm + synthetic camera renderer) and
    supports realtime vs. max-speed execution.
    """

    kind = "sim"

    def __init__(self, config: SimSessionConfig | None = None) -> None:
        self.session = SimSession(config)

    # -- lifecycle ---------------------------------------------------------

    def start(self) -> None:
        self.session.start(self.session.config)

    def stop(self) -> None:
        self.session.stop()

    # -- drones / streams --------------------------------------------------

    def drone_ids(self) -> list[str]:
        return [f"sim-{i}" for i in range(self.session.config.num_drones)]

    def latest_frame(self, drone_id: str) -> bytes | None:
        idx = _index(drone_id)
        return self.session.frame(idx) if idx is not None else None

    def latest_pose(self, drone_id: str) -> dict[str, Any] | None:
        idx = _index(drone_id)
        return self.session.latest_pose(idx) if idx is not None else None

    def trajectories(self) -> list[dict[str, Any]]:
        return self.session.trajectories().get("drones", [])

    def world_model_status(self) -> dict[str, Any]:
        # The session builds the sim splat from these frames; status is reported
        # by the SessionService (which owns the engine), not the environment.
        return {"available": False, "running": False, "reason": "splat owned by session"}

    # -- ground-truth helpers (the sim knows everything exactly) -----------

    def image_size(self) -> tuple[int, int]:
        size = self.session.config.image_size
        return size, int(size * 0.75)

    @property
    def fov_deg(self) -> float:
        return 75.0

    def positions(self) -> dict[str, list[float]]:
        out: dict[str, list[float]] = {}
        for drone in self.session.status().get("drones", []):
            out[drone["droneId"]] = [float(v) for v in drone["position"]]
        return out

    def camera_rot(self, drone_id: str) -> np.ndarray | None:
        """World directions of the camera axes (cols = right, down, forward).

        Sim camera: forward = body +x, right = body +y, up = body +z, so image
        +y (down) = body -z.
        """
        pose = self.latest_pose(drone_id)
        if pose is None:
            return None
        r = _pose_rotation(pose)  # body -> world (columns = body x, y, z in world)
        return np.column_stack([r[:, 1], -r[:, 2], r[:, 0]])

    def set_speed(self, mode: str) -> None:
        self.session.set_max_speed(mode == "max")

    def status(self) -> dict[str, Any]:
        status = self.session.status()
        status["speed"] = "max" if self.session.config.max_speed else "realtime"
        return status
