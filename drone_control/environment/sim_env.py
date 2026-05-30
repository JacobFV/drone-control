from __future__ import annotations

from typing import Any

import numpy as np

from drone_control.perception.segmentation import _pose_rotation, rotmat_to_quat_xyzw
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

    def omniscient_frame(self, view: dict[str, list[float]] | None = None) -> bytes | None:
        """God's-eye view of the sim world (sim-only; no real-world analogue)."""
        return self.session.omniscient_frame(view)

    def world_model_status(self) -> dict[str, Any]:
        # The session owns the splat engine and feeds it the SAME (frame, pose)
        # any real environment would — never sim ground truth.
        return {"available": False, "running": False, "reason": "splat owned by session"}

    def camera_pose(self, drone_id: str) -> dict[str, Any] | None:
        """Calibrated camera-to-world pose (standard convention) for perception.

        This is pure camera *calibration* — the only sim-specific knowledge
        perception is allowed: the sim's forward camera is mounted at body +x
        (right = body +y, up = body +z, so image +y/down = body -z). We convert
        that mounting into the standard rotation so depth/cloud/splat treat sim
        frames identically to a real camera's. NOTHING here exposes scene
        geometry — see the rule in session_service._perceive.
        """
        idx = _index(drone_id)
        pose = self.session.latest_pose(idx) if idx is not None else None
        if pose is None:
            return None
        r = _pose_rotation(pose)  # body -> world (columns = body x, y, z in world)
        cam_rot = np.column_stack([r[:, 1], -r[:, 2], r[:, 0]])  # cols: right, down, forward
        # The optical frame is left-handed (det −1) so it cannot be carried by a
        # quaternion without corruption — ship the matrix directly. (rotation_xyzw
        # is kept for any legacy consumer but is lossy for this frame.)
        intr = self.session.camera_intrinsics()
        return {
            "x": float(pose["x"]),
            "y": float(pose["y"]),
            "z": float(pose["z"]),
            "R": cam_rot.tolist(),
            "rotation_xyzw": rotmat_to_quat_xyzw(cam_rot),
            "intrinsics": intr,  # calibration: true focal for the chosen OV lens
        }

    def set_speed(self, mode: str) -> None:
        self.session.set_max_speed(mode == "max")

    def status(self) -> dict[str, Any]:
        status = self.session.status()
        status["speed"] = "max" if self.session.config.max_speed else "realtime"
        return status
