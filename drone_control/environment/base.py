from __future__ import annotations

from typing import Any, Protocol, runtime_checkable


@runtime_checkable
class Environment(Protocol):
    """Uniform surface over a simulated or real drone environment.

    Both implementations emit the same per-drone shape (``{pose, jpeg}``) so the
    session/perception/recording path downstream is identical regardless of
    whether frames come from the simulator or live hardware.
    """

    kind: str  # "sim" | "real"

    def start(self) -> None:
        ...

    def stop(self) -> None:
        ...

    def drone_ids(self) -> list[str]:
        ...

    def latest_frame(self, drone_id: str) -> bytes | None:
        ...

    def latest_pose(self, drone_id: str) -> dict[str, Any] | None:
        ...

    def trajectories(self) -> list[dict[str, Any]]:
        """[{droneId, color, goal, poses:[{x,y,z,qw,qx,qy,qz},...]}, ...]."""
        ...

    def world_model_status(self) -> dict[str, Any]:
        ...

    def set_speed(self, mode: str) -> None:
        """``"realtime"`` or ``"max"`` (sim only; real ignores)."""
        ...

    def status(self) -> dict[str, Any]:
        ...
