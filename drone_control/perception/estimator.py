from __future__ import annotations

from typing import Protocol

from .state import PoseEstimate


class PoseEstimator(Protocol):
    def latest_pose(self) -> PoseEstimate | None:
        ...


class NullPoseEstimator:
    def latest_pose(self) -> PoseEstimate | None:
        return None


class VisualOdometryStatusAdapter:
    def __init__(self, estimator: object) -> None:
        self.estimator = estimator

    def latest_pose(self) -> PoseEstimate | None:
        poses = getattr(self.estimator, "poses", None)
        if callable(poses):
            values = poses(-1)
            if values:
                item = values[-1]
                return PoseEstimate(
                    timestamp=item.get("timestamp"),
                    frame_index=item.get("frameIndex"),
                    translation=tuple(item.get("translation", (0.0, 0.0, 0.0))),
                    rotation_xyzw=tuple(item.get("rotation", (0.0, 0.0, 0.0, 1.0))),
                    quality=str(item.get("quality", "tracking")),
                    confidence=float(item.get("confidence", 0.5)),
                )
        return None

