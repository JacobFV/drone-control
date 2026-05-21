from __future__ import annotations

from dataclasses import dataclass, field
from typing import Protocol

from .estimator import NullPoseEstimator, PoseEstimator
from .frames import FrameMetadataSource, StaticFrameSource
from .state import ImuSample, MapSummary, PerceptionStatus


class ImuSource(Protocol):
    def latest(self) -> ImuSample | None:
        ...


class NullImuSource:
    def latest(self) -> ImuSample | None:
        return None


@dataclass(slots=True)
class PerceptionPipeline:
    frame_source: FrameMetadataSource = field(default_factory=StaticFrameSource)
    pose_estimator: PoseEstimator = field(default_factory=NullPoseEstimator)
    imu_source: ImuSource = field(default_factory=NullImuSource)
    map_summary: MapSummary = field(default_factory=MapSummary)

    def status(self) -> PerceptionStatus:
        frame = self.frame_source.latest()
        pose = self.pose_estimator.latest_pose()
        imu = self.imu_source.latest()
        confidence = 0.0
        if frame is not None:
            confidence += 0.25
        if pose is not None:
            confidence += min(0.45, max(0.0, pose.confidence))
        if imu is not None:
            confidence += 0.15
        if self.map_summary.state != "none":
            confidence += 0.15
        return PerceptionStatus(
            frame=frame,
            pose=pose,
            imu=imu,
            map_summary=self.map_summary,
            confidence=min(1.0, confidence),
        )
