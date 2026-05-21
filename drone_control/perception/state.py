from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any


@dataclass(frozen=True, slots=True)
class FrameMetadata:
    index: int | None = None
    timestamp: float | None = None
    width: int | None = None
    height: int | None = None
    source: str = ""

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True, slots=True)
class PoseEstimate:
    timestamp: float | None = None
    frame_index: int | None = None
    translation: tuple[float, float, float] | None = None
    rotation_xyzw: tuple[float, float, float, float] | None = None
    quality: str = "unavailable"
    confidence: float = 0.0

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True, slots=True)
class ImuSample:
    timestamp: float | None = None
    acceleration: tuple[float, float, float] | None = None
    gyro: tuple[float, float, float] | None = None

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True, slots=True)
class MapSummary:
    state: str = "none"
    record_id: str | None = None
    keyframes: int = 0
    points: int = 0
    label: str = ""

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True, slots=True)
class PerceptionStatus:
    frame: FrameMetadata | None = None
    pose: PoseEstimate | None = None
    imu: ImuSample | None = None
    map_summary: MapSummary = field(default_factory=MapSummary)
    confidence: float = 0.0

    def as_dict(self) -> dict[str, Any]:
        return {
            "frame": self.frame.as_dict() if self.frame else None,
            "pose": self.pose.as_dict() if self.pose else None,
            "imu": self.imu.as_dict() if self.imu else None,
            "mapSummary": self.map_summary.as_dict(),
            "confidence": self.confidence,
        }

