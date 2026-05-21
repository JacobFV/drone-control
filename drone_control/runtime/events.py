from __future__ import annotations

import time
from dataclasses import asdict, dataclass
from typing import Any

from drone_control.actions import DroneAction
from drone_control.perception.state import FrameMetadata, ImuSample, MapSummary, PoseEstimate


@dataclass(frozen=True, slots=True)
class DroneObservation:
    timestamp: float
    drone_id: str
    link_state: str
    latest_frame: FrameMetadata | None = None
    pose: PoseEstimate | None = None
    imu: ImuSample | None = None
    map_summary: MapSummary | None = None
    battery: float | None = None
    confidence: float = 0.0

    @classmethod
    def empty(cls, drone_id: str, *, link_state: str = "unknown") -> "DroneObservation":
        return cls(timestamp=time.time(), drone_id=drone_id, link_state=link_state)

    def as_dict(self) -> dict[str, Any]:
        return {
            "timestamp": self.timestamp,
            "droneId": self.drone_id,
            "linkState": self.link_state,
            "latestFrame": self.latest_frame.as_dict() if self.latest_frame else None,
            "pose": self.pose.as_dict() if self.pose else None,
            "imu": self.imu.as_dict() if self.imu else None,
            "mapSummary": self.map_summary.as_dict() if self.map_summary else None,
            "battery": self.battery,
            "confidence": self.confidence,
        }


@dataclass(frozen=True, slots=True)
class RuntimeEvent:
    type: str
    drone_id: str
    timestamp: float

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True, slots=True)
class RuntimeErrorEvent(RuntimeEvent):
    message: str
    fatal: bool = False

    def __init__(self, drone_id: str, message: str, *, fatal: bool = False, timestamp: float | None = None) -> None:
        object.__setattr__(self, "type", "runtime_error")
        object.__setattr__(self, "drone_id", drone_id)
        object.__setattr__(self, "timestamp", time.time() if timestamp is None else timestamp)
        object.__setattr__(self, "message", message)
        object.__setattr__(self, "fatal", fatal)

    def as_dict(self) -> dict[str, Any]:
        return {
            "type": self.type,
            "droneId": self.drone_id,
            "timestamp": self.timestamp,
            "message": self.message,
            "fatal": self.fatal,
        }


@dataclass(frozen=True, slots=True)
class LinkStatusEvent(RuntimeEvent):
    state: str
    sent: int
    errors: int

    def __init__(self, drone_id: str, state: str, *, sent: int = 0, errors: int = 0, timestamp: float | None = None) -> None:
        object.__setattr__(self, "type", "link_status")
        object.__setattr__(self, "drone_id", drone_id)
        object.__setattr__(self, "timestamp", time.time() if timestamp is None else timestamp)
        object.__setattr__(self, "state", state)
        object.__setattr__(self, "sent", sent)
        object.__setattr__(self, "errors", errors)

    def as_dict(self) -> dict[str, Any]:
        return {
            "type": self.type,
            "droneId": self.drone_id,
            "timestamp": self.timestamp,
            "state": self.state,
            "sent": self.sent,
            "errors": self.errors,
        }


@dataclass(frozen=True, slots=True)
class ObservationEvent(RuntimeEvent):
    observation: DroneObservation

    def __init__(self, drone_id: str, observation: DroneObservation, *, timestamp: float | None = None) -> None:
        object.__setattr__(self, "type", "observation")
        object.__setattr__(self, "drone_id", drone_id)
        object.__setattr__(self, "timestamp", time.time() if timestamp is None else timestamp)
        object.__setattr__(self, "observation", observation)

    def as_dict(self) -> dict[str, Any]:
        return {
            "type": self.type,
            "droneId": self.drone_id,
            "timestamp": self.timestamp,
            "observation": self.observation.as_dict(),
        }


@dataclass(frozen=True, slots=True)
class ActionEvent(RuntimeEvent):
    action: DroneAction
    controller: str

    def __init__(self, drone_id: str, action: DroneAction, *, controller: str, timestamp: float | None = None) -> None:
        object.__setattr__(self, "type", "action")
        object.__setattr__(self, "drone_id", drone_id)
        object.__setattr__(self, "timestamp", time.time() if timestamp is None else timestamp)
        object.__setattr__(self, "action", action.sanitized())
        object.__setattr__(self, "controller", controller)

    def as_dict(self) -> dict[str, Any]:
        return {
            "type": self.type,
            "droneId": self.drone_id,
            "timestamp": self.timestamp,
            "controller": self.controller,
            "action": asdict(self.action),
        }
