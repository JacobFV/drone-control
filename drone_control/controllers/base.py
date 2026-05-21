from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Protocol

from drone_control.actions import DroneAction
from drone_control.runtime.events import DroneObservation


@dataclass(frozen=True, slots=True)
class SafetyConstraints:
    armed: bool = False
    max_throttle: int = 192
    require_heartbeat: bool = True
    heartbeat_timeout_seconds: float = 0.75
    throttle_slew_per_second: float = 160.0
    command_hz: float = 20.0
    metadata: dict[str, Any] = field(default_factory=dict)

    def as_dict(self) -> dict[str, Any]:
        return {
            "armed": self.armed,
            "maxThrottle": self.max_throttle,
            "requireHeartbeat": self.require_heartbeat,
            "heartbeatTimeoutSeconds": self.heartbeat_timeout_seconds,
            "throttleSlewPerSecond": self.throttle_slew_per_second,
            "commandHz": self.command_hz,
            "metadata": self.metadata,
        }


@dataclass(frozen=True, slots=True)
class ActionRequest:
    action: DroneAction
    reason: str = ""
    confidence: float = 1.0
    fault: str | None = None

    @classmethod
    def stop(cls, reason: str = "stop") -> "ActionRequest":
        return cls(action=DroneAction.motor_stop(), reason=reason, confidence=1.0)


class DroneController(Protocol):
    name: str

    def step(
        self,
        observation: DroneObservation,
        history: list[DroneObservation],
        constraints: SafetyConstraints,
    ) -> ActionRequest:
        ...


class DisabledController:
    name = "disabled"

    def step(
        self,
        observation: DroneObservation,
        history: list[DroneObservation],
        constraints: SafetyConstraints,
    ) -> ActionRequest:
        return ActionRequest.stop("disabled")

