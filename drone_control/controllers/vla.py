from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable

from drone_control.actions import DroneAction
from drone_control.runtime.events import DroneObservation

from .base import ActionRequest, SafetyConstraints


@dataclass(slots=True)
class VLAController:
    """
    Strict adapter boundary for future single-drone VLA control.

    The callable must return a dict with an ``action`` object. This adapter does
    schema-level validation only; the shared safety wrapper still clamps output
    before packets are built.
    """

    model_step: Callable[[dict[str, Any]], dict[str, Any]] | None = None
    mission_context: dict[str, Any] | None = None
    name: str = "vla"
    recent_actions: list[DroneAction] = field(default_factory=list)

    @property
    def available(self) -> bool:
        return self.model_step is not None

    def step(
        self,
        observation: DroneObservation,
        history: list[DroneObservation],
        constraints: SafetyConstraints,
    ) -> ActionRequest:
        if self.model_step is None:
            return ActionRequest.stop("vla_unavailable")
        payload = {
            "observation": observation.as_dict(),
            "history": [item.as_dict() for item in history[-20:]],
            "recentActions": [_action_as_schema(item) for item in self.recent_actions[-20:]],
            "constraints": constraints.as_dict(),
            "mission": self.mission_context or {},
        }
        try:
            result = self.model_step(payload)
            action, confidence, reason = parse_vla_output(result)
            return ActionRequest(action, reason=reason, confidence=confidence)
        except (TypeError, ValueError, KeyError) as exc:
            return ActionRequest(DroneAction.motor_stop(), reason="vla_invalid_output", fault=str(exc))

    def set_recent_actions(self, actions: list[DroneAction]) -> None:
        self.recent_actions = list(actions[-20:])

    def record_action(self, action: DroneAction) -> None:
        self.recent_actions.append(action.sanitized())
        del self.recent_actions[:-20]


def parse_vla_output(result: dict[str, Any]) -> tuple[DroneAction, float, str]:
    if not isinstance(result, dict):
        raise TypeError("model output must be an object")
    action_data = result.get("action")
    if not isinstance(action_data, dict):
        raise TypeError("action must be an object")
    allowed = {
        "roll",
        "pitch",
        "throttle",
        "yaw",
        "takeoff",
        "land",
        "emergency_stop",
        "emergencyStop",
        "calibrate",
        "headless",
        "flip",
    }
    extra = set(action_data) - allowed
    if extra:
        raise ValueError(f"unknown action fields: {sorted(extra)}")
    for axis in ("roll", "pitch", "throttle", "yaw"):
        value = action_data.get(axis, 128)
        if not isinstance(value, (int, float)):
            raise TypeError(f"{axis} must be numeric")
        if value < 0 or value > 255:
            raise ValueError(f"{axis} out of range")
    for flag in ("takeoff", "land", "emergency_stop", "emergencyStop", "calibrate", "headless", "flip"):
        if flag in action_data and not isinstance(action_data[flag], bool):
            raise TypeError(f"{flag} must be boolean")
    confidence = result.get("confidence")
    if not isinstance(confidence, (int, float)):
        raise TypeError("confidence must be numeric")
    if confidence < 0.0 or confidence > 1.0:
        raise ValueError("confidence out of range")
    reason = result.get("reason", "vla")
    if not isinstance(reason, str) or not reason:
        raise TypeError("reason must be a non-empty string")
    return (
        DroneAction(
            roll=action_data.get("roll", 128),
            pitch=action_data.get("pitch", 128),
            throttle=action_data.get("throttle", 128),
            yaw=action_data.get("yaw", 128),
            takeoff=action_data.get("takeoff", False),
            land=action_data.get("land", False),
            emergency_stop=action_data.get("emergency_stop", action_data.get("emergencyStop", False)),
            calibrate=action_data.get("calibrate", False),
            headless=action_data.get("headless", False),
            flip=action_data.get("flip", False),
        ).sanitized(),
        float(confidence),
        reason,
    )


def _action_as_schema(action: DroneAction) -> dict[str, Any]:
    action = action.sanitized()
    return {
        "roll": action.roll,
        "pitch": action.pitch,
        "throttle": action.throttle,
        "yaw": action.yaw,
        "takeoff": action.takeoff,
        "land": action.land,
        "emergencyStop": action.emergency_stop,
        "calibrate": action.calibrate,
        "headless": action.headless,
        "flip": action.flip,
    }
