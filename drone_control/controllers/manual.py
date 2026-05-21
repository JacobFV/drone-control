from __future__ import annotations

from dataclasses import dataclass, field

from drone_control.actions import DroneAction
from drone_control.runtime.events import DroneObservation

from .base import ActionRequest, SafetyConstraints


@dataclass(slots=True)
class ManualController:
    name: str = "manual"
    _target: DroneAction = field(default_factory=lambda: DroneAction(roll=128, pitch=128, throttle=0, yaw=128))
    _stop_requested: bool = False

    def set_axes(
        self,
        *,
        roll: int | float | None = None,
        pitch: int | float | None = None,
        throttle: int | float | None = None,
        yaw: int | float | None = None,
    ) -> None:
        self._target = DroneAction(
            roll=self._target.roll if roll is None else roll,
            pitch=self._target.pitch if pitch is None else pitch,
            throttle=self._target.throttle if throttle is None else throttle,
            yaw=self._target.yaw if yaw is None else yaw,
        ).sanitized()
        self._stop_requested = False

    def stop(self) -> None:
        self._target = DroneAction.motor_stop()
        self._stop_requested = True

    def step(
        self,
        observation: DroneObservation,
        history: list[DroneObservation],
        constraints: SafetyConstraints,
    ) -> ActionRequest:
        reason = "manual_stop" if self._stop_requested else "manual_axes"
        return ActionRequest(self._target, reason=reason)

