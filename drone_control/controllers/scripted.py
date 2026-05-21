from __future__ import annotations

from dataclasses import dataclass

from drone_control.actions import DroneAction
from drone_control.runtime.events import DroneObservation

from .base import ActionRequest, SafetyConstraints


@dataclass(slots=True)
class ScriptedController:
    name: str
    sequence: list[DroneAction]
    hold_last: bool = True
    _index: int = 0

    def step(
        self,
        observation: DroneObservation,
        history: list[DroneObservation],
        constraints: SafetyConstraints,
    ) -> ActionRequest:
        if not self.sequence:
            return ActionRequest(DroneAction.neutral(), reason="empty_script")
        index = min(self._index, len(self.sequence) - 1)
        action = self.sequence[index]
        if self._index < len(self.sequence) - 1 or not self.hold_last:
            self._index += 1
        return ActionRequest(action, reason=self.name)


def neutral_controller() -> ScriptedController:
    return ScriptedController("scripted_neutral", [DroneAction.neutral()])


def stop_controller() -> ScriptedController:
    return ScriptedController("scripted_stop", [DroneAction.motor_stop()])


def takeoff_controller(*, ticks: int = 20, throttle: int = 160) -> ScriptedController:
    sequence = [DroneAction.neutral() for _ in range(3)]
    sequence.append(DroneAction(throttle=0, takeoff=True))
    sequence.extend(DroneAction(throttle=throttle) for _ in range(max(1, ticks)))
    return ScriptedController("scripted_takeoff", sequence)


def land_controller(*, ticks: int = 20) -> ScriptedController:
    sequence = [DroneAction(throttle=100) for _ in range(max(1, ticks))]
    sequence.append(DroneAction(throttle=0, land=True))
    sequence.extend(DroneAction.motor_stop() for _ in range(3))
    return ScriptedController("scripted_land", sequence)


def scripted_controller(name: str) -> ScriptedController:
    key = name.strip().lower().replace("-", "_")
    if key in {"neutral", "scripted_neutral"}:
        return neutral_controller()
    if key in {"stop", "disabled", "scripted_stop"}:
        return stop_controller()
    if key in {"takeoff", "scripted_takeoff"}:
        return takeoff_controller()
    if key in {"land", "scripted_land"}:
        return land_controller()
    raise ValueError(f"unknown scripted controller: {name}")

