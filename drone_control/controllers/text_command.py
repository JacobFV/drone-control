from __future__ import annotations

from dataclasses import dataclass

from drone_control.actions import DroneAction
from drone_control.runtime.events import DroneObservation

from .base import ActionRequest, SafetyConstraints


@dataclass(slots=True)
class TextCommandController:
    command: str
    name: str = "text_command"

    def step(
        self,
        observation: DroneObservation,
        history: list[DroneObservation],
        constraints: SafetyConstraints,
    ) -> ActionRequest:
        command = self.command.lower()
        if "emergency" in command or "halt" in command or "stop" in command:
            return ActionRequest(DroneAction.motor_stop(), reason="text_stop")
        if "land" in command:
            return ActionRequest(DroneAction(land=True, throttle=0), reason="text_land")
        if "takeoff" in command:
            return ActionRequest(DroneAction(takeoff=True), reason="text_takeoff")
        if "calibrate" in command:
            return ActionRequest(DroneAction(calibrate=True), reason="text_calibrate")
        return ActionRequest(DroneAction.neutral(), reason="text_neutral")
