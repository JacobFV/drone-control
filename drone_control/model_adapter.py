from __future__ import annotations

import math
import time

from .actions import DroneAction


class ModelAdapter:
    """
    Replace this class with your monolithic model.

    Expected contract:
        step(observations: list[object], text_command: str) -> list[DroneAction | dict]
    """

    def step(self, observations: list[object], text_command: str) -> list[DroneAction]:
        command = text_command.lower()
        if "stop" in command or "kill" in command:
            return [DroneAction.motor_stop() for _ in observations]
        if "calibrate" in command:
            return [DroneAction(calibrate=True) for _ in observations]
        if "land" in command:
            return [DroneAction(land=True) for _ in observations]
        if "takeoff" in command:
            return [DroneAction(takeoff=True) for _ in observations]

        phase = math.sin(time.monotonic() * 0.5)
        return [DroneAction(roll=128, pitch=128, throttle=128 + int(4 * phase), yaw=128) for _ in observations]

