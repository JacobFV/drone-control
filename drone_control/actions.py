from __future__ import annotations

from dataclasses import dataclass


def clamp_byte(value: int | float) -> int:
    return max(0, min(255, int(round(value))))


@dataclass(slots=True)
class DroneAction:
    roll: int = 128
    pitch: int = 128
    throttle: int = 128
    yaw: int = 128
    takeoff: bool = False
    land: bool = False
    emergency_stop: bool = False
    calibrate: bool = False
    headless: bool = False
    flip: bool = False

    @classmethod
    def neutral(cls) -> "DroneAction":
        return cls()

    @classmethod
    def motor_stop(cls) -> "DroneAction":
        return cls(throttle=0, emergency_stop=True)

    def sanitized(self) -> "DroneAction":
        return DroneAction(
            roll=clamp_byte(self.roll),
            pitch=clamp_byte(self.pitch),
            throttle=clamp_byte(self.throttle),
            yaw=clamp_byte(self.yaw),
            takeoff=bool(self.takeoff),
            land=bool(self.land),
            emergency_stop=bool(self.emergency_stop),
            calibrate=bool(self.calibrate),
            headless=bool(self.headless),
            flip=bool(self.flip),
        )


def action_from_dict(data: dict) -> DroneAction:
    return DroneAction(
        roll=data.get("roll", 128),
        pitch=data.get("pitch", 128),
        throttle=data.get("throttle", 128),
        yaw=data.get("yaw", 128),
        takeoff=data.get("takeoff", False),
        land=data.get("land", False),
        emergency_stop=data.get("emergency_stop", False),
        calibrate=data.get("calibrate", False),
        headless=data.get("headless", False),
        flip=data.get("flip", False),
    ).sanitized()

