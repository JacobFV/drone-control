from __future__ import annotations

import time
from dataclasses import dataclass, field

from drone_control.actions import DroneAction, clamp_byte
from drone_control.runtime.events import DroneObservation

from .base import ActionRequest, DroneController, SafetyConstraints


@dataclass(slots=True)
class SafetyState:
    armed: bool = False
    fault_reason: str | None = None
    stop_reason: str | None = None
    last_heartbeat_at: float | None = None
    current: DroneAction = field(default_factory=DroneAction.neutral)

    def as_dict(self) -> dict[str, object]:
        return {
            "armed": self.armed,
            "faultReason": self.fault_reason,
            "stopReason": self.stop_reason,
            "current": {
                "roll": self.current.roll,
                "pitch": self.current.pitch,
                "throttle": self.current.throttle,
                "yaw": self.current.yaw,
                "takeoff": self.current.takeoff,
                "land": self.current.land,
                "emergencyStop": self.current.emergency_stop,
            },
        }


class SafetyController:
    def __init__(self, controller: DroneController, constraints: SafetyConstraints | None = None) -> None:
        self.inner = controller
        self.constraints = constraints or SafetyConstraints()
        self.state = SafetyState(armed=self.constraints.armed)
        self.name = f"safety:{controller.name}"
        self._last_update_at: float | None = None

    def arm(self, now: float | None = None) -> None:
        if self.state.fault_reason:
            raise RuntimeError(f"cannot arm while faulted: {self.state.fault_reason}")
        self.state.armed = True
        self.state.stop_reason = None
        if self.state.current.emergency_stop:
            self.state.current = DroneAction.neutral()
        self.heartbeat(now)

    def disarm(self) -> None:
        self.state.armed = False
        self.state.stop_reason = "disarm"
        self.state.current = DroneAction.motor_stop()

    def heartbeat(self, now: float | None = None) -> None:
        self.state.last_heartbeat_at = time.monotonic() if now is None else now

    def clear_fault(self) -> None:
        self.state.fault_reason = None
        self.state.stop_reason = None
        self.state.armed = False
        self.state.current = DroneAction.neutral()

    def set_controller(self, controller: DroneController) -> None:
        if controller.name != self.inner.name:
            self.state.stop_reason = "mode_switch"
            self.state.current = DroneAction.motor_stop()
        self.inner = controller
        self.name = f"safety:{controller.name}"

    def step(
        self,
        observation: DroneObservation,
        history: list[DroneObservation],
        constraints: SafetyConstraints | None = None,
    ) -> ActionRequest:
        if constraints is not None:
            self.constraints = constraints
        now = time.monotonic()
        if self._heartbeat_expired(now):
            self.state.stop_reason = "heartbeat_timeout"
            self.state.current = DroneAction.motor_stop()
            return ActionRequest.stop(self.state.stop_reason)
        if self.state.fault_reason:
            self.state.current = DroneAction.motor_stop()
            return ActionRequest(DroneAction.motor_stop(), reason="faulted", fault=self.state.fault_reason)
        if not self.state.armed and self.inner.name not in {"disabled", "scripted_stop"}:
            self.state.current = DroneAction.motor_stop()
            return ActionRequest.stop("disarmed")

        request = self.inner.step(observation, history, self.constraints)
        if request.fault:
            self.state.fault_reason = request.fault
            self.state.current = DroneAction.motor_stop()
            return ActionRequest(DroneAction.motor_stop(), reason="controller_fault", fault=request.fault)

        action = self._clamp_action(request.action.sanitized(), now)
        self.state.current = action
        return ActionRequest(action, reason=request.reason, confidence=request.confidence)

    def _heartbeat_expired(self, now: float) -> bool:
        if not self.constraints.require_heartbeat or not self.state.armed:
            return False
        if self.state.last_heartbeat_at is None:
            return True
        return now - self.state.last_heartbeat_at > self.constraints.heartbeat_timeout_seconds

    def _clamp_action(self, action: DroneAction, now: float) -> DroneAction:
        max_throttle = clamp_byte(self.constraints.max_throttle)
        target_throttle = min(clamp_byte(action.throttle), max_throttle)
        if action.emergency_stop:
            target_throttle = 0
        elapsed = self.constraints.command_hz ** -1 if self._last_update_at is None else max(0.0, now - self._last_update_at)
        self._last_update_at = now
        step = max(1, int(round(self.constraints.throttle_slew_per_second * elapsed)))
        current = self.state.current.throttle
        if action.emergency_stop:
            throttle = 0
        elif target_throttle > current:
            throttle = min(target_throttle, current + step)
        else:
            throttle = max(target_throttle, current - step)
        return DroneAction(
            roll=action.roll,
            pitch=action.pitch,
            throttle=throttle,
            yaw=action.yaw,
            takeoff=action.takeoff,
            land=action.land,
            emergency_stop=action.emergency_stop,
            calibrate=action.calibrate,
            headless=action.headless,
            flip=action.flip,
        ).sanitized()
