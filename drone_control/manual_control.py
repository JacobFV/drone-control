from __future__ import annotations

import time
from dataclasses import asdict
from dataclasses import dataclass
from enum import Enum

from .actions import DroneAction
from .actions import clamp_byte


DEFAULT_MAX_THROTTLE = 192
DEFAULT_HEARTBEAT_TIMEOUT_SECONDS = 0.75
DEFAULT_ACK_TIMEOUT_SECONDS = 0.4
DEFAULT_COMMAND_HZ = 20.0
DEFAULT_THROTTLE_SLEW_PER_SECOND = 160.0
DEFAULT_RAMP_DOWN_PER_SECOND = 320.0
DEFAULT_STOP_CONFIRM_COMMANDS = 3


class ManualControlState(str, Enum):
    DISARMED = "disarmed"
    ARMED = "armed"
    ACTIVE = "active"
    STOPPING = "stopping"
    FAULTED = "faulted"


@dataclass(frozen=True, slots=True)
class ManualControlConfig:
    max_throttle: int = DEFAULT_MAX_THROTTLE
    heartbeat_timeout_seconds: float = DEFAULT_HEARTBEAT_TIMEOUT_SECONDS
    ack_timeout_seconds: float = DEFAULT_ACK_TIMEOUT_SECONDS
    command_hz: float = DEFAULT_COMMAND_HZ
    throttle_slew_per_second: float = DEFAULT_THROTTLE_SLEW_PER_SECOND
    ramp_down_per_second: float = DEFAULT_RAMP_DOWN_PER_SECOND
    stop_confirm_commands: int = DEFAULT_STOP_CONFIRM_COMMANDS

    @property
    def command_interval_seconds(self) -> float:
        return 1.0 / max(1.0, self.command_hz)


def action_to_axis_dict(action: DroneAction) -> dict[str, int | bool]:
    return asdict(action.sanitized())


class ManualControlSession:
    """
    Server-side manual-control safety state machine.

    The HTTP layer can call arm/disarm/heartbeat/set_target_axes as requests
    arrive, call tick(now) from its service loop, send any returned DroneAction,
    and then call ack(now) when that command has been accepted by the transport.
    """

    def __init__(self, config: ManualControlConfig | None = None) -> None:
        self.config = config or ManualControlConfig()
        self.state = ManualControlState.DISARMED
        self.fault_reason: str | None = None
        self.stop_reason: str | None = None

        self._target = DroneAction(roll=128, pitch=128, throttle=0, yaw=128)
        self._current = DroneAction(roll=128, pitch=128, throttle=0, yaw=128)
        self._last_heartbeat_at: float | None = None
        self._last_emit_at: float | None = None
        self._last_update_at: float | None = None
        self._pending_ack_since: float | None = None
        self._stop_confirms_remaining = 0

    @property
    def armed(self) -> bool:
        return self.state in {
            ManualControlState.ARMED,
            ManualControlState.ACTIVE,
            ManualControlState.STOPPING,
        }

    def arm(self, now: float | None = None) -> None:
        now = self._now(now)
        if self.state == ManualControlState.FAULTED:
            raise RuntimeError(f"cannot arm while faulted: {self.fault_reason}")
        self.state = ManualControlState.ARMED
        self.fault_reason = None
        self.stop_reason = None
        self._last_heartbeat_at = now
        self._last_update_at = now
        self._pending_ack_since = None
        self._stop_confirms_remaining = 0
        self._target = DroneAction(roll=128, pitch=128, throttle=0, yaw=128)
        self._current = DroneAction(roll=128, pitch=128, throttle=0, yaw=128)

    def configure(
        self,
        *,
        max_throttle: int | None = None,
        command_hz: float | None = None,
        throttle_slew_per_second: float | None = None,
        heartbeat_timeout_seconds: float | None = None,
    ) -> None:
        if self.armed:
            raise RuntimeError("manual policy can only be changed while disarmed")
        self.config = ManualControlConfig(
            max_throttle=self.config.max_throttle if max_throttle is None else max(0, min(255, int(max_throttle))),
            heartbeat_timeout_seconds=self.config.heartbeat_timeout_seconds if heartbeat_timeout_seconds is None else float(heartbeat_timeout_seconds),
            ack_timeout_seconds=self.config.ack_timeout_seconds,
            command_hz=self.config.command_hz if command_hz is None else max(1.0, float(command_hz)),
            throttle_slew_per_second=self.config.throttle_slew_per_second if throttle_slew_per_second is None else max(1.0, float(throttle_slew_per_second)),
            ramp_down_per_second=self.config.ramp_down_per_second,
            stop_confirm_commands=self.config.stop_confirm_commands,
        )

    def disarm(self, now: float | None = None) -> DroneAction | None:
        return self._begin_stopping(self._now(now), "disarm", force_emit=True)

    def clear_fault(self) -> None:
        self.state = ManualControlState.DISARMED
        self.fault_reason = None
        self.stop_reason = None
        self._pending_ack_since = None
        self._stop_confirms_remaining = 0
        self._target = DroneAction(roll=128, pitch=128, throttle=0, yaw=128)
        self._current = DroneAction(roll=128, pitch=128, throttle=0, yaw=128)

    def heartbeat(self, now: float | None = None) -> None:
        self._last_heartbeat_at = self._now(now)

    def ack(self, now: float | None = None) -> None:
        self._pending_ack_since = None

    def set_target_axes(
        self,
        *,
        roll: int | float | None = None,
        pitch: int | float | None = None,
        throttle: int | float | None = None,
        yaw: int | float | None = None,
        now: float | None = None,
    ) -> bool:
        now = self._now(now)
        if self.state not in {ManualControlState.ARMED, ManualControlState.ACTIVE}:
            return False
        self._check_timeouts(now)
        if self.state not in {ManualControlState.ARMED, ManualControlState.ACTIVE}:
            return False

        self._target = DroneAction(
            roll=self._axis_or_current(roll, self._target.roll),
            pitch=self._axis_or_current(pitch, self._target.pitch),
            throttle=self._throttle_or_current(throttle, self._target.throttle),
            yaw=self._axis_or_current(yaw, self._target.yaw),
        )
        self.state = ManualControlState.ACTIVE
        return True

    def emergency_stop(self, now: float | None = None) -> DroneAction:
        now = self._now(now)
        self.state = ManualControlState.FAULTED
        self.fault_reason = "emergency_stop"
        self.stop_reason = None
        self._current = DroneAction.motor_stop()
        return self._emit(self._current, now)

    def mode_switch_stop(self, now: float | None = None) -> DroneAction | None:
        return self._begin_stopping(self._now(now), "mode_switch", force_emit=True)

    def tick(self, now: float | None = None) -> DroneAction | None:
        now = self._now(now)
        self._check_timeouts(now)
        if not self._should_emit(now):
            return None

        if self.state == ManualControlState.DISARMED:
            return None
        if self.state == ManualControlState.FAULTED:
            self._current = DroneAction.motor_stop()
            return self._emit(self._current, now)
        if self.state == ManualControlState.STOPPING:
            return self._tick_stopping(now)
        if self.state in {ManualControlState.ARMED, ManualControlState.ACTIVE}:
            self._current = self._slew_toward_target(now)
            return self._emit(self._current, now)
        return None

    def tick_dict(self, now: float | None = None) -> dict[str, int | bool] | None:
        action = self.tick(now)
        if action is None:
            return None
        return action_to_axis_dict(action)

    def current_action_dict(self) -> dict[str, int | bool]:
        return action_to_axis_dict(self._current)

    def _tick_stopping(self, now: float) -> DroneAction:
        self._target = DroneAction(roll=128, pitch=128, throttle=0, yaw=128)
        self._current = self._ramp_down(now)
        action = self._emit(self._current, now)
        if self._current.throttle == 0:
            self._stop_confirms_remaining -= 1
            if self._stop_confirms_remaining <= 0:
                self.state = ManualControlState.DISARMED
                self.stop_reason = None
                self._pending_ack_since = None
        return action

    def _begin_stopping(self, now: float, reason: str, *, force_emit: bool) -> DroneAction | None:
        self.fault_reason = None
        self.stop_reason = reason
        self.state = ManualControlState.STOPPING
        self._target = DroneAction(roll=128, pitch=128, throttle=0, yaw=128)
        self._stop_confirms_remaining = max(1, self.config.stop_confirm_commands)
        if force_emit:
            self._last_emit_at = None
            return self.tick(now)
        return None

    def _check_timeouts(self, now: float) -> None:
        if self.state in {ManualControlState.ARMED, ManualControlState.ACTIVE}:
            if self._last_heartbeat_at is None:
                self._begin_stopping(now, "missing_heartbeat", force_emit=False)
            elif now - self._last_heartbeat_at > self.config.heartbeat_timeout_seconds:
                self._begin_stopping(now, "heartbeat_timeout", force_emit=False)

        if (
            self.state in {ManualControlState.ARMED, ManualControlState.ACTIVE, ManualControlState.STOPPING}
            and self._pending_ack_since is not None
            and now - self._pending_ack_since > self.config.ack_timeout_seconds
        ):
            self.state = ManualControlState.FAULTED
            self.fault_reason = "ack_timeout"
            self.stop_reason = None
            self._current = DroneAction.motor_stop()
            self._last_emit_at = None

    def _slew_toward_target(self, now: float) -> DroneAction:
        elapsed = self._elapsed_update(now)
        step = max(1, int(round(self.config.throttle_slew_per_second * elapsed)))
        return DroneAction(
            roll=self._target.roll,
            pitch=self._target.pitch,
            throttle=self._step_axis(self._current.throttle, self._target.throttle, step),
            yaw=self._target.yaw,
        ).sanitized()

    def _ramp_down(self, now: float) -> DroneAction:
        elapsed = self._elapsed_update(now)
        step = max(1, int(round(self.config.ramp_down_per_second * elapsed)))
        return DroneAction(
            roll=128,
            pitch=128,
            throttle=self._step_axis(self._current.throttle, 0, step),
            yaw=128,
        ).sanitized()

    def _emit(self, action: DroneAction, now: float) -> DroneAction:
        action = action.sanitized()
        if self.state == ManualControlState.DISARMED and not self._is_stop_action(action):
            raise RuntimeError("refusing to emit non-stop command while disarmed")
        self._last_emit_at = now
        self._pending_ack_since = now
        return action

    def _should_emit(self, now: float) -> bool:
        if self._last_emit_at is None:
            return True
        return now - self._last_emit_at >= self.config.command_interval_seconds

    def _elapsed_update(self, now: float) -> float:
        if self._last_update_at is None:
            self._last_update_at = now
            return self.config.command_interval_seconds
        elapsed = max(0.0, now - self._last_update_at)
        self._last_update_at = now
        return elapsed

    def _axis_or_current(self, value: int | float | None, current: int) -> int:
        if value is None:
            return current
        return clamp_byte(value)

    def _throttle_or_current(self, value: int | float | None, current: int) -> int:
        if value is None:
            return current
        return max(0, min(clamp_byte(value), clamp_byte(self.config.max_throttle)))

    @staticmethod
    def _step_axis(current: int, target: int, step: int) -> int:
        if current < target:
            return min(target, current + step)
        if current > target:
            return max(target, current - step)
        return current

    @staticmethod
    def _is_stop_action(action: DroneAction) -> bool:
        return action.throttle == 0 or action.emergency_stop

    @staticmethod
    def _now(now: float | None) -> float:
        if now is None:
            return time.monotonic()
        return now
