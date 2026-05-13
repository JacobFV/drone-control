from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drone_control.actions import DroneAction
from drone_control.manual_control import ManualControlConfig
from drone_control.manual_control import ManualControlSession
from drone_control.manual_control import ManualControlState


def test_arm_active_cap_and_slew() -> None:
    session = ManualControlSession(ManualControlConfig(max_throttle=150, command_hz=10.0))
    now = 100.0
    assert session.tick(now) is None
    assert not session.set_target_axes(throttle=180, now=now)

    session.arm(now)
    session.heartbeat(now)
    assert session.set_target_axes(roll=140, pitch=120, throttle=220, yaw=130, now=now)

    action = session.tick(now)
    assert isinstance(action, DroneAction)
    assert action.throttle <= 150
    assert action.throttle < 150
    assert action.roll == 140
    session.ack(now)

    now += 2.0
    session.heartbeat(now)
    action = session.tick(now)
    assert action is not None
    assert action.throttle == 150


def test_heartbeat_timeout_ramps_to_disarmed() -> None:
    session = ManualControlSession(
        ManualControlConfig(
            heartbeat_timeout_seconds=0.2,
            command_hz=10.0,
            ramp_down_per_second=1000.0,
            stop_confirm_commands=2,
        )
    )
    now = 200.0
    session.arm(now)
    session.heartbeat(now)
    assert session.set_target_axes(throttle=150, now=now)
    action = session.tick(now)
    assert action is not None
    session.ack(now)

    now += 0.3
    action = session.tick(now)
    assert action is not None
    assert session.state == ManualControlState.STOPPING
    session.ack(now)

    now += 0.11
    action = session.tick(now)
    assert action is not None
    assert action.throttle == 0
    assert session.state == ManualControlState.DISARMED


def test_ack_timeout_faults_with_emergency_stop() -> None:
    session = ManualControlSession(
        ManualControlConfig(ack_timeout_seconds=0.1, command_hz=50.0)
    )
    now = 300.0
    session.arm(now)
    session.heartbeat(now)
    assert session.set_target_axes(throttle=80, now=now)
    assert session.tick(now) is not None

    now += 0.2
    action = session.tick(now)
    assert action is not None
    assert session.state == ManualControlState.FAULTED
    assert session.fault_reason == "ack_timeout"
    assert action.emergency_stop
    assert action.throttle == 0


def test_explicit_stops() -> None:
    session = ManualControlSession(ManualControlConfig(command_hz=20.0))
    now = 400.0
    session.arm(now)
    session.heartbeat(now)
    assert session.set_target_axes(throttle=64, now=now)
    action = session.tick(now)
    assert action is not None
    session.ack(now)

    now += 0.1
    action = session.mode_switch_stop(now)
    assert action is not None
    assert action.throttle < 64
    session.ack(now)

    session.clear_fault()
    assert session.state == ManualControlState.DISARMED
    action = session.emergency_stop(now)
    assert action == DroneAction.motor_stop()
    assert session.state == ManualControlState.FAULTED


if __name__ == "__main__":
    test_arm_active_cap_and_slew()
    test_heartbeat_timeout_ramps_to_disarmed()
    test_ack_timeout_faults_with_emergency_stop()
    test_explicit_stops()
    print("manual control safety demo passed")
