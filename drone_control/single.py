from __future__ import annotations

import argparse
import signal
import time
from collections.abc import Iterable

from .actions import DroneAction
from .protocols import make_protocol
from .transport import UdpDroneLink, UdpTarget

COMMANDS = [
    "neutral",
    "calibrate",
    "takeoff",
    "land",
    "stop",
    "emergency_stop",
    "kill",
    "axis-test",
    "axis_test",
    "throttle-sweep",
    "throttle_sweep",
    "mix-test",
    "mix_test",
    "manual",
]


def action_for_command(command: str) -> DroneAction:
    command = command.strip().lower()
    if command == "neutral":
        return DroneAction.neutral()
    if command == "calibrate":
        return DroneAction(calibrate=True)
    if command == "takeoff":
        return DroneAction(takeoff=True)
    if command == "land":
        return DroneAction(land=True)
    if command == "stop":
        return DroneAction(throttle=0)
    if command in {"emergency_stop", "kill"}:
        return DroneAction.motor_stop()
    raise ValueError(f"unknown command {command!r}")


def axis_test_steps(amplitude: int, pulse_seconds: float, neutral_seconds: float) -> list[tuple[str, DroneAction, float]]:
    high = min(255, 128 + abs(amplitude))
    low = max(0, 128 - abs(amplitude))
    return [
        ("settle neutral", DroneAction.neutral(), neutral_seconds),
        ("throttle high", DroneAction(throttle=high), pulse_seconds),
        ("neutral", DroneAction.neutral(), neutral_seconds),
        ("pitch high", DroneAction(pitch=high), pulse_seconds),
        ("neutral", DroneAction.neutral(), neutral_seconds),
        ("pitch low", DroneAction(pitch=low), pulse_seconds),
        ("neutral", DroneAction.neutral(), neutral_seconds),
        ("yaw high", DroneAction(yaw=high), pulse_seconds),
        ("neutral", DroneAction.neutral(), neutral_seconds),
        ("yaw low", DroneAction(yaw=low), pulse_seconds),
        ("neutral", DroneAction.neutral(), neutral_seconds),
        ("roll high", DroneAction(roll=high), pulse_seconds),
        ("neutral", DroneAction.neutral(), neutral_seconds),
        ("roll low", DroneAction(roll=low), pulse_seconds),
        ("final neutral", DroneAction.neutral(), neutral_seconds),
    ]


def motor_ramp_steps(values: list[int], step_seconds: float, settle_seconds: float) -> list[tuple[str, DroneAction, float]]:
    steps: list[tuple[str, DroneAction, float]] = [
        ("settle neutral", DroneAction.neutral(), settle_seconds),
        ("motors off", DroneAction(throttle=0), 0.5),
    ]
    for throttle in values:
        steps.append((f"throttle {throttle}", DroneAction(throttle=throttle), step_seconds))
    steps.append(("motors off", DroneAction(throttle=0), 0.8))
    steps.append(("final neutral", DroneAction.neutral(), settle_seconds))
    return steps


def mix_test_steps(base_throttle: int, amplitude: int, pulse_seconds: float, neutral_seconds: float) -> list[tuple[str, DroneAction, float]]:
    high = min(255, 128 + abs(amplitude))
    low = max(0, 128 - abs(amplitude))
    return [
        ("settle neutral", DroneAction.neutral(), neutral_seconds),
        ("motors off", DroneAction(throttle=0), 0.4),
        ("base throttle", DroneAction(throttle=base_throttle), neutral_seconds),
        ("pitch high", DroneAction(pitch=high, throttle=base_throttle), pulse_seconds),
        ("base throttle", DroneAction(throttle=base_throttle), neutral_seconds),
        ("pitch low", DroneAction(pitch=low, throttle=base_throttle), pulse_seconds),
        ("base throttle", DroneAction(throttle=base_throttle), neutral_seconds),
        ("yaw high", DroneAction(yaw=high, throttle=base_throttle), pulse_seconds),
        ("base throttle", DroneAction(throttle=base_throttle), neutral_seconds),
        ("yaw low", DroneAction(yaw=low, throttle=base_throttle), pulse_seconds),
        ("base throttle", DroneAction(throttle=base_throttle), neutral_seconds),
        ("roll high", DroneAction(roll=high, throttle=base_throttle), pulse_seconds),
        ("base throttle", DroneAction(throttle=base_throttle), neutral_seconds),
        ("roll low", DroneAction(roll=low, throttle=base_throttle), pulse_seconds),
        ("motors off", DroneAction(throttle=0), 0.8),
        ("final neutral", DroneAction.neutral(), neutral_seconds),
    ]


def repeated_action_steps(action: DroneAction, seconds: float) -> Iterable[tuple[str, DroneAction, float]]:
    yield "command", action, seconds


def parse_int_list(value: str) -> list[int]:
    result = []
    for part in value.split(","):
        part = part.strip()
        if not part:
            continue
        result.append(max(0, min(255, int(part))))
    if not result:
        raise ValueError("expected at least one ramp value")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description="Run one drone command loop.")
    parser.add_argument("--iface", required=True)
    parser.add_argument("--ip", default="192.168.1.1")
    parser.add_argument("--port", type=int, default=7099)
    parser.add_argument("--protocol", default="wifi_8k_prefixed_short")
    parser.add_argument("--command", default="neutral", choices=COMMANDS)
    parser.add_argument("--hz", type=float, default=20.0)
    parser.add_argument("--seconds", type=float, default=0.0, help="0 means run until Ctrl-C.")
    parser.add_argument("--test-amplitude", type=int, default=32, help="Axis-test offset from neutral 128.")
    parser.add_argument("--pulse-seconds", type=float, default=0.45, help="Seconds per non-neutral axis-test pulse.")
    parser.add_argument("--neutral-seconds", type=float, default=0.8, help="Seconds between axis-test pulses.")
    parser.add_argument("--ramp-values", default="160,192,224,240,255", help="Comma-separated throttle bytes for throttle-sweep.")
    parser.add_argument("--ramp-step-seconds", type=float, default=0.8, help="Seconds per throttle-sweep ramp value.")
    parser.add_argument("--mix-throttle", type=int, default=224, help="Base throttle byte for mix-test.")
    parser.add_argument("--roll", type=int, default=128, help="Manual command roll byte.")
    parser.add_argument("--pitch", type=int, default=128, help="Manual command pitch byte.")
    parser.add_argument("--throttle", type=int, default=128, help="Manual command throttle byte.")
    parser.add_argument("--yaw", type=int, default=128, help="Manual command yaw byte.")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--no-bind-device", action="store_true")
    args = parser.parse_args()

    protocol = make_protocol(args.protocol)
    interval = 1.0 / args.hz
    running = True

    def stop(_signum: int, _frame: object) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    link = None
    if not args.dry_run:
        link = UdpDroneLink(UdpTarget(args.ip, args.port, args.iface), bind_device=not args.no_bind_device)

    print(
        f"single loop iface={args.iface} ip={args.ip} port={args.port} "
        f"protocol={args.protocol} command={args.command} dry_run={args.dry_run}"
    )
    if args.command in {"axis-test", "axis_test"}:
        steps = axis_test_steps(args.test_amplitude, args.pulse_seconds, args.neutral_seconds)
    elif args.command in {"throttle-sweep", "throttle_sweep"}:
        steps = motor_ramp_steps(parse_int_list(args.ramp_values), args.ramp_step_seconds, args.neutral_seconds)
    elif args.command in {"mix-test", "mix_test"}:
        steps = mix_test_steps(args.mix_throttle, args.test_amplitude, args.pulse_seconds, args.neutral_seconds)
    elif args.command == "manual":
        action = DroneAction(roll=args.roll, pitch=args.pitch, throttle=args.throttle, yaw=args.yaw).sanitized()
        steps = repeated_action_steps(action, args.seconds)
    else:
        action = action_for_command(args.command)
        steps = repeated_action_steps(action, args.seconds)

    count = 0
    keepalive = getattr(protocol, "keepalive", None)
    next_keepalive = time.monotonic()
    try:
        for label, action, seconds in steps:
            if not running:
                break
            deadline = time.monotonic() + seconds if seconds > 0 else None
            print(
                f"step={label} seconds={seconds:.2f} "
                f"roll={action.roll} pitch={action.pitch} throttle={action.throttle} yaw={action.yaw}"
            )
            while running and (deadline is None or time.monotonic() < deadline):
                packet = protocol.build(action)
                if args.dry_run:
                    if count % max(1, int(args.hz)) == 0:
                        print(packet.hex(" "))
                else:
                    assert link is not None
                    link.send(packet)
                    if callable(keepalive) and time.monotonic() >= next_keepalive:
                        link.send(keepalive())
                        next_keepalive = time.monotonic() + 1.0
                count += 1
                time.sleep(interval)
    finally:
        if link is not None:
            for _ in range(5):
                link.send(protocol.build(DroneAction(throttle=0)))
                time.sleep(0.05)
            link.close()
    print(f"sent {count} packets")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
