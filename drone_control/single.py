from __future__ import annotations

import argparse
import os
import select
import signal
import termios
import time
import tty
from collections.abc import Callable
from collections.abc import Iterable
from types import SimpleNamespace

from .actions import DroneAction
from .protocols import make_protocol
from .transport import DroneLink, make_drone_link

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
    "interactive",
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


def clamp_axis(value: int) -> int:
    return max(0, min(255, value))


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


def read_key(fd: int) -> str | None:
    ready, _, _ = select.select([fd], [], [], 0)
    if not ready:
        return None
    first = os.read(fd, 1)
    if not first:
        return None
    if first == b"\x1b":
        time.sleep(0.01)
        ready, _, _ = select.select([fd], [], [], 0)
        if ready:
            rest = os.read(fd, 8)
            return (first + rest).decode(errors="ignore")
    return first.decode(errors="ignore")


def send_action(
    protocol: object,
    link: DroneLink | None,
    action: DroneAction,
    dry_run: bool,
    count: int,
) -> None:
    packet = protocol.build(action.sanitized())
    if dry_run:
        if count % 20 == 0:
            print(packet.hex(" "))
    else:
        assert link is not None
        link.send(packet)


def send_keepalive_if_due(
    protocol: object,
    link: DroneLink | None,
    dry_run: bool,
    next_keepalive: float,
) -> float:
    keepalive = getattr(protocol, "keepalive", None)
    if not dry_run and callable(keepalive) and time.monotonic() >= next_keepalive:
        assert link is not None
        link.send(keepalive())
        return time.monotonic() + 1.0
    return next_keepalive


def ramp_throttle(
    protocol: object,
    link: DroneLink | None,
    action: DroneAction,
    target: int,
    step: int,
    interval: float,
    dry_run: bool,
    count: int,
    next_keepalive: float,
) -> tuple[int, float]:
    target = clamp_axis(target)
    step = max(1, abs(step))
    while action.throttle != target:
        if action.throttle < target:
            action.throttle = min(target, action.throttle + step)
        else:
            action.throttle = max(target, action.throttle - step)
        send_action(protocol, link, action, dry_run, count)
        next_keepalive = send_keepalive_if_due(protocol, link, dry_run, next_keepalive)
        count += 1
        time.sleep(interval)
    return count, next_keepalive


def ramped_motor_stop(
    protocol: object,
    link: DroneLink | None,
    action: DroneAction,
    step: int,
    interval: float,
    dry_run: bool,
    count: int,
    next_keepalive: float,
) -> tuple[int, float]:
    action.roll = 128
    action.pitch = 128
    action.yaw = 128
    if action.throttle > 152:
        count, next_keepalive = ramp_throttle(protocol, link, action, 152, step, interval, dry_run, count, next_keepalive)
        for _ in range(4):
            send_action(protocol, link, action, dry_run, count)
            next_keepalive = send_keepalive_if_due(protocol, link, dry_run, next_keepalive)
            count += 1
            time.sleep(interval)
    count, next_keepalive = ramp_throttle(protocol, link, action, 0, step, interval, dry_run, count, next_keepalive)
    for _ in range(4):
        send_action(protocol, link, action, dry_run, count)
        next_keepalive = send_keepalive_if_due(protocol, link, dry_run, next_keepalive)
        count += 1
        time.sleep(interval)
    return count, next_keepalive


def interactive_loop(
    protocol: object,
    link: DroneLink | None,
    args: argparse.Namespace,
    interval: float,
    is_running: Callable[[], bool],
) -> int:
    action = DroneAction(throttle=clamp_axis(args.interactive_start_throttle))
    step = max(1, min(64, args.interactive_step))
    resume_throttle = clamp_axis(args.interactive_resume_throttle)
    if action.throttle > 0:
        resume_throttle = action.throttle
    next_keepalive = time.monotonic()
    count = 0

    with open("/dev/tty", "rb+", buffering=0) as terminal:
        fd = terminal.fileno()
        old_attrs = termios.tcgetattr(fd)
        tty.setcbreak(fd)
        try:
            os.write(
                fd,
                (
                    "\nInteractive controls:\n"
                    "  Up/Down: throttle +/- step\n"
                    "  Left/Right: yaw +/- step\n"
                    "  W/S: pitch +/- step    A/D: roll +/- step\n"
                    "  C: center roll/pitch/yaw    N: neutral all sticks\n"
                    "  Space: toggle ramp stop/resume    Z: direct throttle=0\n"
                    "  +/-: adjust step\n"
                    "  Esc, Enter, or Q: stop and exit\n\n"
                ).encode(),
            )
            while is_running():
                key = read_key(fd)
                if key in {"\x1b", "\r", "\n", "q", "Q"}:
                    break
                if key == "\x1b[A":
                    action.throttle = clamp_axis(action.throttle + step)
                    if action.throttle > 0:
                        resume_throttle = action.throttle
                elif key == "\x1b[B":
                    action.throttle = clamp_axis(action.throttle - step)
                    if action.throttle > 0:
                        resume_throttle = action.throttle
                elif key == "\x1b[C":
                    action.yaw = clamp_axis(action.yaw + step)
                elif key == "\x1b[D":
                    action.yaw = clamp_axis(action.yaw - step)
                elif key in {"w", "W"}:
                    action.pitch = clamp_axis(action.pitch + step)
                elif key in {"s", "S"}:
                    action.pitch = clamp_axis(action.pitch - step)
                elif key in {"d", "D"}:
                    action.roll = clamp_axis(action.roll + step)
                elif key in {"a", "A"}:
                    action.roll = clamp_axis(action.roll - step)
                elif key in {"c", "C"}:
                    action.roll = action.pitch = action.yaw = 128
                elif key in {"n", "N"}:
                    action = DroneAction.neutral()
                    resume_throttle = action.throttle
                elif key == " ":
                    if action.throttle == 0:
                        count, next_keepalive = ramp_throttle(
                            protocol,
                            link,
                            action,
                            resume_throttle,
                            step,
                            interval,
                            args.dry_run,
                            count,
                            next_keepalive,
                        )
                    else:
                        resume_throttle = action.throttle
                        count, next_keepalive = ramped_motor_stop(
                            protocol,
                            link,
                            action,
                            step,
                            interval,
                            args.dry_run,
                            count,
                            next_keepalive,
                        )
                elif key in {"z", "Z"}:
                    action = DroneAction(throttle=0)
                elif key in {"+", "="}:
                    step = min(64, step + 1)
                elif key in {"-", "_"}:
                    step = max(1, step - 1)

                send_action(protocol, link, action, args.dry_run, count)
                next_keepalive = send_keepalive_if_due(protocol, link, args.dry_run, next_keepalive)
                count += 1
                os.write(
                    fd,
                    (
                        "\r"
                        f"roll={action.roll:3d} pitch={action.pitch:3d} "
                        f"throttle={action.throttle:3d} yaw={action.yaw:3d} "
                        f"step={step:2d}   "
                    ).encode(),
                )
                time.sleep(interval)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_attrs)
            os.write(fd, b"\n")
    count, _next_keepalive = ramped_motor_stop(
        protocol,
        link,
        action,
        step,
        interval,
        args.dry_run,
        count,
        next_keepalive,
    )
    return count


def main() -> int:
    parser = argparse.ArgumentParser(description="Run one drone command loop.")
    parser.add_argument("--link-type", default="udp", choices=["udp", "esp_serial"])
    parser.add_argument("--iface", default="")
    parser.add_argument("--ip", default="192.168.1.1")
    parser.add_argument("--port", type=int, default=7099)
    parser.add_argument("--ssid", default="", help="Drone AP SSID. Required for --link-type esp_serial.")
    parser.add_argument("--password", default="", help="Drone AP password for ESP bridge, if any.")
    parser.add_argument("--serial-port", default="", help="ESP32 USB serial device for --link-type esp_serial.")
    parser.add_argument("--serial-baud", type=int, default=921600)
    parser.add_argument("--esp-connect-timeout", type=float, default=12.0)
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
    parser.add_argument("--interactive-step", type=int, default=8, help="Byte increment per interactive key press.")
    parser.add_argument("--interactive-start-throttle", type=int, default=0, help="Starting throttle byte for interactive mode.")
    parser.add_argument("--interactive-resume-throttle", type=int, default=176, help="Fallback throttle byte when Space resumes from 0.")
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
        link = make_drone_link(
            SimpleNamespace(
                link_type=args.link_type,
                iface=args.iface or None,
                ip=args.ip,
                port=args.port,
                bind_device=not args.no_bind_device,
                ssid=args.ssid,
                password=args.password,
                serial_port=args.serial_port,
                serial_baud=args.serial_baud,
                esp_connect_timeout=args.esp_connect_timeout,
            )
        )

    print(
        f"single loop link={args.link_type} iface={args.iface or '-'} serial={args.serial_port or '-'} "
        f"ssid={args.ssid or '-'} ip={args.ip} port={args.port} "
        f"protocol={args.protocol} command={args.command} dry_run={args.dry_run}"
    )
    if args.command in {"axis-test", "axis_test"}:
        steps = axis_test_steps(args.test_amplitude, args.pulse_seconds, args.neutral_seconds)
    elif args.command in {"throttle-sweep", "throttle_sweep"}:
        steps = motor_ramp_steps(parse_int_list(args.ramp_values), args.ramp_step_seconds, args.neutral_seconds)
    elif args.command in {"mix-test", "mix_test"}:
        steps = mix_test_steps(args.mix_throttle, args.test_amplitude, args.pulse_seconds, args.neutral_seconds)
    elif args.command == "interactive":
        steps = []
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
        if args.command == "interactive":
            count = interactive_loop(protocol, link, args, interval, lambda: running)
        else:
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
            for action in [DroneAction(throttle=152)] * 4 + [DroneAction(throttle=0)] * 5:
                link.send(protocol.build(action))
                time.sleep(0.05)
            link.close()
    print(f"sent {count} packets")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
