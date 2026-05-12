from __future__ import annotations

import argparse
import signal
import threading
import time
from dataclasses import dataclass

from .actions import DroneAction, action_from_dict
from .config import DroneConfig, load_config
from .model_adapter import ModelAdapter
from .protocols import PacketProtocol, make_protocol
from .transport import UdpDroneLink, UdpTarget


@dataclass
class DroneRuntime:
    config: DroneConfig
    protocol: PacketProtocol
    link: UdpDroneLink | None
    dry_run: bool
    action: DroneAction
    lock: threading.Lock
    sent: int = 0
    errors: int = 0

    def update(self, action: DroneAction) -> None:
        with self.lock:
            self.action = action.sanitized()

    def read_action(self) -> DroneAction:
        with self.lock:
            return self.action


class SwarmController:
    def __init__(self, configs: list[DroneConfig], *, command: str, control_hz: float, model_hz: float, dry_run: bool) -> None:
        self.configs = configs
        self.command = command
        self.control_hz = control_hz
        self.model_hz = model_hz
        self.dry_run = dry_run
        self.model = ModelAdapter()
        self.running = False
        self.runtimes: list[DroneRuntime] = []
        self.threads: list[threading.Thread] = []

    def start(self) -> None:
        self.running = True
        for cfg in self.configs:
            link = None
            if not self.dry_run:
                link = UdpDroneLink(UdpTarget(cfg.ip, cfg.port, cfg.iface), bind_device=cfg.bind_device)
            runtime = DroneRuntime(
                config=cfg,
                protocol=make_protocol(cfg.protocol),
                link=link,
                dry_run=self.dry_run,
                action=DroneAction.neutral(),
                lock=threading.Lock(),
            )
            self.runtimes.append(runtime)
            thread = threading.Thread(target=self._control_loop, args=(runtime,), daemon=True)
            thread.start()
            self.threads.append(thread)

        model_thread = threading.Thread(target=self._model_loop, daemon=True)
        model_thread.start()
        self.threads.append(model_thread)

    def stop(self) -> None:
        self.running = False
        for thread in self.threads:
            thread.join(timeout=1.0)
        for runtime in self.runtimes:
            if runtime.link is not None:
                for _ in range(5):
                    runtime.link.send(runtime.protocol.build(DroneAction.motor_stop()))
                    time.sleep(0.03)
                runtime.link.close()

    def _control_loop(self, runtime: DroneRuntime) -> None:
        interval = 1.0 / self.control_hz
        while self.running:
            action = runtime.read_action()
            packet = runtime.protocol.build(action)
            if runtime.dry_run:
                if runtime.sent % max(1, int(self.control_hz)) == 0:
                    print(f"[{runtime.config.id}] {packet[:32].hex(' ')}")
            else:
                try:
                    assert runtime.link is not None
                    runtime.link.send(packet)
                except OSError as exc:
                    runtime.errors += 1
                    print(f"[{runtime.config.id}] send error: {exc}")
            runtime.sent += 1
            time.sleep(interval)

    def _model_loop(self) -> None:
        interval = 1.0 / self.model_hz
        while self.running:
            observations = [None] * len(self.runtimes)
            raw_actions = self.model.step(observations, self.command)
            if len(raw_actions) != len(self.runtimes):
                print(f"model returned {len(raw_actions)} actions for {len(self.runtimes)} drones; ignoring tick")
                time.sleep(interval)
                continue
            for runtime, raw_action in zip(self.runtimes, raw_actions):
                action = raw_action if isinstance(raw_action, DroneAction) else action_from_dict(raw_action)
                runtime.update(action)
            time.sleep(interval)


def main() -> int:
    parser = argparse.ArgumentParser(description="Run a multi-drone control loop from a JSON config.")
    parser.add_argument("--config", default="config/drones.example.json")
    parser.add_argument("--command", default="neutral hover")
    parser.add_argument("--control-hz", type=float, default=20.0)
    parser.add_argument("--model-hz", type=float, default=4.0)
    parser.add_argument("--seconds", type=float, default=0.0)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    configs = load_config(args.config)
    controller = SwarmController(
        configs,
        command=args.command,
        control_hz=args.control_hz,
        model_hz=args.model_hz,
        dry_run=args.dry_run,
    )
    running = True

    def stop(_signum: int, _frame: object) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    controller.start()
    deadline = time.monotonic() + args.seconds if args.seconds > 0 else None
    try:
        while running and (deadline is None or time.monotonic() < deadline):
            time.sleep(0.2)
    finally:
        controller.stop()

    for runtime in controller.runtimes:
        print(f"{runtime.config.id}: sent={runtime.sent} errors={runtime.errors}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

