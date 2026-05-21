from __future__ import annotations

import argparse
import signal
import time

from .config import load_config
from .protocols import make_protocol
from .runtime.manager import RuntimeManager, RuntimeManagerConfig


class SwarmController:
    """
    CLI facade over the production runtime manager.

    The old implementation duplicated packet loops and model parsing here. That
    made service/runtime behavior diverge from CLI behavior. This facade keeps
    the public command-line entry point but delegates lifecycle, controller
    selection, safety, events, and link output to RuntimeManager.
    """

    def __init__(self, configs: list[object], *, command: str, control_hz: float, model_hz: float, dry_run: bool) -> None:
        self.configs = configs
        self.command = command
        self.control_hz = control_hz
        self.model_hz = model_hz
        self.dry_run = dry_run
        self.manager = RuntimeManager(
            config=RuntimeManagerConfig(control_hz=control_hz, dry_run=dry_run, enable_io=not dry_run)
        )
        self.manager.configure_drones(configs)  # type: ignore[arg-type]
        self.manager.set_all_controllers(command)

    def start(self) -> None:
        self.manager.arm_all()
        self.manager.heartbeat_all()
        self.manager.start_all()

    def stop(self) -> None:
        self.manager.stop_all()

    @property
    def runtimes(self) -> list[object]:
        return self.manager.snapshot_objects()

    def print_dry_run_packets(self) -> None:
        snapshots = self.manager.snapshot_objects()
        config_by_id = {cfg.id: cfg for cfg in self.configs if hasattr(cfg, "id")}
        for snapshot in snapshots:
            action = snapshot.last_action
            if action is None:
                continue
            cfg = config_by_id.get(snapshot.drone_id)
            if cfg is None:
                continue
            packet = make_protocol(cfg.protocol).build(action)
            print(f"[{snapshot.drone_id}] {packet[:32].hex(' ')}")


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
    next_print = time.monotonic() + max(0.05, 1.0 / max(1.0, args.control_hz))
    try:
        while running and (deadline is None or time.monotonic() < deadline):
            if args.dry_run and time.monotonic() >= next_print:
                controller.print_dry_run_packets()
                next_print = time.monotonic() + 1.0
            time.sleep(0.05)
    finally:
        controller.stop()

    for snapshot in controller.runtimes:
        print(f"{snapshot.drone_id}: sent={snapshot.sent} errors={snapshot.errors}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
