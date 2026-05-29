from __future__ import annotations

from typing import Any

from drone_control.sim.session import SimSession, SimSessionConfig


def _index(drone_id: str) -> int | None:
    if drone_id.startswith("sim-"):
        try:
            return int(drone_id.split("-", 1)[1])
        except ValueError:
            return None
    return None


class SimEnvironment:
    """Environment backed by the in-process swarm simulator.

    Wraps ``SimSession`` (analytic-expert swarm + synthetic camera renderer) and
    supports realtime vs. max-speed execution.
    """

    kind = "sim"

    def __init__(self, config: SimSessionConfig | None = None) -> None:
        self.session = SimSession(config)

    # -- lifecycle ---------------------------------------------------------

    def start(self) -> None:
        self.session.start(self.session.config)

    def stop(self) -> None:
        self.session.stop()

    # -- drones / streams --------------------------------------------------

    def drone_ids(self) -> list[str]:
        return [f"sim-{i}" for i in range(self.session.config.num_drones)]

    def latest_frame(self, drone_id: str) -> bytes | None:
        idx = _index(drone_id)
        return self.session.frame(idx) if idx is not None else None

    def latest_pose(self, drone_id: str) -> dict[str, Any] | None:
        idx = _index(drone_id)
        return self.session.latest_pose(idx) if idx is not None else None

    def trajectories(self) -> list[dict[str, Any]]:
        return self.session.trajectories().get("drones", [])

    def world_model_status(self) -> dict[str, Any]:
        return {
            "available": False,
            "running": False,
            "reason": "the simulator does not build a live splat",
        }

    def set_speed(self, mode: str) -> None:
        self.session.set_max_speed(mode == "max")

    def status(self) -> dict[str, Any]:
        status = self.session.status()
        status["speed"] = "max" if self.session.config.max_speed else "realtime"
        return status
