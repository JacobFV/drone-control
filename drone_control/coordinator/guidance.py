"""
Guidance command bus — the bridge between the low-frequency VLM and the
high-frequency batched controller.

The VLM runs at a low rate (seconds) and steers the swarm by writing *guidance*
into this bus via tool calls: per-drone target positions, multi-waypoint
trajectories, style vectors, and (optionally) a policy selection. The
high-frequency batched controller reads the bus every tick (20+ Hz) and folds
the resolved guidance into each drone's observation payload as conditioning:

  * ``target`` / ``trajectory`` -> ``goalRel`` (target - current position)
  * ``style``                   -> ``style`` vector
  * ``policy_id``               -> ``policyId`` (groups the batch; see manager)

Targets, trajectories and style vectors are pure conditioning, so the whole
swarm stays in ONE batched forward pass. ``select_policy`` assigns different
weights per drone, which splits the batch by policy group — supported, with that
cost made explicit.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass, field
from typing import Any

Vec3 = tuple[float, float, float]


@dataclass(slots=True)
class GuidanceState:
    target: Vec3 | None = None
    trajectory: list[Vec3] = field(default_factory=list)
    traj_index: int = 0
    loop: bool = False
    style: list[float] = field(default_factory=list)
    policy_id: str | None = None
    waypoint_radius: float = 0.6

    def resolve(self, pos: Vec3) -> tuple[Vec3 | None, list[float], str | None]:
        """Return the effective (target, style, policy_id), advancing the trajectory
        waypoint when the drone is within ``waypoint_radius`` of it."""

        target = self.target
        if self.trajectory:
            if self.traj_index >= len(self.trajectory):
                self.traj_index = 0 if self.loop else len(self.trajectory) - 1
            wp = self.trajectory[self.traj_index]
            if _dist(pos, wp) < self.waypoint_radius and self.traj_index < len(self.trajectory):
                nxt = self.traj_index + 1
                if nxt >= len(self.trajectory):
                    self.traj_index = 0 if self.loop else len(self.trajectory) - 1
                else:
                    self.traj_index = nxt
                wp = self.trajectory[self.traj_index]
            target = wp
        return target, list(self.style), self.policy_id

    def as_dict(self) -> dict[str, Any]:
        return {
            "target": list(self.target) if self.target else None,
            "trajectory": [list(w) for w in self.trajectory],
            "trajIndex": self.traj_index,
            "loop": self.loop,
            "style": list(self.style),
            "policyId": self.policy_id,
        }


class GuidanceBus:
    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._states: dict[str, GuidanceState] = {}

    def _get(self, drone_id: str) -> GuidanceState:
        state = self._states.get(drone_id)
        if state is None:
            state = GuidanceState()
            self._states[drone_id] = state
        return state

    def set_target(self, drone_id: str, target: Vec3 | None) -> None:
        with self._lock:
            state = self._get(drone_id)
            state.target = tuple(float(v) for v in target) if target is not None else None
            state.trajectory = []
            state.traj_index = 0

    def set_trajectory(self, drone_id: str, waypoints: list[Vec3], *, loop: bool = False) -> None:
        with self._lock:
            state = self._get(drone_id)
            state.trajectory = [tuple(float(v) for v in w) for w in waypoints]
            state.traj_index = 0
            state.loop = bool(loop)
            state.target = None

    def set_style(self, drone_id: str, style: list[float]) -> None:
        with self._lock:
            self._get(drone_id).style = [float(v) for v in style]

    def select_policy(self, drone_id: str, policy_id: str | None) -> None:
        with self._lock:
            self._get(drone_id).policy_id = policy_id or None

    def clear(self, drone_id: str) -> None:
        with self._lock:
            self._states.pop(drone_id, None)

    def resolve(self, drone_id: str, pos: Vec3) -> tuple[Vec3 | None, list[float], str | None]:
        with self._lock:
            state = self._states.get(drone_id)
            if state is None:
                return None, [], None
            return state.resolve(pos)

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return {drone_id: state.as_dict() for drone_id, state in self._states.items()}


def _dist(a: Vec3, b: Vec3) -> float:
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2) ** 0.5


# --------------------------------------------------------------------------- #
# VLM tool schema + application
# --------------------------------------------------------------------------- #

# Anthropic-style tool definitions the low-frequency VLM can call to steer the
# swarm. Apply the returned tool calls with apply_tool_calls(bus, calls).
GUIDANCE_TOOLS: list[dict[str, Any]] = [
    {
        "name": "set_target",
        "description": "Set a single 3D target position (metres, world frame) for a drone. "
        "The high-frequency controller flies the drone toward it. Clears any trajectory.",
        "input_schema": {
            "type": "object",
            "properties": {
                "droneId": {"type": "string"},
                "x": {"type": "number"},
                "y": {"type": "number"},
                "z": {"type": "number"},
            },
            "required": ["droneId", "x", "y", "z"],
        },
    },
    {
        "name": "set_trajectory",
        "description": "Set an ordered list of 3D waypoints for a drone to follow; it advances "
        "to the next waypoint as each is reached. Set loop=true to cycle.",
        "input_schema": {
            "type": "object",
            "properties": {
                "droneId": {"type": "string"},
                "waypoints": {
                    "type": "array",
                    "items": {"type": "array", "items": {"type": "number"}, "minItems": 3, "maxItems": 3},
                },
                "loop": {"type": "boolean"},
            },
            "required": ["droneId", "waypoints"],
        },
    },
    {
        "name": "set_style",
        "description": "Set a style/behaviour conditioning vector for a drone (e.g. aggressive vs "
        "cautious). Pure conditioning — keeps the swarm in one batched forward pass.",
        "input_schema": {
            "type": "object",
            "properties": {
                "droneId": {"type": "string"},
                "style": {"type": "array", "items": {"type": "number"}},
            },
            "required": ["droneId", "style"],
        },
    },
    {
        "name": "select_policy",
        "description": "Assign a named control policy to a drone. Drones on different policies are "
        "batched separately (this splits the batch), so prefer targets/styles when possible.",
        "input_schema": {
            "type": "object",
            "properties": {
                "droneId": {"type": "string"},
                "policyId": {"type": ["string", "null"]},
            },
            "required": ["droneId"],
        },
    },
]


def apply_tool_calls(bus: GuidanceBus, calls: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Apply VLM tool calls to the bus. Returns a per-call result log."""

    results: list[dict[str, Any]] = []
    for call in calls or []:
        name = call.get("name") or call.get("tool")
        args = call.get("arguments") or call.get("input") or call.get("args") or {}
        try:
            _apply_one(bus, str(name), dict(args))
            results.append({"tool": name, "ok": True})
        except Exception as exc:  # surface bad tool calls without crashing the loop
            results.append({"tool": name, "ok": False, "error": str(exc)})
    return results


def _apply_one(bus: GuidanceBus, name: str, args: dict[str, Any]) -> None:
    drone_id = str(args["droneId"])
    if name == "set_target":
        bus.set_target(drone_id, (float(args["x"]), float(args["y"]), float(args["z"])))
    elif name == "set_trajectory":
        bus.set_trajectory(drone_id, [tuple(w) for w in args["waypoints"]], loop=bool(args.get("loop", False)))
    elif name == "set_style":
        bus.set_style(drone_id, list(args["style"]))
    elif name == "select_policy":
        bus.select_policy(drone_id, args.get("policyId"))
    else:
        raise ValueError(f"unknown guidance tool: {name}")
