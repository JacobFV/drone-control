"""
Swarm tasks: goal sampling, reward, and termination over the batched state.

Each task operates on the flattened ``K = envs*drones`` body layout and returns
per-body tensors. Goals are 3D target positions (the unit the guidance command
bus also speaks), so a goal-conditioned policy trained here consumes the same
target signal the VLM injects at deploy time.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import torch

from .dynamics import SwarmState


class Task(Protocol):
    name: str

    def sample_goals(self, k: int, device: torch.device) -> torch.Tensor: ...
    def reward(self, state: SwarmState, goals: torch.Tensor, t: int) -> torch.Tensor: ...
    def done(self, state: SwarmState, goals: torch.Tensor, t: int, max_steps: int) -> torch.Tensor: ...


def _out_of_bounds(state: SwarmState, radius: float = 30.0) -> torch.Tensor:
    horiz = state.pos[:, :2].norm(dim=1) > radius
    high = state.pos[:, 2] > radius
    return horiz | high


@dataclass(slots=True)
class HoverTask:
    altitude: float = 3.0
    spread: float = 4.0
    name: str = "hover"

    def sample_goals(self, k: int, device: torch.device) -> torch.Tensor:
        goals = torch.zeros((k, 3), device=device)
        goals[:, :2] = (torch.rand((k, 2), device=device) - 0.5) * 2 * self.spread
        goals[:, 2] = self.altitude
        return goals

    def reward(self, state: SwarmState, goals: torch.Tensor, t: int) -> torch.Tensor:
        dist = (state.pos - goals).norm(dim=1)
        upright = state.quat[:, 0].abs()  # |w| ~ 1 when level
        return -dist - 0.05 * state.vel.norm(dim=1) + 0.1 * upright

    def done(self, state: SwarmState, goals: torch.Tensor, t: int, max_steps: int) -> torch.Tensor:
        return _out_of_bounds(state) | torch.full((state.k,), t >= max_steps, device=state.device, dtype=torch.bool)


@dataclass(slots=True)
class GotoTask:
    """Fly to a randomly placed waypoint and hold."""

    box: float = 8.0
    min_alt: float = 1.0
    max_alt: float = 6.0
    name: str = "goto"

    def sample_goals(self, k: int, device: torch.device) -> torch.Tensor:
        goals = torch.zeros((k, 3), device=device)
        goals[:, :2] = (torch.rand((k, 2), device=device) - 0.5) * 2 * self.box
        goals[:, 2] = self.min_alt + torch.rand(k, device=device) * (self.max_alt - self.min_alt)
        return goals

    def reward(self, state: SwarmState, goals: torch.Tensor, t: int) -> torch.Tensor:
        dist = (state.pos - goals).norm(dim=1)
        reached = (dist < 0.5).float()
        return -dist - 0.02 * state.vel.norm(dim=1) + reached

    def done(self, state: SwarmState, goals: torch.Tensor, t: int, max_steps: int) -> torch.Tensor:
        return _out_of_bounds(state) | torch.full((state.k,), t >= max_steps, device=state.device, dtype=torch.bool)


@dataclass(slots=True)
class FormationTask:
    """Each drone holds a slot offset from a shared, slowly moving anchor.

    The goal tensor is the per-drone slot position; the env's drone layout decides
    the offsets so a whole env's drones converge to a formation.
    """

    radius: float = 2.0
    altitude: float = 3.0
    name: str = "formation"

    def sample_goals(self, k: int, device: torch.device) -> torch.Tensor:
        # Ring of slots; callers that know (envs, drones) can override, but a
        # deterministic ring by index is a reasonable default.
        idx = torch.arange(k, device=device)
        angle = idx.float() * 2.399963  # golden-angle spread
        goals = torch.stack(
            [self.radius * torch.cos(angle), self.radius * torch.sin(angle), torch.full_like(angle, self.altitude)],
            dim=1,
        )
        return goals

    def reward(self, state: SwarmState, goals: torch.Tensor, t: int) -> torch.Tensor:
        dist = (state.pos - goals).norm(dim=1)
        return -dist - 0.03 * state.vel.norm(dim=1)

    def done(self, state: SwarmState, goals: torch.Tensor, t: int, max_steps: int) -> torch.Tensor:
        return _out_of_bounds(state) | torch.full((state.k,), t >= max_steps, device=state.device, dtype=torch.bool)


TASKS: dict[str, type] = {"hover": HoverTask, "goto": GotoTask, "formation": FormationTask}


def make_task(name: str) -> Task:
    key = name.strip().lower()
    if key not in TASKS:
        raise ValueError(f"unknown task: {name} (have {sorted(TASKS)})")
    return TASKS[key]()  # type: ignore[return-value]
