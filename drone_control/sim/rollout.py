"""
Closed-loop policy rollout in the sim.

Drives the env with an arbitrary ``policy_step`` callable that maps a list of
per-drone payloads (the same dicts the real batched VLA hub builds:
observation + goalRel + frameJpegB64 + recentActions + style) to a ``[K,4]``
byte-action array. This is exactly the deploy-time control path, so a policy
that flies here flies on the real stack. Returns goal-distance metrics.
"""

from __future__ import annotations

import base64
from collections import deque
from dataclasses import dataclass
from typing import Any, Callable

import numpy as np

from .env import SwarmEnv
from .render import CameraRenderer

PolicyStep = Callable[[list[dict[str, Any]]], np.ndarray]


@dataclass(slots=True)
class RolloutMetrics:
    start_mean_dist: float
    end_mean_dist: float
    min_mean_dist: float
    steps: int

    @property
    def improvement(self) -> float:
        return 0.0 if self.start_mean_dist == 0 else 1.0 - self.end_mean_dist / self.start_mean_dist


def build_payloads(
    env: SwarmEnv,
    frames: list[bytes] | None,
    histories: list[deque],
    style: list[float] | None,
) -> list[dict[str, Any]]:
    pos = env.state.pos.cpu()
    goal_rel = (env.goals - env.state.pos).cpu()
    payloads: list[dict[str, Any]] = []
    for i in range(env.k):
        payloads.append(
            {
                "droneId": f"sim-{i}",
                "observation": {
                    "pose": {"translation": [float(v) for v in pos[i]], "confidence": 1.0},
                    "linkState": "dry_run",
                    "confidence": 1.0,
                },
                "frameJpegB64": base64.b64encode(frames[i]).decode("ascii") if frames else None,
                "recentActions": list(histories[i]),
                "goalRel": [float(v) for v in goal_rel[i]],
                "style": style or [0.0, 0.0, 0.0, 0.0],
            }
        )
    return payloads


def run_policy_rollout(
    env: SwarmEnv,
    policy_step: PolicyStep,
    *,
    steps: int = 300,
    render: bool = True,
    style: list[float] | None = None,
) -> RolloutMetrics:
    renderer = CameraRenderer() if render else None
    histories: list[deque] = [deque(maxlen=20) for _ in range(env.k)]
    obs = env.reset()
    start = obs.goal_rel.norm(dim=1).mean().item()
    min_dist = start

    for _ in range(steps):
        frames = None
        if renderer is not None:
            frames = renderer.render(
                env.state.pos.cpu().numpy(),
                env.state.quat.cpu().numpy(),
                env.goals.cpu().numpy(),
            )
        payloads = build_payloads(env, frames, histories, style)
        actions = np.asarray(policy_step(payloads), dtype=np.float32)  # [K,4] bytes
        import torch

        obs, _reward, _done = env.step(torch.from_numpy(actions), as_bytes=True)
        for i in range(env.k):
            row = actions[i]
            histories[i].append(
                {"roll": int(row[0]), "pitch": int(row[1]), "throttle": int(row[2]), "yaw": int(row[3])}
            )
        min_dist = min(min_dist, obs.goal_rel.norm(dim=1).mean().item())

    end = obs.goal_rel.norm(dim=1).mean().item()
    return RolloutMetrics(start_mean_dist=start, end_mean_dist=end, min_mean_dist=min_dist, steps=steps)
