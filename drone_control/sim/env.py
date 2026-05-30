"""
Gym-like batched swarm environment.

Vectorised over ``B`` envs x ``N`` drones (K = B*N bodies). ``reset`` and ``step``
operate on torch tensors; ``step`` accepts commands in either normalised
([-1,1]/[0,1]) or DroneAction byte (0..255) space, matching the real stack.

Observations are provided both as a structured ``SwarmObs`` (per-body fields,
goal-relative vector) for building DroneObservation-style payloads, and as a flat
``[K, obs_dim]`` tensor for RL / expert controllers.
"""

from __future__ import annotations

from dataclasses import dataclass

import torch

from .dynamics import QuadParams, QuadrotorDynamics, SwarmState, byte_to_norm, quat_to_euler, quat_xyzw
from .tasks import Task, make_task


OBS_DIM = 14  # goal_rel(3) + vel(3) + euler(3) + omega(3) + alt(1) + dist(1)


@dataclass(slots=True)
class SwarmObs:
    pos: torch.Tensor       # [K,3]
    vel: torch.Tensor       # [K,3]
    quat_xyzw: torch.Tensor  # [K,4]
    omega: torch.Tensor     # [K,3]
    goal: torch.Tensor      # [K,3]
    goal_rel: torch.Tensor  # [K,3] goal - pos
    specific_force: torch.Tensor  # [K,3] body accel (IMU)
    flat: torch.Tensor      # [K, OBS_DIM]


@dataclass(slots=True)
class EnvConfig:
    num_envs: int = 64
    num_drones: int = 1
    task: str = "goto"
    max_steps: int = 400
    spawn_radius: float = 3.0
    spawn_alt: float = 2.0
    device: str = "cpu"
    seed: int | None = None


class SwarmEnv:
    def __init__(self, config: EnvConfig | None = None, *, params: QuadParams | None = None) -> None:
        self.config = config or EnvConfig()
        self.device = torch.device(self.config.device)
        self.dyn = QuadrotorDynamics(params, device=self.device)
        self.task: Task = make_task(self.config.task)
        self.k = self.config.num_envs * self.config.num_drones
        self._gen = torch.Generator(device="cpu")
        if self.config.seed is not None:
            self._gen.manual_seed(self.config.seed)
        self.state: SwarmState = self.dyn.zeros(self.k)
        self.goals: torch.Tensor = torch.zeros((self.k, 3), device=self.device)
        self.last_command: torch.Tensor = torch.zeros((self.k, 4), device=self.device)
        self.t = 0

    @property
    def num_envs(self) -> int:
        return self.config.num_envs

    @property
    def num_drones(self) -> int:
        return self.config.num_drones

    def reset(self) -> SwarmObs:
        k = self.k
        state = self.dyn.zeros(k)
        rand = torch.rand((k, 2), generator=self._gen).to(self.device)
        state.pos[:, :2] = (rand - 0.5) * 2 * self.config.spawn_radius
        state.pos[:, 2] = self.config.spawn_alt
        self.state = state
        self.goals = self.task.sample_goals(k, self.device)
        self.last_command = torch.zeros((k, 4), device=self.device)
        self.t = 0
        return self._observe()

    def reset_done(self, done: torch.Tensor) -> None:
        """Re-spawn only the bodies whose episodes ended (for continuous rollouts)."""

        if not bool(done.any()):
            return
        idx = done.nonzero(as_tuple=True)[0]
        n = idx.numel()
        rand = torch.rand((n, 2), generator=self._gen).to(self.device)
        self.state.pos[idx, :2] = (rand - 0.5) * 2 * self.config.spawn_radius
        self.state.pos[idx, 2] = self.config.spawn_alt
        self.state.vel[idx] = 0.0
        self.state.omega[idx] = 0.0
        self.state.quat[idx] = 0.0
        self.state.quat[idx, 0] = 1.0
        self.goals[idx] = self.task.sample_goals(n, self.device)[: n]

    def step(
        self,
        command: torch.Tensor,
        *,
        as_bytes: bool = False,
        substeps: int = 2,
        ext_accel: torch.Tensor | None = None,
    ):
        """Advance one tick. ``command`` is [K,4] or [B,N,4]. Returns (obs, reward, done).

        ``ext_accel`` ([K,3] world-frame) injects airflow disturbances.
        """

        command = command.reshape(self.k, 4).to(self.device)
        norm = byte_to_norm(command) if as_bytes else command
        self.last_command = norm
        if ext_accel is not None:
            ext_accel = ext_accel.reshape(self.k, 3).to(self.device)
        self.state = self.dyn.step(self.state, norm, substeps=substeps, ext_accel=ext_accel)
        self.t += 1
        reward = self.task.reward(self.state, self.goals, self.t)
        done = self.task.done(self.state, self.goals, self.t, self.config.max_steps)
        return self._observe(), reward, done

    def set_goals(self, goals: torch.Tensor) -> None:
        self.goals = goals.reshape(self.k, 3).to(self.device)

    def _observe(self) -> SwarmObs:
        euler = quat_to_euler(self.state.quat)
        goal_rel = self.goals - self.state.pos
        sf = self.dyn.specific_force(self.state, self.last_command)
        quat_x = quat_xyzw(self.state.quat)
        dist = goal_rel.norm(dim=1, keepdim=True)
        alt = self.state.pos[:, 2:3]
        flat = torch.cat([goal_rel, self.state.vel, euler, self.state.omega, alt, dist], dim=1)
        return SwarmObs(
            pos=self.state.pos,
            vel=self.state.vel,
            quat_xyzw=quat_x,
            omega=self.state.omega,
            goal=self.goals,
            goal_rel=goal_rel,
            specific_force=sf,
            flat=flat,
        )
