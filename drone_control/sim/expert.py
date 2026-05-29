"""
Analytic expert controller (teacher) for the swarm sim.

A batched PD position controller that maps the current state + goal to a
normalised ``DroneAction`` command. Used to (a) sanity-check that the env is
controllable and (b) generate teacher demonstrations for behaviour-cloning /
diffusion-policy training. It is the "ground-truth low-level policy" the learned
hi-frequency controller imitates.
"""

from __future__ import annotations

from dataclasses import dataclass

import torch

from .dynamics import QuadParams, quat_to_euler
from .env import SwarmObs


@dataclass(slots=True)
class ExpertConfig:
    kp_xy: float = 2.2
    kd_xy: float = 2.6
    kp_z: float = 4.0
    kd_z: float = 3.0
    kp_yaw: float = 1.5
    max_accel: float = 6.0


class ExpertController:
    def __init__(self, params: QuadParams, config: ExpertConfig | None = None) -> None:
        self.params = params
        self.config = config or ExpertConfig()

    def command(self, obs: SwarmObs) -> torch.Tensor:
        """Return a normalised command [K,4] (roll, pitch, throttle, yaw)."""

        c = self.config
        g = self.params.gravity
        goal_rel = obs.goal_rel
        vel = obs.vel
        # Yaw from the current attitude (quat stored xyzw in obs -> reuse state euler).
        euler = quat_to_euler(_xyzw_to_wxyz(obs.quat_xyzw))
        yaw = euler[:, 2]

        # Desired world-frame horizontal acceleration (PD to the goal).
        acc_xy = c.kp_xy * goal_rel[:, :2] - c.kd_xy * vel[:, :2]
        acc_xy = acc_xy.clamp(-c.max_accel, c.max_accel)
        ax, ay = acc_xy[:, 0], acc_xy[:, 1]

        # Rotate desired accel into the yaw-aligned body frame.
        cos_y, sin_y = torch.cos(yaw), torch.sin(yaw)
        a_bx = cos_y * ax + sin_y * ay
        a_by = -sin_y * ax + cos_y * ay

        max_tilt = self.params.max_tilt
        pitch_cmd = (a_bx / g / max_tilt).clamp(-1.0, 1.0)
        roll_cmd = (-a_by / g / max_tilt).clamp(-1.0, 1.0)

        # Altitude PD -> collective thrust -> throttle (0.5 hovers at T/W=2).
        az_des = c.kp_z * goal_rel[:, 2] - c.kd_z * vel[:, 2]
        throttle = (0.5 * (1.0 + az_des / g) / (self.params.thrust_to_weight / 2.0)).clamp(0.0, 1.0)

        # Hold heading: command a yaw rate that drives yaw -> 0.
        yaw_cmd = (-c.kp_yaw * yaw / self.params.max_yaw_rate).clamp(-1.0, 1.0)

        return torch.stack([roll_cmd, pitch_cmd, throttle, yaw_cmd], dim=1)


def _xyzw_to_wxyz(q: torch.Tensor) -> torch.Tensor:
    return torch.stack([q[:, 3], q[:, 0], q[:, 1], q[:, 2]], dim=1)
