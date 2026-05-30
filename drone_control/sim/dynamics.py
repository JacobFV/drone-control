"""
Batched 6-DOF quadrotor dynamics (torch, vectorised over K = envs*drones bodies).

Control mirrors how the real E99-style drones interpret RC: roll/pitch command a
desired body tilt, yaw commands a yaw rate, throttle commands collective thrust.
An inner attitude/rate PD loop turns those into body torques; Newton-Euler then
integrates rigid-body motion. Inputs use the same ``DroneAction`` byte space
(0..255, 128 neutral) as the rest of the stack, so a policy trained in sim is
directly deployable.

All tensors are shape ``[K, ...]``; the env owns the (envs, drones) layout and
flattens to K for the integrator.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Any

import torch


@dataclass(slots=True)
class QuadParams:
    mass: float = 0.035            # ~ tiny whoop / E99 class (kg)
    gravity: float = 9.81
    inertia: tuple[float, float, float] = (2.3e-5, 2.3e-5, 4.0e-5)
    drag: float = 0.10             # linear velocity drag (N per m/s)
    max_tilt: float = 0.6          # rad, full stick -> ~34 deg
    max_yaw_rate: float = 3.0      # rad/s at full yaw stick
    thrust_to_weight: float = 2.0  # throttle=1.0 -> 2x hover thrust (0.5 hovers)
    # Attitude/rate PD gains are INERTIA-NORMALISED (units of 1/s^2 and 1/s): the
    # controller commands an angular acceleration, then we multiply by inertia to
    # get torque, so gains are independent of the (tiny) inertia and stay stable
    # at the physics rate. att_kp ~ wn^2, att_kd ~ 2*zeta*wn.
    att_kp: float = 130.0          # wn ~ 11 rad/s
    att_kd: float = 18.0           # zeta ~ 0.8
    yaw_kp: float = 6.0            # yaw-rate P
    dt: float = 0.02               # 50 Hz physics
    ground_z: float = 0.0


@dataclass(slots=True)
class SwarmState:
    pos: torch.Tensor    # [K, 3] world position
    vel: torch.Tensor    # [K, 3] world linear velocity
    quat: torch.Tensor   # [K, 4] body->world, (w, x, y, z)
    omega: torch.Tensor  # [K, 3] body angular velocity

    def clone(self) -> "SwarmState":
        return SwarmState(self.pos.clone(), self.vel.clone(), self.quat.clone(), self.omega.clone())

    @property
    def k(self) -> int:
        return self.pos.shape[0]

    @property
    def device(self) -> torch.device:
        return self.pos.device


def byte_to_norm(commands: torch.Tensor) -> torch.Tensor:
    """Map DroneAction bytes [K,4] (roll,pitch,throttle,yaw, 0..255) to normalised.

    roll/pitch/yaw -> [-1, 1] (128 = neutral), throttle -> [0, 1].
    """

    roll = (commands[:, 0] - 128.0) / 128.0
    pitch = (commands[:, 1] - 128.0) / 128.0
    throttle = commands[:, 2] / 255.0
    yaw = (commands[:, 3] - 128.0) / 128.0
    return torch.stack([roll, pitch, throttle, yaw], dim=1).clamp(-1.0, 1.0)


def norm_to_byte(norm: torch.Tensor) -> torch.Tensor:
    roll = norm[:, 0] * 128.0 + 128.0
    pitch = norm[:, 1] * 128.0 + 128.0
    throttle = norm[:, 2] * 255.0
    yaw = norm[:, 3] * 128.0 + 128.0
    return torch.stack([roll, pitch, throttle, yaw], dim=1).round().clamp(0, 255)


class QuadrotorDynamics:
    def __init__(self, params: QuadParams | None = None, *, device: str | torch.device = "cpu", dtype: torch.dtype = torch.float32) -> None:
        self.params = params or QuadParams()
        self.device = torch.device(device)
        self.dtype = dtype
        p = self.params
        self.inertia = torch.tensor(p.inertia, device=self.device, dtype=dtype)
        self.inertia_inv = 1.0 / self.inertia
        self.g_vec = torch.tensor([0.0, 0.0, -p.gravity], device=self.device, dtype=dtype)
        self.hover_thrust = p.mass * p.gravity

    def zeros(self, k: int) -> SwarmState:
        z3 = torch.zeros((k, 3), device=self.device, dtype=self.dtype)
        quat = torch.zeros((k, 4), device=self.device, dtype=self.dtype)
        quat[:, 0] = 1.0
        return SwarmState(pos=z3.clone(), vel=z3.clone(), quat=quat, omega=z3.clone())

    def step(
        self,
        state: SwarmState,
        command_norm: torch.Tensor,
        *,
        substeps: int = 1,
        ext_accel: torch.Tensor | None = None,
    ) -> SwarmState:
        """Advance one control tick. ``command_norm`` is [K,4] normalised.

        ``ext_accel`` is an optional [K,3] world-frame acceleration applied to
        every body each substep — used to inject airflow (wind / fan / wake /
        downwash) disturbances from the flow field.
        """

        dt = self.params.dt / max(1, substeps)
        pos, vel, quat, omega = state.pos, state.vel, state.quat, state.omega
        roll_cmd = command_norm[:, 0]
        pitch_cmd = command_norm[:, 1]
        throttle = command_norm[:, 2].clamp(0.0, 1.0)
        yaw_cmd = command_norm[:, 3]
        thrust = throttle * (self.hover_thrust * self.params.thrust_to_weight)

        for _ in range(max(1, substeps)):
            rot = quat_to_rotmat(quat)                      # [K,3,3] body->world
            euler = quat_to_euler(quat)                     # roll, pitch, yaw
            roll, pitch = euler[:, 0], euler[:, 1]

            # Inner attitude / yaw-rate PD -> commanded angular acceleration,
            # then scale by inertia for torque (gains are inertia-normalised).
            des_roll = roll_cmd * self.params.max_tilt
            des_pitch = pitch_cmd * self.params.max_tilt
            des_yaw_rate = yaw_cmd * self.params.max_yaw_rate
            ang_accel = torch.stack(
                [
                    self.params.att_kp * (des_roll - roll) - self.params.att_kd * omega[:, 0],
                    self.params.att_kp * (des_pitch - pitch) - self.params.att_kd * omega[:, 1],
                    self.params.yaw_kp * (des_yaw_rate - omega[:, 2]),
                ],
                dim=1,
            )
            torque = self.inertia * ang_accel

            # Linear: thrust along body z, gravity, linear drag, + external
            # airflow acceleration (wind / fans / wakes / rotor downwash).
            thrust_world = rot[:, :, 2] * thrust.unsqueeze(1)
            accel = thrust_world / self.params.mass + self.g_vec - self.params.drag * vel / self.params.mass
            if ext_accel is not None:
                accel = accel + ext_accel
            vel = vel + accel * dt
            pos = pos + vel * dt

            # Angular: Euler's equation, then quaternion kinematics.
            iw = self.inertia * omega
            omega_dot = self.inertia_inv * (torque - torch.cross(omega, iw, dim=1))
            omega = omega + omega_dot * dt
            quat = integrate_quat(quat, omega, dt)

            # Ground contact: rest on the floor, kill downward motion & spin.
            below = pos[:, 2] < self.params.ground_z
            if bool(below.any()):
                pos = pos.clone()
                vel = vel.clone()
                omega = omega.clone()
                pos[below, 2] = self.params.ground_z
                landing = below & (vel[:, 2] < 0)
                vel[landing] = 0.0
                omega[landing] = 0.0

        return SwarmState(pos=pos, vel=vel, quat=quat, omega=omega)

    def specific_force(self, state: SwarmState, command_norm: torch.Tensor) -> torch.Tensor:
        """Body-frame specific force (accelerometer reading, [K,3]) for IMU obs."""

        throttle = command_norm[:, 2].clamp(0.0, 1.0)
        thrust = throttle * (self.hover_thrust * self.params.thrust_to_weight)
        # Accelerometer measures non-gravitational acceleration in the body frame:
        # thrust/m along +z minus drag, expressed in body coordinates.
        rot = quat_to_rotmat(state.quat)
        drag_world = -self.params.drag * state.vel / self.params.mass
        f_world = rot[:, :, 2] * thrust.unsqueeze(1) / self.params.mass + drag_world
        return torch.einsum("kij,kj->ki", rot.transpose(1, 2), f_world)


# --------------------------------------------------------------------------- #
# Quaternion / rotation helpers (w, x, y, z), batched
# --------------------------------------------------------------------------- #


def quat_to_rotmat(q: torch.Tensor) -> torch.Tensor:
    q = torch.nn.functional.normalize(q, dim=-1)
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    rot = torch.stack(
        [
            1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
            2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
            2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y),
        ],
        dim=1,
    )
    return rot.reshape(-1, 3, 3)


def quat_to_euler(q: torch.Tensor) -> torch.Tensor:
    """Return [K,3] roll(x), pitch(y), yaw(z) from (w,x,y,z)."""

    q = torch.nn.functional.normalize(q, dim=-1)
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    roll = torch.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = torch.asin(torch.clamp(2 * (w * y - z * x), -1.0, 1.0))
    yaw = torch.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return torch.stack([roll, pitch, yaw], dim=1)


def integrate_quat(q: torch.Tensor, omega_body: torch.Tensor, dt: float) -> torch.Tensor:
    """First-order quaternion integration with body angular velocity."""

    wx, wy, wz = omega_body[:, 0], omega_body[:, 1], omega_body[:, 2]
    zeros = torch.zeros_like(wx)
    omega_quat = torch.stack([zeros, wx, wy, wz], dim=1)
    qdot = 0.5 * quat_mul(q, omega_quat)
    return torch.nn.functional.normalize(q + qdot * dt, dim=-1)


def quat_mul(a: torch.Tensor, b: torch.Tensor) -> torch.Tensor:
    aw, ax, ay, az = a[:, 0], a[:, 1], a[:, 2], a[:, 3]
    bw, bx, by, bz = b[:, 0], b[:, 1], b[:, 2], b[:, 3]
    return torch.stack(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ],
        dim=1,
    )


def quat_xyzw(q_wxyz: torch.Tensor) -> torch.Tensor:
    """Convert (w,x,y,z) -> (x,y,z,w) for the rest of the stack's conventions."""

    return torch.stack([q_wxyz[:, 1], q_wxyz[:, 2], q_wxyz[:, 3], q_wxyz[:, 0]], dim=1)


def merge_params(params: QuadParams, **overrides: Any) -> QuadParams:
    return replace(params, **overrides)
