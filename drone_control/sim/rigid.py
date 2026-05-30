"""
Free rigid-body dynamics for the swarm sim — crates, pallets, debris that the
airflow field can blow around.

Each body is an oriented box integrated with simple semi-implicit Euler. It
feels gravity and an aerodynamic force from the local wind (sampled from the
same :class:`~drone_control.sim.flow.FlowField` the drones and cloth feel),

    F = c_aero * |v_rel| * v_rel,   v_rel = wind - body velocity,

with ``c_aero = 0.5 * rho * Cd * A``. A small offset-face wind sample produces a
tumbling torque so gusts make the bodies roll, not just slide. Bodies bounce off
the ground plane (z=0) with restitution + tangential friction.

Everything is batched numpy float64 over ``M`` bodies and fully deterministic
(seeded RNG only). The renderer paints each body as six shaded quads, the same
``(corners, color, label)`` shape it already uses for static boxes.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from drone_control.sim.flow import AIR_DENSITY

GRAVITY = np.array([0.0, 0.0, -9.81], dtype=np.float64)

# Stability clamps.
_MAX_VEL = 30.0    # m/s
_MAX_OMEGA = 8.0   # rad/s


@dataclass(slots=True)
class RigidSpec:
    """Scene-authored description of one free rigid body (pure data)."""

    label: str
    color: tuple[int, int, int]
    size: tuple[float, float, float]            # box full dimensions (m)
    mass: float = 0.5                           # kg
    pos: tuple[float, float, float] = (0.0, 0.0, 1.0)   # initial center
    vel: tuple[float, float, float] = (0.0, 0.0, 0.0)
    restitution: float = 0.35                   # bounce on ground contact
    friction: float = 0.5                       # tangential damping on contact
    drag_cd: float = 1.05
    spin: tuple[float, float, float] = (0.0, 0.0, 0.0)  # initial body omega (rad/s)


# --------------------------------------------------------------------- quaternion helpers


def _quat_mul(q: np.ndarray, r: np.ndarray) -> np.ndarray:
    """Hamilton product of two batches of (w,x,y,z) quaternions [M,4]."""
    w0, x0, y0, z0 = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    w1, x1, y1, z1 = r[:, 0], r[:, 1], r[:, 2], r[:, 3]
    return np.stack(
        [
            w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1,
            w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1,
            w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1,
            w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1,
        ],
        axis=1,
    )


def _quat_normalize(q: np.ndarray) -> np.ndarray:
    """Renormalize a batch of quaternions [M,4]; fall back to identity if degenerate."""
    n = np.linalg.norm(q, axis=1, keepdims=True)
    out = np.where(n > 1e-9, q / np.maximum(n, 1e-9), q)
    bad = (n[:, 0] <= 1e-9)
    if np.any(bad):
        out[bad] = np.array([1.0, 0.0, 0.0, 0.0])
    return out


def _quat_to_rotmat(q: np.ndarray) -> np.ndarray:
    """Batch of (w,x,y,z) quaternions [M,4] -> rotation matrices [M,3,3]."""
    q = _quat_normalize(np.asarray(q, dtype=np.float64))
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    R = np.empty((q.shape[0], 3, 3), dtype=np.float64)
    R[:, 0, 0] = 1 - 2 * (y * y + z * z)
    R[:, 0, 1] = 2 * (x * y - z * w)
    R[:, 0, 2] = 2 * (x * z + y * w)
    R[:, 1, 0] = 2 * (x * y + z * w)
    R[:, 1, 1] = 1 - 2 * (x * x + z * z)
    R[:, 1, 2] = 2 * (y * z - x * w)
    R[:, 2, 0] = 2 * (x * z - y * w)
    R[:, 2, 1] = 2 * (y * z + x * w)
    R[:, 2, 2] = 1 - 2 * (x * x + y * y)
    return R


def _clamp_rows(v: np.ndarray, limit: float) -> np.ndarray:
    """Scale any row whose magnitude exceeds ``limit`` back down to it."""
    if v.size == 0:
        return v
    mag = np.linalg.norm(v, axis=1, keepdims=True)
    scale = np.minimum(1.0, limit / np.maximum(mag, 1e-9))
    return v * scale


# Per-face shading and the eight unit-cube corner signs, defined once.
# Faces given as (axis, sign) of the outward normal in body frame.
_FACE_AXES = [
    (2, +1),  # top
    (2, -1),  # bottom
    (0, +1),  # +x side
    (0, -1),  # -x side
    (1, +1),  # +y side
    (1, -1),  # -y side
]
# Corner ordering (CCW) for each face, as (sx,sy,sz) sign triples.
_FACE_CORNERS = {
    (2, +1): [(-1, -1, +1), (+1, -1, +1), (+1, +1, +1), (-1, +1, +1)],
    (2, -1): [(-1, -1, -1), (-1, +1, -1), (+1, +1, -1), (+1, -1, -1)],
    (0, +1): [(+1, -1, -1), (+1, +1, -1), (+1, +1, +1), (+1, -1, +1)],
    (0, -1): [(-1, -1, -1), (-1, -1, +1), (-1, +1, +1), (-1, +1, -1)],
    (1, +1): [(-1, +1, -1), (-1, +1, +1), (+1, +1, +1), (+1, +1, -1)],
    (1, -1): [(-1, -1, -1), (+1, -1, -1), (+1, -1, +1), (-1, -1, +1)],
}


class RigidWorld:
    """Batched free rigid boxes blown around by an external wind field."""

    def __init__(self, specs: list[RigidSpec], *, seed: int = 0) -> None:
        self._rng = np.random.default_rng(seed)
        self.specs = list(specs)
        M = len(self.specs)
        self.M = M

        self.labels = [s.label for s in self.specs]
        self.colors = [tuple(int(c) for c in s.color) for s in self.specs]

        if M == 0:
            # Empty world: every array is shaped so the batched ops are no-ops.
            self.pos = np.zeros((0, 3))
            self.vel = np.zeros((0, 3))
            self.quat = np.zeros((0, 4))
            self.omega = np.zeros((0, 3))
            self.size = np.zeros((0, 3))
            self.mass = np.zeros((0,))
            self.inv_inertia = np.zeros((0, 3))
            self.restitution = np.zeros((0,))
            self.friction = np.zeros((0,))
            self.c_aero = np.zeros((0,))
            self._mean_inv_inertia = np.zeros((0,))
            return

        self.pos = np.array([s.pos for s in self.specs], dtype=np.float64)
        self.vel = np.array([s.vel for s in self.specs], dtype=np.float64)
        self.size = np.array([s.size for s in self.specs], dtype=np.float64)
        self.mass = np.maximum(np.array([s.mass for s in self.specs], dtype=np.float64), 1e-6)
        self.restitution = np.clip(
            np.array([s.restitution for s in self.specs], dtype=np.float64), 0.0, 1.0
        )
        self.friction = np.clip(
            np.array([s.friction for s in self.specs], dtype=np.float64), 0.0, 1.0
        )

        # Quaternions start at identity (w,x,y,z) = (1,0,0,0); spin -> body omega.
        self.quat = np.tile(np.array([1.0, 0.0, 0.0, 0.0]), (M, 1))
        self.omega = np.array([s.spin for s in self.specs], dtype=np.float64)

        # Solid-box principal inertia, stored inverted (guard against zero size).
        sx, sy, sz = self.size[:, 0], self.size[:, 1], self.size[:, 2]
        Ix = self.mass * (sy * sy + sz * sz) / 12.0
        Iy = self.mass * (sx * sx + sz * sz) / 12.0
        Iz = self.mass * (sx * sx + sy * sy) / 12.0
        I = np.stack([Ix, Iy, Iz], axis=1)
        self.inv_inertia = np.where(I > 1e-9, 1.0 / np.maximum(I, 1e-9), 0.0)
        self._mean_inv_inertia = np.mean(self.inv_inertia, axis=1)

        # Aero coefficient c_aero = 0.5 * rho * Cd * A, A = mean cross-section area.
        cd = np.array([s.drag_cd for s in self.specs], dtype=np.float64)
        area = (sy * sz + sx * sz + sx * sy) / 3.0
        self.c_aero = 0.5 * AIR_DENSITY * cd * area

    # ------------------------------------------------------------------- step

    def step(self, dt: float, wind_at) -> None:
        """Advance ``dt`` seconds. ``wind_at(points[N,3]) -> wind[N,3]``."""
        if self.M == 0 or dt <= 0:
            return

        # --- aerodynamic force at body centers -----------------------------
        w = np.atleast_2d(np.asarray(wind_at(self.pos), dtype=np.float64))
        w = np.nan_to_num(w, nan=0.0, posinf=_MAX_VEL, neginf=-_MAX_VEL)
        v_rel = w - self.vel
        speed = np.linalg.norm(v_rel, axis=1, keepdims=True)
        force_aero = self.c_aero[:, None] * speed * v_rel
        accel_aero = force_aero / self.mass[:, None]

        # --- linear integration (semi-implicit Euler) ----------------------
        self.vel += (GRAVITY[None, :] + accel_aero) * dt
        self.pos += self.vel * dt

        # --- tumbling torque from an offset wind sample --------------------
        # Sample wind at the +x face center; the aero force there, applied at a
        # lever arm from the center, makes the body tumble. Kept deliberately
        # small and stable.
        R = _quat_to_rotmat(self.quat)
        arm_body = np.zeros((self.M, 3))
        arm_body[:, 0] = self.size[:, 0] * 0.5
        arm_world = np.einsum("mij,mj->mi", R, arm_body)
        face_pt = self.pos + arm_world
        w_face = np.atleast_2d(np.asarray(wind_at(face_pt), dtype=np.float64))
        w_face = np.nan_to_num(w_face, nan=0.0, posinf=_MAX_VEL, neginf=-_MAX_VEL)
        vrel_face = w_face - self.vel
        sp_face = np.linalg.norm(vrel_face, axis=1, keepdims=True)
        force_face = 0.08 * self.c_aero[:, None] * sp_face * vrel_face  # small gain
        torque = np.cross(arm_world, force_face)

        # World-approx angular update: scale torque by mean inverse inertia.
        self.omega += torque * self._mean_inv_inertia[:, None] * dt
        self.omega = _clamp_rows(self.omega, _MAX_OMEGA)

        # First-order quaternion integration: q += 0.5 * q * (0,wx,wy,wz) * dt.
        omega_q = np.zeros((self.M, 4))
        omega_q[:, 1:] = self.omega
        self.quat = self.quat + 0.5 * _quat_mul(self.quat, omega_q) * dt
        self.quat = _quat_normalize(self.quat)

        # --- light global damping ------------------------------------------
        self.vel *= 0.999
        self.omega *= 0.995

        # --- ground contact (plane z=0) ------------------------------------
        # Cheap support radius: half the box height.
        half_h = self.size[:, 2] * 0.5
        rest_z = half_h
        contact = self.pos[:, 2] < rest_z
        if np.any(contact):
            # Sit on/above the ground.
            self.pos[:, 2] = np.where(contact, np.maximum(self.pos[:, 2], rest_z), self.pos[:, 2])
            # Reflect downward velocity with restitution.
            descending = contact & (self.vel[:, 2] < 0.0)
            self.vel[:, 2] = np.where(
                descending, -self.restitution * self.vel[:, 2], self.vel[:, 2]
            )
            # Tangential friction on the contacting bodies.
            fric = np.where(contact, 1.0 - self.friction, 1.0)
            self.vel[:, 0] *= fric
            self.vel[:, 1] *= fric
            # Damp spin on contact.
            self.omega *= np.where(contact, 0.7, 1.0)[:, None]

        # --- stability clamps + sanitize -----------------------------------
        self.vel = _clamp_rows(self.vel, _MAX_VEL)
        self.omega = _clamp_rows(self.omega, _MAX_OMEGA)
        self._sanitize()

    def _sanitize(self) -> None:
        """Send any non-finite body back to a safe resting state."""
        bad = ~(
            np.isfinite(self.pos).all(axis=1)
            & np.isfinite(self.vel).all(axis=1)
            & np.isfinite(self.quat).all(axis=1)
            & np.isfinite(self.omega).all(axis=1)
        )
        if np.any(bad):
            self.pos[bad] = np.array([0.0, 0.0, 1.0])
            self.vel[bad] = 0.0
            self.quat[bad] = np.array([1.0, 0.0, 0.0, 0.0])
            self.omega[bad] = 0.0

    # ----------------------------------------------------------------- render

    def face_quads(self) -> list[tuple[list[np.ndarray], tuple[int, int, int], str]]:
        """Six shaded quads per body, as ``(corners, color, label)`` for the rasteriser."""
        out: list[tuple[list[np.ndarray], tuple[int, int, int], str]] = []
        if self.M == 0:
            return out

        R = _quat_to_rotmat(self.quat)
        half = self.size * 0.5  # [M,3]

        # Brightness by face, picked from the world-space normal of each face.
        for m in range(self.M):
            Rm = R[m]
            base = np.array(self.colors[m], dtype=np.float64)
            for axis, sign in _FACE_AXES:
                # World-space outward normal of this face.
                normal_body = np.zeros(3)
                normal_body[axis] = float(sign)
                normal_world = Rm @ normal_body
                # Shade by where the face points: up brightest, down darkest,
                # sides in between (slight split so adjacent sides read apart).
                nz = float(normal_world[2])
                if nz > 0.5:
                    shade = 1.0
                elif nz < -0.5:
                    shade = 0.5
                else:
                    shade = 0.8 if axis == 0 else 0.68
                color = tuple(int(c) for c in np.clip(base * shade, 0, 255))

                corners = []
                for sgn in _FACE_CORNERS[(axis, sign)]:
                    local = np.array(sgn, dtype=np.float64) * half[m]
                    corners.append(self.pos[m] + Rm @ local)
                out.append((corners, color, self.labels[m]))
        return out

    def centers(self) -> np.ndarray:
        """Body center positions [M,3] (used for wakes)."""
        return self.pos.copy()
