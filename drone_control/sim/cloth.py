"""
Mass-spring cloth flag — a small soft body coupled to the airflow field.

A grid of point masses joined by structural / shear springs, pinned along its
mast edge, integrated with Verlet + Provot constraint relaxation. Per node it
feels gravity and an aerodynamic force from the local wind (sampled from the
same :class:`~drone_control.sim.flow.FlowField` the drones feel), so the flag
streams, snaps, and ripples with gusts. Deterministic given its initial layout.

Kept intentionally small (a few dozen nodes) so it runs inside the live sim
loop; the renderer projects the deformed grid as shaded quads.
"""

from __future__ import annotations

import numpy as np


def _clamp_rows(v: np.ndarray, limit: float) -> np.ndarray:
    """Scale any row whose magnitude exceeds ``limit`` back down to it."""
    mag = np.linalg.norm(v, axis=1, keepdims=True)
    scale = np.minimum(1.0, limit / np.maximum(mag, 1e-9))
    return v * scale


class ClothFlag:
    def __init__(
        self,
        mast_top: tuple[float, float, float],
        flag_dir: tuple[float, float, float],
        *,
        nx: int = 8,
        ny: int = 5,
        width: float = 2.8,
        height: float = 1.5,
        wind_response: float = 7.0,
        color: tuple[int, int, int] = (210, 70, 70),
        label: str = "flag",
    ) -> None:
        self.nx, self.ny = nx, ny
        self.color = color
        self.label = label
        self.wind_response = wind_response
        self.dx = width / max(1, nx - 1)
        self.dy = height / max(1, ny - 1)

        direction = np.asarray(flag_dir, dtype=float)
        n = float(np.linalg.norm(direction))
        direction = direction / n if n > 1e-9 else np.array([1.0, 0.0, 0.0])
        down = np.array([0.0, 0.0, -1.0])
        top = np.asarray(mast_top, dtype=float)

        grid = np.zeros((ny, nx, 3))
        for r in range(ny):
            for c in range(nx):
                grid[r, c] = top + direction * (self.dx * c) + down * (self.dy * r)
        self.pos = grid.reshape(-1, 3).copy()
        self.prev = self.pos.copy()

        # Mast edge (column 0) is pinned.
        self.pinned = np.zeros(nx * ny, dtype=bool)
        for r in range(ny):
            self.pinned[r * nx + 0] = True
        self.anchor = self.pos[self.pinned].copy()

        # Spring pairs (structural + shear) with rest lengths.
        pairs: list[tuple[int, int, float]] = []

        def idx(r: int, c: int) -> int:
            return r * nx + c

        for r in range(ny):
            for c in range(nx):
                if c + 1 < nx:
                    pairs.append((idx(r, c), idx(r, c + 1), self.dx))
                if r + 1 < ny:
                    pairs.append((idx(r, c), idx(r + 1, c), self.dy))
                if c + 1 < nx and r + 1 < ny:
                    diag = float(np.hypot(self.dx, self.dy))
                    pairs.append((idx(r, c), idx(r + 1, c + 1), diag))
                    pairs.append((idx(r + 1, c), idx(r, c + 1), diag))
        self._pair_i = np.array([p[0] for p in pairs])
        self._pair_j = np.array([p[1] for p in pairs])
        self._rest = np.array([p[2] for p in pairs])
        # Per-node constraint degree — Jacobi relaxation must average the
        # accumulated corrections by it, or shared nodes over-correct and blow up.
        deg = np.zeros(nx * ny)
        np.add.at(deg, self._pair_i, 1.0)
        np.add.at(deg, self._pair_j, 1.0)
        self._deg = np.maximum(deg, 1.0)[:, None]

    def step(self, dt: float, wind_nodes: np.ndarray, *, gravity: float = 9.81) -> None:
        if dt <= 0:
            return
        wind = np.nan_to_num(np.asarray(wind_nodes, dtype=float), nan=0.0, posinf=30.0, neginf=-30.0)

        # Verlet inertia term, clamped so a single bad step can't run away.
        inertia = _clamp_rows(self.pos - self.prev, 0.4)
        vel = inertia / dt
        vel = _clamp_rows(vel, 20.0)            # node speed cap
        v_rel = _clamp_rows(wind - vel, 20.0)   # relative wind cap
        speed = np.linalg.norm(v_rel, axis=1, keepdims=True)
        acc = self.wind_response * speed * v_rel
        acc[:, 2] -= gravity
        acc = _clamp_rows(acc, 80.0)            # acceleration cap

        new = self.pos + inertia * 0.96 + acc * (dt * dt)
        new[self.pinned] = self.anchor
        self.prev = self.pos
        self.pos = new
        self._relax(iterations=3)

    def _relax(self, iterations: int = 3) -> None:
        for _ in range(iterations):
            d = self.pos[self._pair_j] - self.pos[self._pair_i]
            dist = np.linalg.norm(d, axis=1)
            dist = np.where(dist < 1e-6, 1e-6, dist)
            corr = (dist - self._rest) / dist
            shift = 0.5 * corr[:, None] * d
            # Accumulate symmetric corrections, average by per-node constraint
            # degree (Jacobi), then apply (pinned nodes frozen).
            delta = np.zeros_like(self.pos)
            np.add.at(delta, self._pair_i, shift)
            np.add.at(delta, self._pair_j, -shift)
            self.pos += (delta / self._deg) * (~self.pinned)[:, None]
            self.pos[self.pinned] = self.anchor

    def grid(self) -> np.ndarray:
        """Current node positions as [ny, nx, 3]."""
        return self.pos.reshape(self.ny, self.nx, 3)
