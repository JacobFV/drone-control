"""
Atmospheric particle effects for the swarm sim.

Lightweight, purely-visual tracer particles that are advected by the same
airflow field the drones feel. Two flavours:

* **dust** — scattered over a wide ground disk, drifts with the wind, no
  buoyancy; settles back to the disk when it ages out or sinks below ground;
* **smoke** — emitted from a tight source, rises buoyantly while being carried
  by the wind, respawning at the source as it ages.

Particles are *tracers*: they don't push back on the air or each other. Each
particle relaxes its velocity toward the local wind, gets a touch of buoyancy
(smoke) and a tiny deterministic turbulence jitter, then integrates forward.
When a particle dies (too old, or fell through the floor) it respawns in its
emitter's spawn region.

Everything is numpy float64 in the world frame (z up, ground at z=0) and fully
deterministic given a seed — respawns and jitter all come from a single seeded
``np.random.default_rng`` / per-particle phases, never wall-clock randomness.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(slots=True)
class ParticleSpec:
    """Scene-authored emitter description (pure data, like ``FlowSpec``)."""

    kind: str  # "dust" | "smoke"
    pos: tuple[float, float, float]  # emitter origin (world frame)
    count: int = 60  # particles kept alive for this emitter
    color: tuple[int, int, int] = (200, 195, 180)
    spawn_radius: float = 6.0  # dust spreads over a disk; smoke a small radius
    lifetime: float = 4.0  # seconds before a particle respawns
    rise: float = 0.0  # buoyancy (m/s^2 up); smoke > 0, dust ~ 0
    size: float = 1.4  # screen px radius hint at z = 1


# Sanity bounds so a runaway wind field can never blow the buffers up.
_POS_BOUND = 60.0
_VEL_BOUND = 25.0
_RELAX_K = 3.0  # how fast a tracer's velocity chases the local wind


class ParticleField:
    """A batched set of advected tracer particles across several emitters."""

    def __init__(self, specs: list[ParticleSpec], *, seed: int = 0) -> None:
        self.specs = list(specs or [])
        self._rng = np.random.default_rng(seed)

        # Total particle count across every emitter.
        P = int(sum(max(0, int(s.count)) for s in self.specs))
        self._P = P

        # Batched per-particle state.
        self.pos = np.zeros((P, 3), dtype=np.float64)
        self.vel = np.zeros((P, 3), dtype=np.float64)
        self.age = np.zeros(P, dtype=np.float64)
        self.life = np.full(P, 1.0, dtype=np.float64)
        self.emitter_index = np.zeros(P, dtype=np.int64)
        self.color = np.zeros((P, 3), dtype=np.float64)
        self.size = np.full(P, 1.0, dtype=np.float64)

        # Per-particle deterministic turbulence phases (advanced by age, not by
        # fresh randomness each step) and a per-emitter buoyancy lookup.
        self._phase = self._rng.uniform(0.0, 2.0 * np.pi, size=(P, 3))
        self._rise = np.zeros(P, dtype=np.float64)

        # Home/spawn parameters per particle so respawns stay in the right
        # region. We store the emitter origin and its spawn geometry inline.
        self._home = np.zeros((P, 3), dtype=np.float64)
        self._spawn_radius = np.zeros(P, dtype=np.float64)
        self._is_smoke = np.zeros(P, dtype=bool)

        if P == 0:
            return

        # Lay out each emitter's slice and fill its constant per-particle data.
        cursor = 0
        for ei, spec in enumerate(self.specs):
            n = max(0, int(spec.count))
            if n == 0:
                continue
            sl = slice(cursor, cursor + n)
            cursor += n

            origin = np.asarray(spec.pos, dtype=np.float64)
            is_smoke = spec.kind == "smoke"
            self.emitter_index[sl] = ei
            self._home[sl] = origin[None, :]
            self._spawn_radius[sl] = float(spec.spawn_radius)
            self._is_smoke[sl] = is_smoke
            self._rise[sl] = float(spec.rise)
            self.life[sl] = max(0.05, float(spec.lifetime))
            self.color[sl] = np.asarray(spec.color, dtype=np.float64)[None, :]
            self.size[sl] = float(spec.size)

            # Scatter initial positions across the spawn region so the field
            # already looks established on frame zero.
            self.pos[sl] = self._spawn_positions(sl)
            # Spread ages across [0, life) so particles don't all die at once.
            self.age[sl] = self._rng.uniform(0.0, 1.0, size=n) * self.life[sl]

    # ----------------------------------------------------------- spawn helper

    def _spawn_positions(self, idx) -> np.ndarray:
        """Fresh positions for the particles selected by ``idx``.

        Dust scatters over a flat disk of ``spawn_radius`` near the ground;
        smoke clusters tightly around the emitter origin. Uses the seeded rng.
        """
        home = self._home[idx]
        radius = self._spawn_radius[idx]
        is_smoke = self._is_smoke[idx]
        n = home.shape[0]
        if n == 0:
            return home.copy()

        # Uniform sample on a disk (sqrt for area-uniformity).
        theta = self._rng.uniform(0.0, 2.0 * np.pi, size=n)
        r = np.sqrt(self._rng.uniform(0.0, 1.0, size=n)) * radius
        dx = r * np.cos(theta)
        dy = r * np.sin(theta)

        out = home.copy()
        # Smoke uses a much tighter horizontal spread around the source.
        smoke_scale = 0.15
        out[:, 0] += np.where(is_smoke, dx * smoke_scale, dx)
        out[:, 1] += np.where(is_smoke, dy * smoke_scale, dy)
        # Dust hugs the ground (small jitter up); smoke starts at the source.
        dust_z = self._rng.uniform(0.0, 0.4, size=n)
        smoke_z = self._rng.uniform(0.0, 0.3, size=n)
        out[:, 2] += np.where(is_smoke, smoke_z, dust_z)
        return out

    # ------------------------------------------------------------------- step

    def step(self, dt: float, wind_at) -> None:
        """Advance the field by ``dt`` seconds.

        ``wind_at(points[N,3]) -> wind[N,3]`` supplies the local airflow.
        """
        if self._P == 0:
            return
        dt = float(dt)
        if dt <= 0.0:
            return

        # Local wind for every particle.
        wind = np.asarray(wind_at(self.pos), dtype=np.float64)
        wind = np.nan_to_num(wind, nan=0.0, posinf=0.0, neginf=0.0)
        if wind.shape != self.vel.shape:
            wind = np.zeros_like(self.vel)

        # Velocity relaxes toward the local wind (light tracer).
        relax = float(np.clip(dt * _RELAX_K, 0.0, 1.0))
        self.vel += (wind - self.vel) * relax

        # Buoyancy on z for emitters with rise (smoke).
        self.vel[:, 2] += self._rise * dt

        # Tiny deterministic turbulence jitter: per-particle phases advanced by
        # age, so it's smooth and reproducible (no fresh randomness per step).
        t = self.age[:, None]
        jitter = np.stack(
            [
                np.sin(2.3 * t[:, 0] + self._phase[:, 0]),
                np.cos(1.9 * t[:, 0] + self._phase[:, 1]),
                np.sin(2.7 * t[:, 0] + self._phase[:, 2]),
            ],
            axis=1,
        )
        self.vel += 0.25 * jitter * dt

        # Clamp velocity, integrate, age.
        np.clip(self.vel, -_VEL_BOUND, _VEL_BOUND, out=self.vel)
        self.pos += self.vel * dt
        self.age += dt

        # Respawn dead particles: too old, or fell through the floor.
        dead = (self.age > self.life) | (self.pos[:, 2] < 0.0)
        if np.any(dead):
            self.pos[dead] = self._spawn_positions(dead)
            # Reset age to ~0 with a tiny rng offset so respawns stagger.
            self.age[dead] = self._rng.uniform(0.0, 0.05, size=int(dead.sum()))
            self.vel[dead] = 0.0

        # Sanitize: no NaNs/infs, keep everything inside sane bounds.
        np.nan_to_num(self.pos, copy=False, nan=0.0, posinf=0.0, neginf=0.0)
        np.nan_to_num(self.vel, copy=False, nan=0.0, posinf=0.0, neginf=0.0)
        np.clip(self.pos, -_POS_BOUND, _POS_BOUND, out=self.pos)
        np.clip(self.vel, -_VEL_BOUND, _VEL_BOUND, out=self.vel)

    # ----------------------------------------------------------------- points

    def points(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Renderable snapshot.

        Returns ``(pos[P,3], rgba[P,4], size[P])`` where alpha encodes a
        lifetime fade (``255 * (1 - age/life)``) so the renderer can dim aging
        particles. With no particles, returns correctly-shaped empties.
        """
        if self._P == 0:
            return (
                np.zeros((0, 3), dtype=np.float64),
                np.zeros((0, 4), dtype=np.int64),
                np.zeros((0,), dtype=np.float64),
            )

        fade = np.clip(1.0 - self.age / np.maximum(1e-6, self.life), 0.0, 1.0)
        alpha = np.rint(255.0 * fade)
        rgb = np.rint(np.clip(self.color, 0.0, 255.0))
        rgba = np.concatenate([rgb, alpha[:, None]], axis=1).astype(np.int64)

        return self.pos.copy(), rgba, self.size.copy()
