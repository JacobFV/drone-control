"""
Volumetric smoke + fire field for the swarm sim's synthetic camera.

Where ``particles.py`` exposes thin tracer *points*, this module models thicker
*puffs* — soft volumetric blobs the renderer composites as alpha splats (and,
for fire, additive glow). We own the physics/emission/advection and hand the
renderer a per-puff snapshot; it owns the screen-space splatting.

Two emitter flavours:

* **smoke** (``SmokeSpec``) — a sustained pool of puffs born at the emitter
  mouth, lofted by buoyancy, carried by the wind (the *same* field the drones
  feel), expanding and fading as they age and dissipate;
* **fire** (``FireSpec``) — a bright emissive flame core (short-lived, fast
  rising, flickering) that ALSO feeds a cooler smoke column rising above it.

Each puff is a tracer: it doesn't push back on the air. Smoke puffs relax their
velocity toward the local wind, gain buoyancy (stronger while young/hot), and
get a small *deterministic* turbulence wobble (driven by a per-puff phase and
the puff's own age — never fresh per-step randomness). Flame puffs rise fast,
barely feel the wind, and flicker hard. Dead puffs (too old, or sunk below the
ground) respawn at their emitter from a single seeded rng.

Everything is numpy float64 in the world frame (z up, ground at z=0) and fully
deterministic given a seed.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(slots=True)
class SmokeSpec:
    """Scene-authored pure-smoke emitter (pure data, like ``FlowSpec``)."""

    pos: tuple[float, float, float]
    radius: float = 0.6  # emitter mouth radius (birth scatter)
    color: tuple[int, int, int] = (90, 90, 96)
    rate: int = 40  # live puffs sustained for this emitter
    rise: float = 1.8  # buoyancy accel (m/s^2 up)
    spread: float = 0.4  # initial lateral velocity jitter (m/s)
    lifetime: float = 6.0  # seconds before a puff dissipates / respawns
    start_radius: float = 0.5  # puff radius at birth
    end_radius: float = 2.6  # puff radius at death (dissipation growth)
    density: float = 0.5  # peak opacity 0..1


@dataclass(slots=True)
class FireSpec:
    """Scene-authored fire emitter: emissive core + a smoke column above it."""

    pos: tuple[float, float, float]
    radius: float = 0.6  # base radius (birth scatter at the flame root)
    height: float = 1.5  # nominal flame height (m)
    intensity: float = 1.0  # overall brightness / scale
    smoke_color: tuple[int, int, int] = (60, 58, 60)
    flame_color: tuple[int, int, int] = (255, 150, 40)  # hot core
    smoke_rate: int = 60  # live smoke puffs above the flame
    flame_rate: int = 30  # live flame puffs in the core
    rise: float = 3.5  # hot air rises fast (m/s^2 up)
    lifetime_smoke: float = 7.0
    lifetime_flame: float = 0.9
    label: str = "fire"


# Sanity bounds so a runaway wind field can never blow the buffers up.
_POS_BOUND = 80.0
_VEL_BOUND = 30.0
_RELAX_K = 2.5  # how fast a smoke tracer's velocity chases the local wind

# Puff "kind" codes.
_KIND_SMOKE = 0
_KIND_FLAME = 1


class VolumetricField:
    """A batched pool of advected smoke/fire puffs across several emitters."""

    def __init__(
        self,
        smokes: list[SmokeSpec],
        fires: list[FireSpec],
        *,
        seed: int = 0,
    ) -> None:
        self.smokes = list(smokes or [])
        self.fires = list(fires or [])
        self._rng = np.random.default_rng(seed)

        # Build a flat plan of per-emitter slices. Each entry records how many
        # puffs the emitter owns and its constant spawn parameters, so respawns
        # land back in the right place with the right physics.
        plan: list[dict] = []
        for s in self.smokes:
            n = max(0, int(s.rate))
            if n:
                plan.append(
                    {
                        "n": n,
                        "kind": _KIND_SMOKE,
                        "origin": np.asarray(s.pos, dtype=np.float64),
                        "mouth_radius": float(s.radius),
                        "rise": float(s.rise),
                        "spread": float(s.spread),
                        "life": max(0.05, float(s.lifetime)),
                        "birth_radius": float(s.start_radius),
                        "death_radius": float(s.end_radius),
                        "density": float(np.clip(s.density, 0.0, 1.0)),
                        "color": np.asarray(s.color, dtype=np.float64),
                        "emissive": 0.0,
                        "column": max(0.05, float(s.lifetime)) * float(s.rise),
                    }
                )
        for fr in self.fires:
            inten = max(0.05, float(fr.intensity))
            # Flame core: short-lived, fast rising, tight at the root.
            nf = max(0, int(fr.flame_rate))
            if nf:
                plan.append(
                    {
                        "n": nf,
                        "kind": _KIND_FLAME,
                        "origin": np.asarray(fr.pos, dtype=np.float64),
                        "mouth_radius": float(fr.radius),
                        "rise": float(fr.rise) * 1.6,  # flames shoot up
                        "spread": float(fr.radius) * 0.5,
                        "life": max(0.05, float(fr.lifetime_flame)),
                        "birth_radius": float(fr.radius) * 0.9 * inten,
                        "death_radius": float(fr.radius) * 0.3 * inten,  # tapers
                        "density": float(np.clip(0.85 * inten, 0.0, 1.0)),
                        "color": np.asarray(fr.flame_color, dtype=np.float64),
                        "emissive": 1.0,
                        "column": float(fr.height),
                    }
                )
            # Smoke column fed above the flame: starts near the flame top.
            ns = max(0, int(fr.smoke_rate))
            if ns:
                top = np.asarray(fr.pos, dtype=np.float64).copy()
                top[2] += float(fr.height)
                plan.append(
                    {
                        "n": ns,
                        "kind": _KIND_SMOKE,
                        "origin": top,
                        "mouth_radius": float(fr.radius) * 1.1,
                        "rise": float(fr.rise) * 0.7,  # cooled, still buoyant
                        "spread": float(fr.radius) * 0.4,
                        "life": max(0.05, float(fr.lifetime_smoke)),
                        "birth_radius": float(fr.radius) * 1.2,
                        "death_radius": float(fr.radius) * 4.0,
                        "density": float(np.clip(0.55 * inten, 0.0, 1.0)),
                        "color": np.asarray(fr.smoke_color, dtype=np.float64),
                        "emissive": 0.0,
                        "column": max(0.05, float(fr.lifetime_smoke))
                        * float(fr.rise)
                        * 0.7,
                    }
                )

        P = int(sum(e["n"] for e in plan))
        self._P = P

        # Batched per-puff state.
        self.pos = np.zeros((P, 3), dtype=np.float64)
        self.vel = np.zeros((P, 3), dtype=np.float64)
        self.age = np.zeros(P, dtype=np.float64)
        self.life = np.full(P, 1.0, dtype=np.float64)
        self.kind = np.zeros(P, dtype=np.int64)

        # Per-puff radius growth endpoints + render constants.
        self.birth_radius = np.zeros(P, dtype=np.float64)
        self.death_radius = np.zeros(P, dtype=np.float64)
        self.base_color = np.zeros((P, 3), dtype=np.float64)
        self.density = np.zeros(P, dtype=np.float64)
        self.emissive = np.zeros(P, dtype=np.float64)

        # Reproducible respawn parameters per puff.
        self._origin = np.zeros((P, 3), dtype=np.float64)
        self._mouth = np.zeros(P, dtype=np.float64)
        self._rise = np.zeros(P, dtype=np.float64)
        self._spread = np.zeros(P, dtype=np.float64)
        self._is_flame = np.zeros(P, dtype=bool)

        # Per-puff deterministic turbulence/flicker phases (advanced by age, not
        # by fresh randomness each step).
        self._phase = self._rng.uniform(0.0, 2.0 * np.pi, size=(P, 3))

        if P == 0:
            return

        cursor = 0
        for e in plan:
            n = e["n"]
            sl = slice(cursor, cursor + n)
            cursor += n

            self.kind[sl] = e["kind"]
            self.life[sl] = e["life"]
            self.birth_radius[sl] = e["birth_radius"]
            self.death_radius[sl] = e["death_radius"]
            self.base_color[sl] = e["color"][None, :]
            self.density[sl] = e["density"]
            self.emissive[sl] = e["emissive"]
            self._origin[sl] = e["origin"][None, :]
            self._mouth[sl] = e["mouth_radius"]
            self._rise[sl] = e["rise"]
            self._spread[sl] = e["spread"]
            self._is_flame[sl] = e["kind"] == _KIND_FLAME

            # Birth puffs at the mouth with a seeded velocity, then pre-advance
            # them up the column so the field already looks established on frame
            # one (positions scattered along the rise, ages spread over life).
            self._spawn(sl)
            frac = self._rng.uniform(0.0, 1.0, size=n)
            self.age[sl] = frac * e["life"]
            # Loft initial positions up the (approximate) buoyant column by the
            # distance a puff would have travelled over its current age.
            self.pos[sl, 2] += frac * e["column"]
            # And nudge laterally so it isn't a perfect vertical line.
            self.pos[sl, 0] += self.vel[sl, 0] * frac * e["life"]
            self.pos[sl, 1] += self.vel[sl, 1] * frac * e["life"]

    # ----------------------------------------------------------- spawn helper

    def _spawn(self, idx) -> None:
        """(Re)birth the puffs selected by ``idx`` at their emitter mouth.

        Positions scatter on the mouth disk; velocities get a small seeded
        lateral kick (``spread``) and a touch of upward push, all from the
        single seeded rng so the whole field is reproducible.
        """
        origin = self._origin[idx]
        mouth = self._mouth[idx]
        spread = self._spread[idx]
        n = origin.shape[0]
        if n == 0:
            return

        # Uniform sample on the mouth disk (sqrt for area-uniformity).
        theta = self._rng.uniform(0.0, 2.0 * np.pi, size=n)
        r = np.sqrt(self._rng.uniform(0.0, 1.0, size=n)) * mouth
        pos = origin.copy()
        pos[:, 0] += r * np.cos(theta)
        pos[:, 1] += r * np.sin(theta)
        pos[:, 2] += self._rng.uniform(0.0, 0.2, size=n) * np.maximum(mouth, 1e-3)
        self.pos[idx] = pos

        # Initial velocity: lateral jitter + a little upward seed.
        vx = self._rng.uniform(-1.0, 1.0, size=n) * spread
        vy = self._rng.uniform(-1.0, 1.0, size=n) * spread
        vz = self._rng.uniform(0.2, 0.8, size=n)
        self.vel[idx] = np.stack([vx, vy, vz], axis=1)

    # ------------------------------------------------------------------- step

    def step(self, dt: float, wind_at) -> None:
        """Advance the field by ``dt`` seconds.

        ``wind_at(points[N,3]) -> wind[N,3]`` supplies the local airflow that
        the smoke advects with (the same field the drones feel).
        """
        if self._P == 0:
            return
        dt = float(dt)
        if dt <= 0.0:
            return

        smoke = ~self._is_flame
        flame = self._is_flame

        # Local wind for every puff.
        wind = np.asarray(wind_at(self.pos), dtype=np.float64)
        wind = np.nan_to_num(wind, nan=0.0, posinf=0.0, neginf=0.0)
        if wind.shape != self.vel.shape:
            wind = np.zeros_like(self.vel)

        # Age-normalised progress 0..1 (1 = death), used for buoyancy decay and
        # the turbulence/flicker phase advance.
        prog = np.clip(self.age / np.maximum(1e-6, self.life), 0.0, 1.0)
        hot = 1.0 - prog  # young puffs are hotter / more buoyant

        # --- Smoke puffs: tracers that chase the wind. ----------------------
        # Velocity relaxes toward the local wind (light tracer).
        relax = float(np.clip(dt * _RELAX_K, 0.0, 1.0))
        self.vel[smoke] += (wind[smoke] - self.vel[smoke]) * relax
        # Buoyancy: stronger while young/hot (hot air near the source rises
        # harder, then the plume cools and levels off).
        self.vel[smoke, 2] += self._rise[smoke] * (0.4 + 0.6 * hot[smoke]) * dt

        # --- Flame puffs: rise fast, barely feel the wind. ------------------
        # A whisper of wind so a strong cross-breeze leans the flame, but mostly
        # they just shoot up.
        self.vel[flame] += (wind[flame] - self.vel[flame]) * (0.1 * relax)
        self.vel[flame, 2] += self._rise[flame] * dt

        # Deterministic turbulence/flicker: per-puff phases advanced by age, so
        # it's smooth and reproducible (no fresh randomness per step). Flames
        # flicker hard and fast; smoke wobbles gently.
        t = self.age
        wob = np.stack(
            [
                np.sin(2.3 * t + self._phase[:, 0]),
                np.cos(1.9 * t + self._phase[:, 1]),
                np.sin(2.7 * t + self._phase[:, 2]),
            ],
            axis=1,
        )
        flick = np.stack(
            [
                np.sin(13.0 * t + self._phase[:, 0]),
                np.cos(11.0 * t + self._phase[:, 1]),
                np.sin(17.0 * t + self._phase[:, 2]),
            ],
            axis=1,
        )
        self.vel[smoke] += 0.35 * wob[smoke] * dt
        self.vel[flame] += 1.2 * flick[flame] * dt

        # Integrate + age.
        np.clip(self.vel, -_VEL_BOUND, _VEL_BOUND, out=self.vel)
        self.pos += self.vel * dt
        self.age += dt

        # Respawn dead puffs (too old) or any that sank below the ground.
        dead = (self.age > self.life) | (self.pos[:, 2] < 0.0)
        if np.any(dead):
            self._spawn(dead)
            self.age[dead] = self._rng.uniform(0.0, 0.05, size=int(dead.sum()))

        # Sanitize: no NaNs/infs, keep everything inside sane bounds.
        np.nan_to_num(self.pos, copy=False, nan=0.0, posinf=0.0, neginf=0.0)
        np.nan_to_num(self.vel, copy=False, nan=0.0, posinf=0.0, neginf=0.0)
        np.clip(self.pos, -_POS_BOUND, _POS_BOUND, out=self.pos)
        np.clip(self.vel, -_VEL_BOUND, _VEL_BOUND, out=self.vel)

    # ----------------------------------------------------------------- puffs

    def puffs(self) -> dict[str, np.ndarray]:
        """Renderable snapshot of every puff (all arrays length P).

        ``radius`` grows from birth->death (smoke expands as it dissipates;
        flames taper). ``opacity`` is ``density`` modulated by an age fade and a
        kind-dependent shaping (smoke fades in then out; flames flicker bright).
        ``color`` is the base tint, with flames pushed toward yellow/white while
        young/near the base. ``emissive`` is the additive-glow term (fire lights
        up the scene; smoke contributes none).
        """
        if self._P == 0:
            return {
                "pos": np.zeros((0, 3), dtype=np.float64),
                "radius": np.zeros((0,), dtype=np.float64),
                "opacity": np.zeros((0,), dtype=np.float64),
                "color": np.zeros((0, 3), dtype=np.int64),
                "emissive": np.zeros((0,), dtype=np.float64),
            }

        prog = np.clip(self.age / np.maximum(1e-6, self.life), 0.0, 1.0)
        flame = self._is_flame
        smoke = ~flame

        # Radius: linear interpolation birth -> death over the puff's life.
        radius = self.birth_radius + (self.death_radius - self.birth_radius) * prog
        radius = np.maximum(radius, 1e-3)

        # Opacity. Smoke: a quick fade-in off the mouth then a long fade-out as
        # it dissipates. Flames: bright with a per-puff flicker, dimming as they
        # rise and burn out.
        opacity = np.zeros(self._P, dtype=np.float64)
        # Smoke fade: ramps up over the first ~20% of life, decays after.
        fade_in = np.clip(prog / 0.2, 0.0, 1.0)
        fade_out = np.clip(1.0 - prog, 0.0, 1.0) ** 1.3
        opacity[smoke] = self.density[smoke] * fade_in[smoke] * fade_out[smoke]
        # Flame flicker: deterministic from the puff's phase + age.
        flick = 0.75 + 0.25 * np.sin(19.0 * self.age + self._phase[:, 0])
        flame_fade = np.clip(1.0 - prog, 0.0, 1.0)
        opacity[flame] = self.density[flame] * flick[flame] * (0.5 + 0.5 * flame_fade[flame])
        opacity = np.clip(opacity, 0.0, 1.0)

        # Colour. Smoke = its base gray. Flame = base hot colour, pushed toward
        # a brighter yellow/white near the base (young puffs) so the core reads
        # white-hot and the tips redder.
        color = self.base_color.copy()
        if np.any(flame):
            hot = (1.0 - prog)[:, None]  # 1 at birth (base), 0 at death (tip)
            white = np.array([255.0, 235.0, 170.0])  # near-white-hot yellow
            fc = self.base_color + (white - self.base_color) * (hot ** 1.5)
            color[flame] = fc[flame]
        color = np.rint(np.clip(color, 0.0, 255.0)).astype(np.int64)

        # Emissive: flames glow (brighter while young), smoke none.
        emissive = np.zeros(self._P, dtype=np.float64)
        emissive[flame] = np.clip(0.6 + 0.4 * (1.0 - prog[flame]), 0.0, 1.0)
        emissive[flame] *= flick[flame]
        emissive = np.clip(emissive, 0.0, 1.0)

        return {
            "pos": self.pos.copy(),
            "radius": radius,
            "opacity": opacity,
            "color": color,
            "emissive": emissive,
        }

    # ---------------------------------------------------------------- bounds

    def bounds(self) -> tuple | None:
        """Axis-aligned bounding box ``(min[3], max[3])`` of puffs, or None."""
        if self._P == 0:
            return None
        lo = np.min(self.pos, axis=0)
        hi = np.max(self.pos, axis=0)
        return (lo.copy(), hi.copy())
