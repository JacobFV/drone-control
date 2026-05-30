"""
Airflow model for the swarm sim.

A flow field that returns a world-frame wind velocity at any set of points,
summed from several physically-motivated contributions:

* **ambient wind** — a steady direction with sinusoidal gusts + a little
  turbulence, plus a boundary-layer height profile (slower near the ground);
* **fan / wind-generator primitives** — directional jets placed in the scene
  that blow along an axis with a conical reach and distance falloff;
* **updrafts** — vertical thermal/HVAC columns;
* **moving-object wakes** — air dragged along behind moving scene objects;
* **rotor downwash** — each drone pushes air down and outward, buffeting other
  drones below it.

Drones (and cloth nodes) feel an aerodynamic force

    F = c_aero * |v_rel| * v_rel,   v_rel = wind - body_velocity

with ``c_aero = 0.5 * rho * Cd * A``. The env adds ``F / m`` as an external
acceleration, so a controller must actively reject wind disturbances — exactly
what we want a sim-trained policy to learn before it meets a real gust.

Everything here is numpy (small N: a handful of drones, a few hundred cloth
nodes); the env converts the per-drone result to a torch tensor for the
integrator.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

AIR_DENSITY = 1.204  # kg/m^3 at 20 C


@dataclass(slots=True)
class FlowSpec:
    """Scene-authored flow primitive (pure data, like ``DynamicSpec``).

    kind:
      * ``wind``    — params: dir (vx,vy,vz) m/s base, gust (0..1), period (s),
                      turbulence (m/s), shear_ref (m), label
      * ``fan``     — params: pos (x,y,z), dir (x,y,z), speed (m/s),
                      radius (m), reach (m), spread (cone widening factor)
      * ``updraft`` — params: cx, cy, radius (m), speed (m/s), top (m)
    """

    kind: str
    params: dict
    label: str = "flow"


def _unit(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    return v / n if n > 1e-9 else v


class FlowField:
    """Evaluates the summed wind velocity at arbitrary world points."""

    def __init__(self, specs: list[FlowSpec] | None = None, *, seed: int = 0) -> None:
        self.specs = list(specs or [])
        self._rng = np.random.default_rng(seed)
        self.winds = [s for s in self.specs if s.kind == "wind"]
        self.fans = [s for s in self.specs if s.kind == "fan"]
        self.updrafts = [s for s in self.specs if s.kind == "updraft"]
        # Per-wind random gust phase so scenes with several winds don't pulse in
        # lockstep (deterministic given the seed).
        self._phase = [float(self._rng.uniform(0, 2 * math.pi)) for _ in self.winds]

    # ---------------------------------------------------------------- ambient

    def ambient(self, t: float, z: float = 2.0) -> np.ndarray:
        """Spatially-uniform wind at height ``z`` (used for status + windsock)."""
        total = np.zeros(3)
        for spec, phase in zip(self.winds, self._phase):
            p = spec.params
            base = np.array(p.get("dir", (0.0, 0.0, 0.0)), dtype=float)
            gust = p.get("gust", 0.0)
            period = max(0.5, p.get("period", 6.0))
            gust_factor = 1.0 + gust * math.sin(2 * math.pi * t / period + phase)
            shear_ref = p.get("shear_ref", 3.0)
            shear = float(np.clip(abs(z) / max(0.5, shear_ref), 0.15, 1.4)) ** 0.5
            total += base * gust_factor * shear
        return total

    # ----------------------------------------------------------------- sample

    def sample(
        self,
        points: np.ndarray,
        t: float,
        *,
        objects: tuple[np.ndarray, np.ndarray] | None = None,
        rotors: tuple[np.ndarray, np.ndarray] | None = None,
        turbulence: bool = True,
    ) -> np.ndarray:
        """World-frame wind velocity [N,3] at each of ``points`` [N,3]."""
        pts = np.atleast_2d(np.asarray(points, dtype=float))
        wind = np.zeros_like(pts)

        # Ambient wind (height-sheared) + small per-point turbulence.
        for spec, phase in zip(self.winds, self._phase):
            p = spec.params
            base = np.array(p.get("dir", (0.0, 0.0, 0.0)), dtype=float)
            gust = p.get("gust", 0.0)
            period = max(0.5, p.get("period", 6.0))
            gust_factor = 1.0 + gust * math.sin(2 * math.pi * t / period + phase)
            shear_ref = p.get("shear_ref", 3.0)
            shear = np.clip(np.abs(pts[:, 2]) / max(0.5, shear_ref), 0.15, 1.4) ** 0.5
            wind += base[None, :] * gust_factor * shear[:, None]
            turb = p.get("turbulence", 0.0)
            if turbulence and turb > 0:
                # Smooth-ish spatial turbulence from a few sinusoids in space+time.
                sx = np.sin(0.7 * pts[:, 0] + 1.3 * t + phase)
                sy = np.cos(0.6 * pts[:, 1] - 1.1 * t + phase)
                sz = np.sin(0.9 * pts[:, 2] + 0.7 * t)
                wind += turb * np.stack([sy, sx, 0.4 * sz], axis=1)

        # Fan jets: blow along dir within a widening cone of length ``reach``.
        for spec in self.fans:
            p = spec.params
            origin = np.array(p.get("pos", (0.0, 0.0, 1.0)), dtype=float)
            direction = _unit(np.array(p.get("dir", (1.0, 0.0, 0.0)), dtype=float))
            speed = float(p.get("speed", 6.0))
            radius = float(p.get("radius", 1.5))
            reach = float(p.get("reach", 10.0))
            spread = float(p.get("spread", 0.5))
            rel = pts - origin[None, :]
            axial = rel @ direction
            radial = rel - axial[:, None] * direction[None, :]
            radial_d = np.linalg.norm(radial, axis=1)
            cone_r = radius * (1.0 + spread * np.clip(axial, 0, None) / max(0.1, reach))
            axial_falloff = np.clip(1.0 - axial / max(0.1, reach), 0.0, 1.0)
            radial_falloff = np.clip(1.0 - radial_d / np.maximum(1e-6, cone_r), 0.0, 1.0)
            mag = speed * axial_falloff * (radial_falloff ** 2) * (axial > 0)
            wind += direction[None, :] * mag[:, None]

        # Vertical updraft columns (thermals / HVAC plumes).
        for spec in self.updrafts:
            p = spec.params
            cx, cy = float(p.get("cx", 0.0)), float(p.get("cy", 0.0))
            radius = float(p.get("radius", 2.0))
            speed = float(p.get("speed", 2.0))
            top = float(p.get("top", 12.0))
            r_xy = np.hypot(pts[:, 0] - cx, pts[:, 1] - cy)
            radial_falloff = np.clip(1.0 - r_xy / max(1e-6, radius), 0.0, 1.0)
            height_falloff = np.clip(1.0 - pts[:, 2] / max(0.1, top), 0.0, 1.0)
            wind[:, 2] += speed * radial_falloff * height_falloff

        # Wakes dragged behind moving scene objects.
        if objects is not None:
            obj_pos, obj_vel = objects
            obj_pos = np.atleast_2d(obj_pos)
            obj_vel = np.atleast_2d(obj_vel)
            for c, v in zip(obj_pos, obj_vel):
                speed = float(np.linalg.norm(v))
                if speed < 0.05:
                    continue
                d = pts - c[None, :]
                sigma = 2.2
                falloff = np.exp(-np.sum(d * d, axis=1) / (2 * sigma * sigma))
                # Air is pulled along the object's motion; gain < 1.
                wind += 0.55 * v[None, :] * falloff[:, None]

        # Rotor downwash: each drone pushes air down + radially out; affects
        # points (other drones / cloth) below it.
        if rotors is not None:
            rotor_pos, rotor_str = rotors
            rotor_pos = np.atleast_2d(rotor_pos)
            rotor_str = np.atleast_1d(rotor_str)
            for c, s in zip(rotor_pos, rotor_str):
                dz = c[2] - pts[:, 2]  # >0 when point is below the rotor
                below = dz > 0.05
                if not np.any(below):
                    continue
                r_xy = np.hypot(pts[:, 0] - c[0], pts[:, 1] - c[1])
                core = 0.9
                radial_falloff = np.exp(-(r_xy ** 2) / (2 * core * core))
                depth_falloff = np.clip(1.0 - dz / 6.0, 0.0, 1.0)
                w = float(s) * 5.0 * radial_falloff * depth_falloff * below
                wind[:, 2] -= w  # push down
                # gentle outward spread near the column
                with np.errstate(invalid="ignore", divide="ignore"):
                    out = np.where(r_xy[:, None] > 1e-3, (pts[:, :2] - c[:2]) / r_xy[:, None], 0.0)
                wind[:, :2] += 0.35 * w[:, None] * out

        return wind


def aero_accel(wind: np.ndarray, vel: np.ndarray, mass: float, c_aero: float) -> np.ndarray:
    """External acceleration [N,3] from aerodynamic drag against the wind.

    F = c_aero * |v_rel| * v_rel, a = F / mass, v_rel = wind - body velocity.
    """
    v_rel = np.asarray(wind, dtype=float) - np.asarray(vel, dtype=float)
    speed = np.linalg.norm(v_rel, axis=1, keepdims=True)
    force = c_aero * speed * v_rel
    return force / max(1e-6, mass)
