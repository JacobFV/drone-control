"""
GPU-free cloth instancing for the swarm sim.

Simulating cloth is expensive, so a scene only runs a *handful* of real cloth
panels (the "masters" — e.g. PyBullet soft bodies elsewhere). This module takes
one master's live, deformed world-space mesh and cheaply renders MANY copies of
it: each instance reuses the master's drape (relative to the master's anchor),
is translated to its own anchor, and gets a small per-instance *sway* layered on
top so the copies don't read as identical clones. The result is hundreds of
gently swaying garments backed by only a few real sims — enough to fill a
clothing store with hanging clothes.

Everything is numpy float64 in the world frame (z up). A cloth hangs DOWN from
its top-edge anchor, so mesh vertices sit at or below the anchor and the ones
farther below are freer to move (the hem swings, the shoulders barely budge).

The instancing is fully deterministic: the only thing that distinguishes one
instance's motion from another is its given ``phase`` — no rng, no wall-clock.
The sway is driven by the *same* airflow field (``wind_at``) that the drones and
particles feel, so garments lean with the wind and flutter across it.
"""

from __future__ import annotations

from collections.abc import Callable, Sequence

import numpy as np


def expand_cloth_instances(
    master_verts: np.ndarray,
    master_anchor: tuple[float, float, float],
    faces: list[tuple[int, int, int]],
    placements: Sequence[tuple],
    t: float,
    wind_at: Callable[[np.ndarray], np.ndarray],
    *,
    sway_gain: float = 0.18,
    flutter: float = 0.06,
    freq: float = 0.9,
) -> list[tuple[np.ndarray, list, tuple[int, int, int], str]]:
    """Expand one simulated master cloth into many cheap swaying instances.

    See module docstring. Returns one ``(verts[V,3], faces, color, label)`` tuple
    per placement, ready for the renderer's triangle painter. ``faces`` is shared
    (the topology is identical across instances).
    """
    master_verts = np.asarray(master_verts, dtype=np.float64)
    # Nothing to expand: no master geometry or no requested instances.
    if master_verts.size == 0 or len(placements) == 0:
        return []

    anchor0 = np.asarray(master_anchor, dtype=np.float64)

    # The master's drape, expressed relative to its own anchor: this is the
    # shared "shape" every instance reuses (just translated + swayed).
    rel = master_verts - anchor0  # [V,3]

    # How far each vertex hangs below the anchor (top edge ~0, hem largest).
    # This scales the sway so the shoulders stay put and the hem swings freely.
    depth_below = np.clip(-rel[:, 2], 0.0, None)  # [V]

    out: list[tuple[np.ndarray, list, tuple[int, int, int], str]] = []
    for anchor, color, label, phase in placements:
        anchor = np.asarray(anchor, dtype=np.float64)
        phase = float(phase)

        # Sample the airflow once at this instance's anchor; reduce to a
        # horizontal direction (wind) and its left-hand perpendicular (flutter).
        w = np.asarray(wind_at(np.array([anchor], dtype=np.float64)), dtype=np.float64)[0]
        w_xy = w[:2]
        speed = float(np.linalg.norm(w_xy))
        wdir = w_xy / speed if speed > 1e-6 else np.array([1.0, 0.0])
        perp = np.array([-wdir[1], wdir[0]])

        # Lateral sway, vectorized over all vertices. Both terms grow with
        # depth_below and oscillate at this instance's phase, so neighbouring
        # garments are out of phase and never move in lockstep.
        #   * along-wind lean, stronger in faster air
        amp = sway_gain * depth_below * (0.4 + speed) * np.sin(2 * np.pi * freq * t + phase)
        #   * cross-wind flutter at a higher, offset frequency
        fl = flutter * depth_below * np.sin(2 * np.pi * freq * 1.7 * t + phase * 1.3)

        # Build the [V,3] xy displacement (z stays 0 — sway is horizontal).
        disp = np.zeros_like(rel)  # [V,3]
        disp[:, :2] = amp[:, None] * wdir[None, :] + fl[:, None] * perp[None, :]

        verts = rel + disp + anchor

        # Keep the renderer safe from any stray nan/inf or runaway coordinates.
        verts = np.nan_to_num(verts, nan=0.0, posinf=0.0, neginf=0.0)
        np.clip(verts, -200.0, 200.0, out=verts)

        out.append((verts, faces, color, label))

    return out
