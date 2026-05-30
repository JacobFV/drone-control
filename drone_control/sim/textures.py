"""
Procedural surface textures for the synthetic camera.

True image-texture mapping in a per-frame software rasteriser is too slow for
the live loop, so instead each face is painted as a small grid of cells whose
colours come from a deterministic, per-material pattern that *modulates the
face's authored colour*. A red car stays red but gains panel/specular variation;
a brick wall gets mortar courses; foliage gets mottled leaves; glass gets a lit
window grid. Cheap, deterministic, and reads as "textured" at camera resolution.

The renderer calls :func:`texture_cells(label, base_color, rows, cols)` to get an
``[rows, cols, 3]`` uint8 grid, then fills the projected quad cell-by-cell.
"""

from __future__ import annotations

import numpy as np

# label -> material name. Anything unmapped falls back to "concrete".
_LABEL_MATERIAL = {
    "building": "concrete", "wall": "concrete", "slab": "concrete", "column": "concrete",
    "escalator": "concrete", "site-cabin": "metal", "dock-door": "metal", "lamp": "metal",
    "antenna": "metal", "crane": "metal", "hook": "metal", "scaffold": "metal",
    "excavator": "metal", "materials": "metal", "fence-post": "metal", "windsock": "metal",
    "glass-wall": "glass", "monitor": "glass", "kiosk": "glass", "balcony": "concrete",
    "railing": "metal", "car": "metal", "bus": "metal", "truck": "metal", "forklift": "metal",
    "agv": "metal", "tree": "foliage", "plant": "foliage", "planter": "foliage",
    "playground": "metal", "pond": "water", "path": "sand", "shelf": "wood", "pallet": "wood",
    "crate": "wood", "hay-bale": "straw", "bench": "wood", "table": "wood", "desk": "wood",
    "chair": "fabric", "partition": "fabric", "person": "fabric", "cyclist": "fabric",
    "dog": "fabric", "marker": "concrete", "debris": "concrete",
}


def material_for(label: str) -> str:
    return _LABEL_MATERIAL.get(label, "concrete")


def _hash_noise(rows: int, cols: int, salt: int) -> np.ndarray:
    """Deterministic value noise in [0,1], shape [rows, cols]."""
    i = np.arange(rows)[:, None]
    j = np.arange(cols)[None, :]
    h = (i * 73856093) ^ (j * 19349663) ^ (salt * 83492791)
    h = (h ^ (h >> 13)) * 1274126177
    return ((h & 0xFFFF) / 65535.0).astype(np.float64)


def texture_cells(label: str, base_color: tuple[int, int, int], rows: int, cols: int) -> np.ndarray:
    """[rows, cols, 3] uint8 — the face's colour modulated by its material."""
    material = material_for(label)
    base = np.asarray(base_color, dtype=np.float64)
    salt = (int(base[0]) * 7 + int(base[1]) * 13 + int(base[2]) * 17 + len(label)) & 0x7FFF
    mult = np.ones((rows, cols, 3))  # brightness/tint multiplier per cell

    def per_col(vec: np.ndarray) -> np.ndarray:   # vec [cols] -> [1, cols, 1]
        return vec.reshape(1, cols, 1)

    def per_row(vec: np.ndarray) -> np.ndarray:   # vec [rows] -> [rows, 1, 1]
        return vec.reshape(rows, 1, 1)

    if material == "concrete":
        n = _hash_noise(rows, cols, salt)
        mult *= (0.9 + 0.18 * n)[..., None]
    elif material == "metal":
        # Vertical brushed streaks + an occasional bright specular column.
        streak = (0.85 + 0.3 * _hash_noise(1, cols, salt)).reshape(cols)
        mult *= per_col(streak)
        spec = (_hash_noise(1, cols, salt + 5).reshape(cols) > 0.82).astype(float)
        mult += per_col(spec) * 0.25
    elif material == "glass":
        # Mullion grid: darker frames, some lit (bright bluish) panes.
        frame = np.ones((rows, cols))
        frame[0, :] = frame[-1, :] = 0.6
        frame[:, 0] = frame[:, -1] = 0.6
        mult *= frame[..., None]
        lit = _hash_noise(rows, cols, salt + 9) > 0.6
        mult *= np.where(lit[..., None], 1.2, 0.92)
        mult[..., 2] *= (1.0 + 0.25 * lit)
    elif material == "foliage":
        n = _hash_noise(rows, cols, salt)
        mult *= (0.75 + 0.5 * n)[..., None]
        mult[..., 1] *= 1.08  # greener
    elif material == "wood":
        # Horizontal grain.
        grain = (0.85 + 0.22 * _hash_noise(rows, 1, salt)).reshape(rows)
        mult *= per_row(grain)
        mult[..., 0] *= 1.04  # warmer
    elif material == "water":
        ripple = _hash_noise(rows, cols, salt)
        mult *= (0.85 + 0.3 * ripple)[..., None]
        mult[..., 2] *= 1.12  # bluer
    elif material == "sand":
        n = _hash_noise(rows, cols, salt)
        mult *= (0.92 + 0.14 * n)[..., None]
    elif material == "straw":
        grain = (0.8 + 0.3 * _hash_noise(1, cols, salt)).reshape(cols)
        mult *= per_col(grain)
        mult[..., 0] *= 1.06
        mult[..., 1] *= 1.03
    elif material == "fabric":
        weave = 0.9 + 0.12 * (((np.arange(rows)[:, None] + np.arange(cols)[None, :]) & 1))
        mult *= weave[..., None]
    else:
        n = _hash_noise(rows, cols, salt)
        mult *= (0.9 + 0.18 * n)[..., None]

    out = np.clip(base[None, None, :] * mult, 0, 255)
    return out.astype(np.uint8)


# Per-material cell resolution (rows, cols). Small = cheap; the renderer only
# textures near, large faces anyway.
_CELL_RES = {
    "glass": (4, 4), "brick": (5, 4), "metal": (1, 5), "wood": (5, 1),
    "concrete": (4, 4), "foliage": (4, 4), "water": (3, 4), "sand": (3, 3),
    "straw": (1, 5), "fabric": (3, 3),
}


def cell_res(label: str) -> tuple[int, int]:
    return _CELL_RES.get(material_for(label), (3, 3))
