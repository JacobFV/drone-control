"""Raycast depth oracle (EVAL ONLY).

Given the sim scene plan and a camera pose, computes a per-pixel ground-truth
depth map by ray-casting the analytic scene geometry (axis-aligned boxes, the
bounded checker floor, and the optional ceiling plane). This is the "oracle"
the prior session used to expose the monocular front-end as broken; it is the
yardstick for the multi-view front-end and lives strictly on the eval side.

Convention matches the perception path: depth is the **Euclidean ray length**
from the camera centre to the first surface (``DepthEstimator._backproject``
normalises rays then multiplies by depth, so its "metric depth" is ray length,
not z-depth). Pixels that hit nothing (sky / beyond the rendered floor extent)
are returned as NaN.
"""

from __future__ import annotations

import numpy as np

from drone_control.sim.render import CameraConfig
from drone_control.sim.scenes import Box, Scene, dynamic_objects


def camera_rays(cfg: CameraConfig, rotation: np.ndarray) -> np.ndarray:
    """Unit ray directions in world frame for every pixel, shape [H, W, 3].

    ``rotation`` is the standard camera->world matrix whose columns are the
    camera right / down / forward axes (i.e. ``camera_pose``'s ``rotation_xyzw``).
    """
    w, h = cfg.width, cfg.height
    focal = (w / 2.0) / np.tan(np.deg2rad(cfg.fov_deg) / 2.0)
    cx, cy = w / 2.0, h / 2.0
    xs = np.arange(w, dtype=np.float64)
    ys = np.arange(h, dtype=np.float64)
    gx, gy = np.meshgrid(xs, ys)
    dirs = np.stack([(gx - cx) / focal, (gy - cy) / focal, np.ones_like(gx)], axis=-1)
    dirs /= np.linalg.norm(dirs, axis=-1, keepdims=True)
    world = dirs @ rotation.T  # [H,W,3]; rotation maps camera dirs -> world
    return world


def _intersect_aabb(origin: np.ndarray, dirs: np.ndarray, lo: np.ndarray, hi: np.ndarray) -> np.ndarray:
    """Slab ray/AABB intersection. Returns entry distance t (NaN where no hit).

    ``dirs`` is [N,3] (unit). ``lo``/``hi`` are the box corners [3].
    """
    inv = 1.0 / np.where(np.abs(dirs) < 1e-12, 1e-12, dirs)
    t1 = (lo - origin) * inv  # [N,3]
    t2 = (hi - origin) * inv
    tmin = np.minimum(t1, t2)
    tmax = np.maximum(t1, t2)
    t_near = np.max(tmin, axis=1)
    t_far = np.min(tmax, axis=1)
    hit = (t_near <= t_far) & (t_far > 1e-4)
    # Entry distance: t_near if in front of the camera, else t_far (origin inside box).
    t = np.where(t_near > 1e-4, t_near, t_far)
    return np.where(hit, t, np.nan)


def _plane_z(origin: np.ndarray, dirs: np.ndarray, z: float, looking: str, extent: float) -> np.ndarray:
    """Intersect a horizontal plane at height ``z`` bounded to |x|,|y| <= extent.

    ``looking`` is "down" (floor, requires dir_z < 0) or "up" (ceiling, dir_z > 0).
    """
    dz = dirs[:, 2]
    if looking == "down":
        valid_dir = dz < -1e-9
    else:
        valid_dir = dz > 1e-9
    t = (z - origin[2]) / np.where(np.abs(dz) < 1e-12, 1e-12, dz)
    hit_x = origin[0] + t * dirs[:, 0]
    hit_y = origin[1] + t * dirs[:, 1]
    inside = (np.abs(hit_x) <= extent) & (np.abs(hit_y) <= extent)
    ok = valid_dir & (t > 1e-4) & inside
    return np.where(ok, t, np.nan)


def raycast_depth(
    scene: Scene,
    center: np.ndarray,
    rotation: np.ndarray,
    cfg: CameraConfig,
    t_sim: float = 0.0,
    include_dynamic: bool = True,
    floor_extent: float | None = None,
) -> np.ndarray:
    """Ground-truth depth map [H, W] (Euclidean ray length; NaN = sky/no return)."""
    center = np.asarray(center, dtype=np.float64)
    dirs = camera_rays(cfg, rotation).reshape(-1, 3)  # [N,3]
    n = dirs.shape[0]
    best = np.full(n, np.inf)

    boxes: list[Box] = list(scene.boxes)
    if include_dynamic:
        boxes.extend(dynamic_objects(scene, t_sim))
    for box in boxes:
        c = np.asarray(box.center, dtype=np.float64)
        half = np.asarray(box.size, dtype=np.float64) / 2.0
        t = _intersect_aabb(center, dirs, c - half, c + half)
        best = np.fmin(best, np.where(np.isnan(t), np.inf, t))

    extent = floor_extent if floor_extent is not None else cfg.grid_range
    t_floor = _plane_z(center, dirs, 0.0, "down", extent)
    best = np.fmin(best, np.where(np.isnan(t_floor), np.inf, t_floor))
    if scene.ceiling_z is not None:
        t_ceil = _plane_z(center, dirs, float(scene.ceiling_z), "up", extent)
        best = np.fmin(best, np.where(np.isnan(t_ceil), np.inf, t_ceil))

    depth = np.where(np.isfinite(best), best, np.nan)
    return depth.reshape(cfg.height, cfg.width)
