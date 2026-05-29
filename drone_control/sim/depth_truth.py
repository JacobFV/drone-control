"""
Ground-truth depth + point cloud for the simulator.

A real-image monocular depth model (Depth Anything) hallucinates geometry on the
abstract synthetic frames, producing a warped dome instead of a scene. But the
sim knows its world exactly, so for sim environments we ray-cast each camera
against the scene (ground plane + axis-aligned boxes, static and moving) to get
**true** metric depth and coloured world points. This yields a point cloud and a
splat seed that actually look like the scene.
"""

from __future__ import annotations

import numpy as np

from .scenes import Box, Scene


def _boxes_arrays(boxes: list[Box]):
    if not boxes:
        return np.zeros((0, 3)), np.zeros((0, 3)), np.zeros((0, 3), np.uint8)
    centers = np.array([b.center for b in boxes], dtype=np.float64)
    half = np.array([b.size for b in boxes], dtype=np.float64) / 2.0
    cols = np.array([b.color for b in boxes], dtype=np.uint8)
    return centers - half, centers + half, cols


def raycast(
    scene: Scene,
    dyn_boxes: list[Box],
    center: np.ndarray,
    cam_rot: np.ndarray,          # cols = world right, down, forward
    fov_deg: float,
    width: int,
    height: int,
    *,
    stride: int = 3,
    far: float = 40.0,
):
    """Cast camera rays against the scene; return (world[K,3], rgb[K,3] uint8,
    depth_grid[R,C] in metres with inf for sky)."""
    focal = (width / 2.0) / np.tan(np.deg2rad(fov_deg) / 2.0)
    cx, cy = width / 2.0, height / 2.0
    ys = np.arange(0, height, stride)
    xs = np.arange(0, width, stride)
    gx, gy = np.meshgrid(xs, ys)
    R, C = gx.shape
    gxf = gx.reshape(-1).astype(np.float64)
    gyf = gy.reshape(-1).astype(np.float64)
    rays_cam = np.stack([(gxf - cx) / focal, (gyf - cy) / focal, np.ones_like(gxf)], axis=1)
    rays_cam /= np.linalg.norm(rays_cam, axis=1, keepdims=True)
    d = (cam_rot @ rays_cam.T).T          # [N,3] world ray directions
    o = np.asarray(center, dtype=np.float64)
    n = d.shape[0]
    best_t = np.full(n, np.inf)
    best_col = np.zeros((n, 3), dtype=np.uint8)

    # Ground plane z = 0 (with checkerboard colour).
    with np.errstate(divide="ignore", invalid="ignore"):
        tg = -o[2] / d[:, 2]
    hit_g = (d[:, 2] < -1e-6) & (tg > 0.05) & (tg < far)
    gxh = o[0] + d[:, 0] * tg
    gyh = o[1] + d[:, 1] * tg
    checker = (np.floor(gxh / 2.0).astype(np.int64) + np.floor(gyh / 2.0).astype(np.int64)) & 1
    gcol = np.where(checker[:, None] == 1,
                    np.array(scene.ground_color, dtype=np.int64),
                    np.array(scene.ground_alt, dtype=np.int64)).astype(np.uint8)
    upd = hit_g & (tg < best_t)
    best_t = np.where(upd, tg, best_t)
    best_col = np.where(upd[:, None], gcol, best_col)

    # Axis-aligned boxes (slab method, vectorised over rays x boxes).
    mins, maxs, cols = _boxes_arrays(list(scene.boxes) + list(dyn_boxes))
    if mins.shape[0]:
        invd = 1.0 / np.where(np.abs(d) < 1e-9, 1e-9, d)              # [N,3]
        t1 = (mins[None] - o[None, None]) * invd[:, None, :]          # [N,M,3]
        t2 = (maxs[None] - o[None, None]) * invd[:, None, :]
        tmin = np.maximum(np.minimum(t1, t2).max(axis=2), 0.0)       # [N,M]
        tmax = np.maximum(t1, t2).min(axis=2)
        hit = (tmax >= tmin) & (tmax > 0.05) & (tmin < far)
        tcand = np.where(hit, tmin, np.inf)
        m_best = tcand.argmin(axis=1)
        m_t = tcand[np.arange(n), m_best]
        better = m_t < best_t
        best_t = np.where(better, m_t, best_t)
        best_col = np.where(better[:, None], cols[m_best], best_col)

    valid = np.isfinite(best_t) & (best_t < far)
    t = best_t[valid]
    world = o[None, :] + d[valid] * t[:, None]
    depth_grid = best_t.reshape(R, C)
    return world, best_col[valid], depth_grid


def colorize_depth(depth_grid: np.ndarray, near: float, far: float) -> np.ndarray:
    """[R,C] metric depth (inf=sky) -> RGB uint8 [R,C,3] (near=warm)."""
    cmap = np.array([[48, 18, 130], [33, 144, 200], [60, 200, 120], [240, 220, 60], [230, 60, 40]], dtype=np.float64)
    finite = np.isfinite(depth_grid)
    dn = np.zeros_like(depth_grid)
    if finite.any():
        dn[finite] = 1.0 - np.clip((depth_grid[finite] - near) / max(1e-3, far - near), 0, 1)  # near -> 1
    x = np.clip(dn, 0, 1) * (len(cmap) - 1)
    lo = np.floor(x).astype(int)
    hi = np.clip(lo + 1, 0, len(cmap) - 1)
    frac = (x - lo)[..., None]
    rgb = (cmap[lo] * (1 - frac) + cmap[hi] * frac)
    rgb[~finite] = np.array([8, 10, 14])  # sky = near-black
    return rgb.astype(np.uint8)
