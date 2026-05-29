"""
Headless synthetic forward-camera renderer for the sim.

A minimal pinhole rasteriser (numpy + PIL) that draws, from each drone's
viewpoint, the signal a goal-conditioned image->action policy needs to learn:
a horizon-split sky/ground, a ground grid (motion + attitude parallax), the
drone's own goal marker (target direction/distance), and the other drones
(swarm awareness). Output is JPEG bytes per camera, matching the real stack's
frame format. Not photoreal — it is a learnable, deterministic visual cue.
"""

from __future__ import annotations

import io
from dataclasses import dataclass

import numpy as np

from .dynamics import quat_to_rotmat

try:
    from PIL import Image, ImageDraw

    _PIL = True
except Exception:  # pragma: no cover
    _PIL = False


@dataclass(slots=True)
class CameraConfig:
    width: int = 128
    height: int = 96
    fov_deg: float = 75.0
    grid_range: float = 14.0
    grid_step: float = 2.0
    far: float = 40.0
    jpeg_quality: int = 80


class CameraRenderer:
    def __init__(self, config: CameraConfig | None = None) -> None:
        if not _PIL:
            raise RuntimeError("Pillow is required for the sim renderer")
        self.config = config or CameraConfig()
        self.focal = (self.config.width / 2.0) / np.tan(np.deg2rad(self.config.fov_deg) / 2.0)
        self.cx = self.config.width / 2.0
        self.cy = self.config.height / 2.0
        self._grid = self._build_grid()

    def render(
        self,
        pos: np.ndarray,
        quat_wxyz: np.ndarray,
        goals: np.ndarray,
        indices: list[int] | None = None,
    ) -> list[bytes]:
        pos = np.asarray(pos, dtype=np.float64)
        quat_wxyz = np.asarray(quat_wxyz, dtype=np.float64)
        goals = np.asarray(goals, dtype=np.float64)
        rot = _rotmats(quat_wxyz)
        idxs = list(range(pos.shape[0])) if indices is None else indices
        frames: list[bytes] = []
        for i in idxs:
            frames.append(self._render_one(i, pos, rot, goals))
        return frames

    def _render_one(self, i: int, pos: np.ndarray, rot: np.ndarray, goals: np.ndarray) -> bytes:
        cfg = self.config
        cam = pos[i]
        r = rot[i]
        # Forward camera: look along body +x, up = body +z, right = body +y.
        forward = r[:, 0]
        up = r[:, 2]
        right = r[:, 1]

        # Sky/ground split via the projected horizon, then draw cues.
        img = Image.new("RGB", (cfg.width, cfg.height), (60, 95, 140))  # sky
        draw = ImageDraw.Draw(img)
        self._fill_ground(draw, forward, up, right)

        def project(p: np.ndarray):
            rel = p - cam
            zc = float(rel @ forward)
            if zc <= 0.05 or zc > cfg.far:
                return None
            xc = float(rel @ right)
            yc = float(rel @ up)
            u = self.cx + self.focal * xc / zc
            v = self.cy - self.focal * yc / zc
            return u, v, zc

        # Ground grid lines.
        for a, b in self._grid:
            pa, pb = project(a), project(b)
            if pa is None or pb is None:
                continue
            draw.line([pa[0], pa[1], pb[0], pb[1]], fill=(90, 120, 90), width=1)

        # Other drones (swarm awareness) as cyan dots.
        for j in range(pos.shape[0]):
            if j == i:
                continue
            pj = project(pos[j])
            if pj is None:
                continue
            rad = max(1.0, 60.0 / pj[2])
            draw.ellipse([pj[0] - rad, pj[1] - rad, pj[0] + rad, pj[1] + rad], fill=(90, 200, 220))

        # Own goal marker (bright magenta) — the target cue.
        pg = project(goals[i])
        if pg is not None:
            rad = max(2.0, 120.0 / pg[2])
            draw.ellipse([pg[0] - rad, pg[1] - rad, pg[0] + rad, pg[1] + rad], outline=(255, 60, 220), width=2)
            draw.line([self.cx, self.cy, pg[0], pg[1]], fill=(255, 60, 220), width=1)

        buffer = io.BytesIO()
        img.save(buffer, format="JPEG", quality=cfg.jpeg_quality)
        return buffer.getvalue()

    def _fill_ground(self, draw: "ImageDraw.ImageDraw", forward, up, right) -> None:
        # Approximate the horizon line: project two far points on the ground plane
        # straight ahead-left and ahead-right and shade everything below.
        cfg = self.config
        # Horizon pitch: angle of forward vector above horizontal.
        pitch = np.arcsin(np.clip(forward[2], -1.0, 1.0))
        roll = np.arctan2(right[2], up[2])
        v_horizon = self.cy + self.focal * np.tan(pitch)
        # Draw a tilted ground polygon below the horizon.
        dx = np.tan(roll) * cfg.width / 2.0
        poly = [
            (0, v_horizon - dx),
            (cfg.width, v_horizon + dx),
            (cfg.width, cfg.height),
            (0, cfg.height),
        ]
        draw.polygon(poly, fill=(40, 60, 40))

    def _build_grid(self) -> list[tuple[np.ndarray, np.ndarray]]:
        cfg = self.config
        lines: list[tuple[np.ndarray, np.ndarray]] = []
        coords = np.arange(-cfg.grid_range, cfg.grid_range + cfg.grid_step, cfg.grid_step)
        lo, hi = -cfg.grid_range, cfg.grid_range
        for x in coords:
            lines.append((np.array([x, lo, 0.0]), np.array([x, hi, 0.0])))
        for y in coords:
            lines.append((np.array([lo, y, 0.0]), np.array([hi, y, 0.0])))
        return lines


def _rotmats(quat_wxyz: np.ndarray) -> np.ndarray:
    import torch

    rot = quat_to_rotmat(torch.from_numpy(quat_wxyz.astype(np.float32)))
    return rot.numpy().astype(np.float64)
