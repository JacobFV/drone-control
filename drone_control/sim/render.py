"""
Headless synthetic forward-camera renderer for the sim.

A minimal pinhole rasteriser (numpy + PIL) that draws, from each drone's
viewpoint, a coloured/textured scene (sky or ceiling gradient, a checkerboard
ground, depth-sorted shaded box geometry from the active scene plan) plus the
policy cues: the drone's own goal marker and the other drones. Output is JPEG
bytes per camera, matching the real stack's frame format. Not photoreal — a
learnable, deterministic visual world that varies per scene plan.
"""

from __future__ import annotations

import io
from dataclasses import dataclass

import numpy as np

from .dynamics import quat_to_rotmat
from .scenes import Box, Scene, build_scene

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


# Per-face shading factors (top brightest, bottom darkest) for box faces.
_FACE_SHADE = {"top": 1.0, "side_x": 0.82, "side_y": 0.68, "bottom": 0.5}


class CameraRenderer:
    def __init__(self, config: CameraConfig | None = None, scene: Scene | str | None = None) -> None:
        if not _PIL:
            raise RuntimeError("Pillow is required for the sim renderer")
        self.config = config or CameraConfig()
        self.scene = scene if isinstance(scene, Scene) else build_scene(scene)
        self.config.far = max(self.config.far, self.scene.far)
        self.focal = (self.config.width / 2.0) / np.tan(np.deg2rad(self.config.fov_deg) / 2.0)
        self.cx = self.config.width / 2.0
        self.cy = self.config.height / 2.0
        self._static_quads = self._build_static_quads()

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
        return [self._render_one(i, pos, rot, goals) for i in idxs]

    def _render_one(self, i: int, pos: np.ndarray, rot: np.ndarray, goals: np.ndarray) -> bytes:
        cfg = self.config
        cam = pos[i]
        r = rot[i]
        forward, up, right = r[:, 0], r[:, 2], r[:, 1]

        img = Image.new("RGB", (cfg.width, cfg.height))
        draw = ImageDraw.Draw(img)
        self._fill_sky(img, draw, forward, up, right)

        def project(p):
            rel = p - cam
            zc = float(rel @ forward)
            if zc <= 0.05 or zc > cfg.far:
                return None
            u = self.cx + self.focal * float(rel @ right) / zc
            v = self.cy - self.focal * float(rel @ up) / zc
            return u, v, zc

        # Depth-sort static scene quads (floor checker, ceiling, box faces) and
        # paint far -> near so nearer geometry occludes.
        rendered = []
        for corners, color in self._static_quads:
            projected = [project(c) for c in corners]
            if any(p is None for p in projected):
                continue
            mean_z = sum(p[2] for p in projected) / len(projected)
            rendered.append((mean_z, [(p[0], p[1]) for p in projected], color))
        rendered.sort(key=lambda q: q[0], reverse=True)
        for _z, poly, color in rendered:
            draw.polygon(poly, fill=color)

        # Other drones as cyan markers.
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

    def _fill_sky(self, img: "Image.Image", draw: "ImageDraw.ImageDraw", forward, up, right) -> None:
        # Vertical gradient (sky for outdoor, ceiling tint for indoor) so the
        # upper half is never flat. The textured ground/ceiling quads paint over
        # the lower portion afterwards.
        cfg = self.config
        top = np.array(self.scene.sky_top, dtype=float)
        bottom = np.array(self.scene.sky_bottom, dtype=float)
        for y in range(cfg.height):
            t = y / max(1, cfg.height - 1)
            color = tuple(int(v) for v in (top * (1 - t) + bottom * t))
            draw.line([(0, y), (cfg.width, y)], fill=color)

    def _build_static_quads(self) -> list[tuple[list[np.ndarray], tuple[int, int, int]]]:
        """Floor checker tiles, optional ceiling tiles, and shaded box faces."""
        quads: list[tuple[list[np.ndarray], tuple[int, int, int]]] = []
        scene = self.scene
        rng = self.config.grid_range
        step = self.config.grid_step
        n = int(rng / step)
        for ix in range(-n, n):
            for iy in range(-n, n):
                x0, y0 = ix * step, iy * step
                x1, y1 = x0 + step, y0 + step
                checker = (ix + iy) & 1
                color = scene.ground_color if checker else scene.ground_alt
                quads.append((
                    [np.array([x0, y0, 0.0]), np.array([x1, y0, 0.0]),
                     np.array([x1, y1, 0.0]), np.array([x0, y1, 0.0])],
                    color,
                ))
                if scene.ceiling_z is not None:
                    z = scene.ceiling_z
                    cc = scene.ceiling_color if checker else _scale(scene.ceiling_color, 0.85)
                    quads.append((
                        [np.array([x0, y0, z]), np.array([x1, y0, z]),
                         np.array([x1, y1, z]), np.array([x0, y1, z])],
                        cc,
                    ))
        for box in scene.boxes:
            quads.extend(_box_faces(box))
        return quads


def _box_faces(box: Box) -> list[tuple[list[np.ndarray], tuple[int, int, int]]]:
    cx, cy, cz = box.center
    sx, sy, sz = box.size
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    # 8 corners.
    c = {
        (a, b, d): np.array([cx + a * hx, cy + b * hy, cz + d * hz])
        for a in (-1, 1) for b in (-1, 1) for d in (-1, 1)
    }
    faces = [
        ([c[(-1, -1, 1)], c[(1, -1, 1)], c[(1, 1, 1)], c[(-1, 1, 1)]], "top"),
        ([c[(-1, -1, -1)], c[(1, -1, -1)], c[(1, 1, -1)], c[(-1, 1, -1)]], "bottom"),
        ([c[(-1, -1, -1)], c[(1, -1, -1)], c[(1, -1, 1)], c[(-1, -1, 1)]], "side_y"),
        ([c[(-1, 1, -1)], c[(1, 1, -1)], c[(1, 1, 1)], c[(-1, 1, 1)]], "side_y"),
        ([c[(-1, -1, -1)], c[(-1, 1, -1)], c[(-1, 1, 1)], c[(-1, -1, 1)]], "side_x"),
        ([c[(1, -1, -1)], c[(1, 1, -1)], c[(1, 1, 1)], c[(1, -1, 1)]], "side_x"),
    ]
    return [(corners, _scale(box.color, _FACE_SHADE[kind])) for corners, kind in faces]


def _scale(color: tuple[int, int, int], factor: float) -> tuple[int, int, int]:
    return tuple(int(max(0, min(255, v * factor))) for v in color)


def _rotmats(quat_wxyz: np.ndarray) -> np.ndarray:
    import torch

    rot = quat_to_rotmat(torch.from_numpy(quat_wxyz.astype(np.float32)))
    return rot.numpy().astype(np.float64)
