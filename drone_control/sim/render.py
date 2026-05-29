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
from .scenes import Box, Scene, build_scene, dynamic_objects

try:
    from PIL import Image, ImageDraw, ImageFilter

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


@dataclass(slots=True)
class CameraNoise:
    """Sensor-realism model approximating cheap CMOS modules (OV2640 etc.):
    read + shot (signal-dependent) noise, chroma noise, vignetting, a touch of
    blur, per-frame white-balance drift, and hard JPEG compression."""

    enabled: bool = True
    read_sigma: float = 6.0       # additive luma noise (0..255 scale)
    shot_scale: float = 0.06      # signal-dependent (shot) noise
    chroma_sigma: float = 8.0     # per-channel colour noise
    vignette: float = 0.35        # corner darkening strength (0..1)
    blur_sigma: float = 0.6       # gaussian blur radius (px)
    wb_jitter: float = 0.04       # white-balance gain jitter
    jpeg_quality: int = 30        # OV2640 compresses hard

    _PRESETS = {
        "off": None,
        "low": dict(read_sigma=3.0, shot_scale=0.03, chroma_sigma=4.0, vignette=0.2, blur_sigma=0.3, wb_jitter=0.02, jpeg_quality=45),
        "medium": dict(read_sigma=6.0, shot_scale=0.06, chroma_sigma=8.0, vignette=0.35, blur_sigma=0.6, wb_jitter=0.04, jpeg_quality=30),
        "high": dict(read_sigma=11.0, shot_scale=0.11, chroma_sigma=15.0, vignette=0.5, blur_sigma=1.0, wb_jitter=0.07, jpeg_quality=20),
    }

    @classmethod
    def from_spec(cls, spec) -> "CameraNoise | None":
        if spec is None or spec is False:
            return None
        if isinstance(spec, str):
            preset = cls._PRESETS.get(spec.lower(), cls._PRESETS["medium"] if spec else None)
            return cls(**preset) if preset else None
        if isinstance(spec, dict):
            if spec.get("enabled") is False:
                return None
            fields = {k: spec[k] for k in spec if k in cls.__slots__ and k != "enabled"}
            return cls(**fields)
        return None


# Per-face shading factors (top brightest, bottom darkest) for box faces.
_FACE_SHADE = {"top": 1.0, "side_x": 0.82, "side_y": 0.68, "bottom": 0.5}


class CameraRenderer:
    def __init__(
        self,
        config: CameraConfig | None = None,
        scene: Scene | str | None = None,
        noise: "CameraNoise | str | dict | None" = None,
    ) -> None:
        if not _PIL:
            raise RuntimeError("Pillow is required for the sim renderer")
        self.config = config or CameraConfig()
        self.scene = scene if isinstance(scene, Scene) else build_scene(scene)
        self.noise = noise if isinstance(noise, CameraNoise) else CameraNoise.from_spec(noise)
        self._rng = np.random.default_rng()
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
        t: float = 0.0,
    ) -> list[bytes]:
        pos = np.asarray(pos, dtype=np.float64)
        quat_wxyz = np.asarray(quat_wxyz, dtype=np.float64)
        goals = np.asarray(goals, dtype=np.float64)
        rot = _rotmats(quat_wxyz)
        # Moving-object faces for this instant (shared across all drone views).
        dyn_quads = []
        for box in dynamic_objects(self.scene, t):
            dyn_quads.extend(_box_faces(box))
        idxs = list(range(pos.shape[0])) if indices is None else indices
        return [self._render_one(i, pos, rot, goals, dyn_quads) for i in idxs]

    def _render_one(self, i: int, pos: np.ndarray, rot: np.ndarray, goals: np.ndarray, dyn_quads=()) -> bytes:
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
        for corners, color in (*self._static_quads, *dyn_quads):
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

        quality = cfg.jpeg_quality
        if self.noise is not None and self.noise.enabled:
            img = self._apply_noise(img)
            quality = self.noise.jpeg_quality
        buffer = io.BytesIO()
        img.save(buffer, format="JPEG", quality=quality)
        return buffer.getvalue()

    def _apply_noise(self, img: "Image.Image") -> "Image.Image":
        n = self.noise
        arr = np.asarray(img, dtype=np.float32)
        h, w = arr.shape[0], arr.shape[1]
        # Signal-dependent (shot) + read noise.
        sigma = n.read_sigma + n.shot_scale * np.sqrt(np.clip(arr, 0, 255) * 255.0)
        arr = arr + self._rng.normal(0.0, 1.0, arr.shape).astype(np.float32) * sigma
        # Per-channel chroma noise.
        if n.chroma_sigma > 0:
            arr += self._rng.normal(0.0, n.chroma_sigma, (h, w, 3)).astype(np.float32)
        # White-balance drift (per-frame gains).
        if n.wb_jitter > 0:
            gains = 1.0 + self._rng.normal(0.0, n.wb_jitter, 3).astype(np.float32)
            arr *= gains
        # Vignette (radial corner darkening).
        if n.vignette > 0:
            ys, xs = np.mgrid[0:h, 0:w]
            cx, cy = (w - 1) / 2.0, (h - 1) / 2.0
            r = np.sqrt(((xs - cx) / cx) ** 2 + ((ys - cy) / cy) ** 2) / np.sqrt(2.0)
            mask = (1.0 - n.vignette * (r ** 2)).astype(np.float32)
            arr *= mask[..., None]
        arr = np.clip(arr, 0, 255).astype(np.uint8)
        out = Image.fromarray(arr, mode="RGB")
        if n.blur_sigma > 0:
            out = out.filter(ImageFilter.GaussianBlur(radius=float(n.blur_sigma)))
        return out

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
