"""
Headless synthetic forward-camera renderer for the sim.

A minimal pinhole rasteriser (numpy + PIL) that draws, from each drone's
viewpoint, a coloured/textured scene (sky or ceiling gradient, a checkerboard
ground, depth-sorted shaded box geometry from the active scene plan) plus the
policy cues: the drone's own goal marker and the other drones. It also paints
the physical extras the sim simulates — moving objects, fan/wind primitives,
wind streaks, atmospheric fog, and the deformed cloth flags. Output is JPEG
bytes per camera, matching the real stack's frame format. Not photoreal — a
learnable, deterministic visual world that varies per scene plan.
"""

from __future__ import annotations

import io
import math
from dataclasses import dataclass

import numpy as np

from . import textures
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
        self.fog_density = float(getattr(self.scene, "fog_density", 0.0))
        self._fog_color = np.array(self.scene.sky_bottom, dtype=float)
        self._static_quads = self._build_static_quads()
        self._fans = [s for s in self.scene.flows if s.kind == "fan"]
        self._streak_seeds = self._build_streak_seeds()
        self._tex_budget = 40  # max textured faces per view (perf bound)

    def render(
        self,
        pos: np.ndarray,
        quat_wxyz: np.ndarray,
        goals: np.ndarray,
        indices: list[int] | None = None,
        t: float = 0.0,
        wind: tuple[float, float, float] | None = None,
        rigids: list | None = None,
        particles: tuple | None = None,
        meshes: list | None = None,
        smoke: dict | None = None,
    ) -> list[bytes]:
        pos = np.asarray(pos, dtype=np.float64)
        quat_wxyz = np.asarray(quat_wxyz, dtype=np.float64)
        goals = np.asarray(goals, dtype=np.float64)
        rot = _rotmats(quat_wxyz)
        # Moving-object faces for this instant (shared across all drone views),
        # plus free rigid bodies (already provided as (corners, color, label)).
        dyn_quads = []
        for box in dynamic_objects(self.scene, t):
            dyn_quads.extend(_box_faces(box))
        if rigids:
            dyn_quads.extend(rigids)
        idxs = list(range(pos.shape[0])) if indices is None else indices
        return [
            self._render_one(i, pos, rot, goals, dyn_quads, wind, t, particles, meshes or [], smoke)
            for i in idxs
        ]

    def _render_one(self, i, pos, rot, goals, dyn_quads, wind, t, particles=None, meshes=(), smoke=None) -> bytes:
        cfg = self.config
        cam = pos[i]
        r = rot[i]
        forward, up, right = r[:, 0], r[:, 2], r[:, 1]

        img, draw, project = self._paint_world(cam, forward, up, right, dyn_quads, wind, t, particles, meshes, smoke)

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

    def _paint_world(self, cam, forward, up, right, dyn_quads, wind, t, particles=None, meshes=(), smoke=None):
        """Paint everything that is part of the WORLD (sky, scene geometry,
        atmospherics, cloth, smoke) from an arbitrary camera. Returns
        ``(img, draw, project)`` so callers add their own cues (per-drone markers,
        goals) on top. Shared by the egocentric drone view and the omniscient view.
        """
        cfg = self.config
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

        def project_batch(points):
            """Vectorised projection of [N,3] world points -> (u[N], v[N], z[N],
            valid[N]); keeps hundreds of cloth-instance verts cheap."""
            rel = np.asarray(points, dtype=float) - cam
            z = rel @ forward
            valid = (z > 0.05) & (z <= cfg.far)
            zc = np.where(valid, z, 1.0)
            u = self.cx + self.focal * (rel @ right) / zc
            v = self.cy - self.focal * (rel @ up) / zc
            return u, v, z, valid

        # When smoke is present we build a per-pixel depth buffer so geometry
        # correctly occludes the volumetric puffs (true depth-aware compositing).
        want_depth = smoke is not None and len(smoke.get("pos", ())) > 0
        depth_tris: list = []

        # Depth-sort static + dynamic quads and paint far -> near with fog.
        rendered = []
        for corners, color, label in (*self._static_quads, *dyn_quads):
            projected = [project(c) for c in corners]
            if any(p is None for p in projected):
                continue
            mean_z = sum(p[2] for p in projected) / len(projected)
            rendered.append((mean_z, [(p[0], p[1]) for p in projected], color, label))
            if want_depth:
                for ti in range(1, len(projected) - 1):
                    depth_tris.append((projected[0], projected[ti], projected[ti + 1]))
        rendered.sort(key=lambda q: q[0], reverse=True)
        # Texture only the nearest N eligible faces per frame (bounds CPU when a
        # drone sits among dense geometry); the rest flat-fill.
        eligible = [idx for idx, q in enumerate(rendered) if self._textureable(q[1], q[3], q[0])]
        eligible.sort(key=lambda idx: rendered[idx][0])  # nearest first
        textured = set(eligible[: self._tex_budget])
        for k, (mean_z, poly, color, label) in enumerate(rendered):
            self._paint_face(draw, poly, color, label, mean_z, textured=(k in textured))

        # Atmospheric particles (dust / smoke) behind the foreground cues.
        if particles is not None:
            self._draw_particles(draw, project, particles)

        # Fan / wind-generator primitives.
        for spec in self._fans:
            self._draw_fan(draw, project, spec)

        # Wind streaks (faint motion lines advected by the ambient wind).
        if wind is not None and (wind[0] ** 2 + wind[1] ** 2 + wind[2] ** 2) > 0.4:
            self._draw_streaks(draw, project, np.asarray(wind, dtype=float), t)

        # Deformable cloth (PyBullet soft bodies + hundreds of instances): drawn
        # in one batched pass — all verts projected together, frustum-culled,
        # LOD'd — so instance count scales without per-mesh Python overhead.
        if meshes:
            self._draw_cloth_meshes(draw, project_batch, meshes, depth_tris if want_depth else None)

        # Volumetric smoke + fire: composite soft depth-sorted puffs, occluded
        # per-pixel by the scene depth buffer (smoke behind a wall stays hidden).
        if want_depth:
            zbuf = self._build_depth(depth_tris)
            img = self._composite_smoke(img, project, smoke, zbuf)
            draw = ImageDraw.Draw(img)

        return img, draw, project

    def render_omniscient(
        self,
        cam_pos: np.ndarray,
        target: np.ndarray,
        pos: np.ndarray,
        quat_wxyz: np.ndarray,
        goals: np.ndarray,
        colors: list[tuple[int, int, int]] | None = None,
        t: float = 0.0,
        wind: tuple[float, float, float] | None = None,
        rigids: list | None = None,
        particles: tuple | None = None,
        meshes: list | None = None,
        smoke: dict | None = None,
    ) -> bytes:
        """God's-eye view of the whole sim world from a free camera at
        ``cam_pos`` looking at ``target``. Draws the scene plus EVERY drone as a
        colored, heading-tagged marker and its goal — the omniscient ground-truth
        view (no sensor noise; this is not what any drone sees)."""
        cam_pos = np.asarray(cam_pos, float)
        target = np.asarray(target, float)
        pos = np.asarray(pos, float)
        forward = target - cam_pos
        forward = forward / (np.linalg.norm(forward) + 1e-9)
        world_up = np.array([0.0, 0.0, 1.0])
        if abs(float(forward @ world_up)) > 0.98:
            world_up = np.array([0.0, 1.0, 0.0])
        right = np.cross(forward, world_up); right /= np.linalg.norm(right) + 1e-9
        up = np.cross(right, forward); up /= np.linalg.norm(up) + 1e-9

        dyn_quads = []
        for box in dynamic_objects(self.scene, t):
            dyn_quads.extend(_box_faces(box))
        if rigids:
            dyn_quads.extend(rigids)

        img, draw, project = self._paint_world(
            cam_pos, forward, up, right, dyn_quads, wind, t, particles, meshes or [], smoke)

        rot = _rotmats(np.asarray(quat_wxyz, float))
        goals = np.asarray(goals, float)
        for j in range(pos.shape[0]):
            color = tuple(colors[j]) if colors and j < len(colors) else (120, 200, 255)
            pj = project(pos[j])
            pg = project(goals[j]) if j < len(goals) else None
            if pg is not None and pj is not None:
                draw.line([pj[0], pj[1], pg[0], pg[1]], fill=(*color, ), width=1)
                gr = max(2.0, 90.0 / pg[2])
                draw.ellipse([pg[0] - gr, pg[1] - gr, pg[0] + gr, pg[1] + gr], outline=color, width=1)
            if pj is None:
                continue
            rad = max(2.5, 130.0 / pj[2])
            draw.ellipse([pj[0] - rad, pj[1] - rad, pj[0] + rad, pj[1] + rad], fill=color, outline=(15, 18, 22))
            # Heading whisker: project a short segment along the body +x (forward).
            head = pos[j] + rot[j][:, 0] * 0.5
            ph = project(head)
            if ph is not None:
                draw.line([pj[0], pj[1], ph[0], ph[1]], fill=(245, 245, 250), width=1)

        buffer = io.BytesIO()
        img.save(buffer, format="JPEG", quality=self.config.jpeg_quality)
        return buffer.getvalue()

    # -------------------------------------------------------------- texturing

    def _fog(self, color, z) -> tuple[int, int, int]:
        if self.fog_density <= 0.0:
            return color
        f = 1.0 - math.exp(-self.fog_density * max(0.0, z))
        c = np.asarray(color, dtype=float)
        out = c * (1.0 - f) + self._fog_color * f
        return (int(out[0]), int(out[1]), int(out[2]))

    def _textureable(self, poly, label, mean_z) -> bool:
        if len(poly) != 4 or label in ("floor", "ceiling") or mean_z > 26.0:
            return False
        xs = [p[0] for p in poly]
        ys = [p[1] for p in poly]
        return (max(xs) - min(xs)) >= 10 and (max(ys) - min(ys)) >= 10

    def _paint_face(self, draw, poly, color, label, mean_z, *, textured: bool = True) -> None:
        """Paint a projected quad. Near, large faces get a material texture
        (a grid of cells modulating the colour); far/small faces flat-fill."""
        if not textured or not self._textureable(poly, label, mean_z):
            draw.polygon(poly, fill=self._fog(color, mean_z))
            return

        rows, cols = textures.cell_res(label)
        cells = textures.texture_cells(label, color, rows, cols)
        for r in range(rows):
            for c in range(cols):
                quad = [
                    _bilerp(poly, c / cols, r / rows),
                    _bilerp(poly, (c + 1) / cols, r / rows),
                    _bilerp(poly, (c + 1) / cols, (r + 1) / rows),
                    _bilerp(poly, c / cols, (r + 1) / rows),
                ]
                col = tuple(int(v) for v in cells[r, c])
                draw.polygon(quad, fill=self._fog(col, mean_z))

    def _draw_particles(self, draw, project, particles) -> None:
        pos, rgba, size = particles
        if pos is None or len(pos) == 0:
            return
        fog = self._fog_color
        for k in range(len(pos)):
            proj = project(pos[k])
            if proj is None:
                continue
            u, v, z = proj
            alpha = float(rgba[k][3]) / 255.0
            if alpha < 0.06:
                continue
            # Approximate alpha over the scene by blending toward the horizon.
            base = np.asarray(rgba[k][:3], dtype=float)
            col = base * alpha + fog * (1.0 - alpha)
            rad = max(0.6, float(size[k]) * 1.2 / max(0.4, z))
            draw.ellipse(
                [u - rad, v - rad, u + rad, v + rad],
                fill=(int(col[0]), int(col[1]), int(col[2])),
            )

    # ------------------------------------------------------------ flow extras

    def _draw_fan(self, draw, project, spec) -> None:
        p = spec.params
        origin = np.array(p.get("pos", (0.0, 0.0, 1.0)), dtype=float)
        direction = np.array(p.get("dir", (1.0, 0.0, 0.0)), dtype=float)
        n = float(np.linalg.norm(direction))
        direction = direction / n if n > 1e-9 else direction
        proj = project(origin)
        if proj is None:
            return
        u, v, z = proj
        rad = max(3.0, 90.0 / z)
        draw.ellipse([u - rad, v - rad, u + rad, v + rad], outline=(180, 188, 198), width=2)
        draw.ellipse([u - rad * 0.28, v - rad * 0.28, u + rad * 0.28, v + rad * 0.28], fill=(150, 158, 170))
        # Blades.
        for ang in (0.0, 1.05, 2.09, 3.14, 4.19, 5.24):
            draw.line([u, v, u + rad * 0.9 * math.cos(ang), v + rad * 0.9 * math.sin(ang)], fill=(120, 128, 140), width=1)
        # Jet direction streaks.
        for d in (1.5, 3.0, 4.5):
            tip = project(origin + direction * d)
            if tip is not None:
                draw.line([u, v, tip[0], tip[1]], fill=(200, 210, 220), width=1)

    def _build_streak_seeds(self) -> np.ndarray:
        rng = np.random.default_rng(7)
        r = self.config.grid_range
        n = 40
        xs = rng.uniform(-r, r, n)
        ys = rng.uniform(-r, r, n)
        zs = rng.uniform(0.6, 6.0, n)
        return np.stack([xs, ys, zs], axis=1)

    def _draw_streaks(self, draw, project, wind, t) -> None:
        wdir = wind / (np.linalg.norm(wind) + 1e-6)
        speed = float(np.linalg.norm(wind))
        # Drift the seed cloud with the wind so streaks appear to move.
        phase = (t * speed * 0.5) % 4.0
        for seed in self._streak_seeds:
            base = seed + wdir * phase
            a = project(base)
            b = project(base + wdir * min(1.2, 0.18 * speed + 0.3))
            if a is None or b is None:
                continue
            draw.line([(a[0], a[1]), (b[0], b[1])], fill=(220, 226, 232), width=1)

    def _build_depth(self, tris) -> np.ndarray:
        """Per-pixel nearest-geometry depth buffer (barycentric-interpolated z)
        from opaque triangles. Sky/empty pixels stay at +inf."""
        h, w = self.config.height, self.config.width
        zbuf = np.full((h, w), np.inf, dtype=np.float32)
        for p0, p1, p2 in tris:
            x0, y0, z0 = p0
            x1, y1, z1 = p1
            x2, y2, z2 = p2
            minx = max(0, int(np.floor(min(x0, x1, x2))))
            maxx = min(w - 1, int(np.ceil(max(x0, x1, x2))))
            miny = max(0, int(np.floor(min(y0, y1, y2))))
            maxy = min(h - 1, int(np.ceil(max(y0, y1, y2))))
            if maxx < minx or maxy < miny:
                continue
            denom = (y1 - y2) * (x0 - x2) + (x2 - x1) * (y0 - y2)
            if abs(denom) < 1e-9:
                continue
            ys, xs = np.mgrid[miny:maxy + 1, minx:maxx + 1]
            xs = xs.astype(np.float32)
            ys = ys.astype(np.float32)
            a = ((y1 - y2) * (xs - x2) + (x2 - x1) * (ys - y2)) / denom
            b = ((y2 - y0) * (xs - x2) + (x0 - x2) * (ys - y2)) / denom
            c = 1.0 - a - b
            inside = (a >= 0) & (b >= 0) & (c >= 0)
            if not inside.any():
                continue
            z = a * z0 + b * z1 + c * z2
            sub = zbuf[miny:maxy + 1, minx:maxx + 1]
            m = inside & (z < sub)
            sub[m] = z[m].astype(np.float32)
            zbuf[miny:maxy + 1, minx:maxx + 1] = sub
        return zbuf

    def _composite_smoke(self, img, project, smoke, zbuf=None) -> "Image.Image":
        """Alpha-composite volumetric smoke/fire puffs over the frame, occluded
        per-pixel by the scene depth buffer.

        Each puff is a soft radial splat: smoke uses 'over' compositing (it
        occludes), fire adds emissive glow. Painted far->near so nearer puffs
        layer correctly. A puff fragment is suppressed where it lies behind the
        scene geometry at that pixel (soft transition), so smoke does not bleed
        through walls.
        """
        pos = np.asarray(smoke["pos"], dtype=float)
        radius = np.asarray(smoke["radius"], dtype=float)
        opacity = np.asarray(smoke["opacity"], dtype=float)
        color = np.asarray(smoke["color"], dtype=float)
        emissive = np.asarray(smoke["emissive"], dtype=float)

        arr = np.asarray(img, dtype=np.float32)
        h, w = arr.shape[0], arr.shape[1]

        # Project all centres; keep those in front of the camera.
        proj = [project(p) for p in pos]
        items = []
        for k, pr in enumerate(proj):
            if pr is None:
                continue
            u, v, z = pr
            sr = max(2.0, self.focal * radius[k] / z)
            items.append((z, u, v, sr, k))
        items.sort(key=lambda q: q[0], reverse=True)  # far -> near

        for z, u, v, sr, k in items:
            x0, x1 = max(0, int(u - sr)), min(w, int(u + sr) + 1)
            y0, y1 = max(0, int(v - sr)), min(h, int(v + sr) + 1)
            if x1 <= x0 or y1 <= y0:
                continue
            ys, xs = np.mgrid[y0:y1, x0:x1]
            d2 = ((xs - u) / sr) ** 2 + ((ys - v) / sr) ** 2
            falloff = np.exp(-2.2 * d2).astype(np.float32)
            # Per-pixel occlusion: fade the puff out as it passes behind geometry.
            if zbuf is not None:
                scene_z = zbuf[y0:y1, x0:x1]
                vis = np.clip((scene_z - z) / 0.5 + 0.5, 0.0, 1.0).astype(np.float32)
                falloff = falloff * vis
            patch = arr[y0:y1, x0:x1, :]
            col = color[k].astype(np.float32)
            if emissive[k] > 0.05:
                # Fire: additive glow (bright, light-emitting).
                patch = patch + (emissive[k] * falloff)[..., None] * col
            else:
                a = np.clip(opacity[k] * falloff, 0.0, 1.0)[..., None]
                patch = patch * (1.0 - a) + col[None, None, :] * a
            arr[y0:y1, x0:x1, :] = patch

        return Image.fromarray(np.clip(arr, 0, 255).astype(np.uint8), mode="RGB")

    def _draw_cloth_meshes(self, draw, project_batch, meshes, depth_tris) -> None:
        """Batched cloth render: project every panel's verts in one numpy op,
        then per panel frustum-cull, LOD (far -> silhouette quad, near -> shaded
        triangles), and optionally emit depth triangles. Scales to hundreds of
        instances because the only per-panel work is a slice + a few PIL fills."""
        arrays = [np.asarray(m[0], dtype=float) for m in meshes]
        counts = np.array([len(a) for a in arrays])
        if counts.sum() == 0:
            return
        allv = np.concatenate(arrays, axis=0)
        u, v, z, valid = project_batch(allv)
        W, H = self.config.width, self.config.height

        # Vectorised per-panel screen bbox + visibility via segmented reductions,
        # so the Python loop below only does real work for VISIBLE panels — the
        # culled majority (hundreds of off-camera garments) cost ~nothing.
        starts = np.concatenate([[0], np.cumsum(counts)[:-1]]).astype(np.intp)
        BIG = 1e9
        umin = np.minimum.reduceat(np.where(valid, u, BIG), starts)
        umax = np.maximum.reduceat(np.where(valid, u, -BIG), starts)
        vmin = np.minimum.reduceat(np.where(valid, v, BIG), starts)
        vmax = np.maximum.reduceat(np.where(valid, v, -BIG), starts)
        vcnt = np.add.reduceat(valid.astype(np.int32), starts)
        zmean_all = np.add.reduceat(np.where(valid, z, 0.0), starts) / np.maximum(vcnt, 1)
        visible = (vcnt >= 3) & (umax >= 0) & (umin <= W) & (vmax >= 0) & (vmin <= H)
        small = np.maximum(umax - umin, vmax - vmin) < 14.0

        off = 0
        for mi, ((verts, faces, color, _label), nV) in enumerate(zip(meshes, counts)):
            sl = slice(off, off + nV)
            off += nV
            if not visible[mi] or not faces:
                continue
            vu, vv, vz, ok = u[sl], v[sl], z[sl], valid[sl]
            x0, x1, y0, y1 = float(umin[mi]), float(umax[mi]), float(vmin[mi]), float(vmax[mi])
            mean_z = float(zmean_all[mi])
            color = tuple(color)

            if small[mi]:
                # Far LOD: single silhouette quad.
                draw.polygon([(x0, y0), (x1, y0), (x1, y1), (x0, y1)],
                             fill=self._fog(_scale(color, 0.82), mean_z))
                if depth_tris is not None:
                    depth_tris.append(((x0, y0, mean_z), (x1, y0, mean_z), (x1, y1, mean_z)))
                    depth_tris.append(((x0, y0, mean_z), (x1, y1, mean_z), (x0, y1, mean_z)))
                continue

            tris = []
            verts = np.asarray(verts, dtype=float)
            for (a, b, c) in faces:
                if a >= nV or b >= nV or c >= nV or not (ok[a] and ok[b] and ok[c]):
                    continue
                tz = (vz[a] + vz[b] + vz[c]) / 3.0
                nrm = np.cross(verts[b] - verts[a], verts[c] - verts[a])
                nn = np.linalg.norm(nrm)
                shade = 0.66 + 0.34 * abs(float(nrm[2]) / nn) if nn > 1e-9 else 0.8
                pa, pb, pc = (vu[a], vv[a]), (vu[b], vv[b]), (vu[c], vv[c])
                tris.append((tz, [pa, pb, pc], self._fog(_scale(color, shade), tz)))
                if depth_tris is not None:
                    depth_tris.append(((vu[a], vv[a], vz[a]), (vu[b], vv[b], vz[b]), (vu[c], vv[c], vz[c])))
            tris.sort(key=lambda q: q[0], reverse=True)
            for _z, poly, col in tris:
                draw.polygon(poly, fill=col)

    # ------------------------------------------------------------------ noise

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
            rr = np.sqrt(((xs - cx) / cx) ** 2 + ((ys - cy) / cy) ** 2) / np.sqrt(2.0)
            mask = (1.0 - n.vignette * (rr ** 2)).astype(np.float32)
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

    def _build_static_quads(self) -> list[tuple[list[np.ndarray], tuple[int, int, int], str]]:
        """Floor checker tiles, optional ceiling tiles, and shaded box faces."""
        quads: list[tuple[list[np.ndarray], tuple[int, int, int], str]] = []
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
                    color, "floor",
                ))
                if scene.ceiling_z is not None:
                    z = scene.ceiling_z
                    cc = scene.ceiling_color if checker else _scale(scene.ceiling_color, 0.85)
                    quads.append((
                        [np.array([x0, y0, z]), np.array([x1, y0, z]),
                         np.array([x1, y1, z]), np.array([x0, y1, z])],
                        cc, "ceiling",
                    ))
        for box in scene.boxes:
            quads.extend(_box_faces(box))
        return quads


def _box_faces(box: Box) -> list[tuple[list[np.ndarray], tuple[int, int, int], str]]:
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
    return [(corners, _scale(box.color, _FACE_SHADE[kind]), box.label) for corners, kind in faces]


def _bilerp(poly, s: float, t: float) -> tuple[float, float]:
    """Bilinear point inside a projected quad ring [p0,p1,p2,p3]."""
    p0, p1, p2, p3 = poly[0], poly[1], poly[2], poly[3]
    ax = p0[0] + (p1[0] - p0[0]) * s
    ay = p0[1] + (p1[1] - p0[1]) * s
    bx = p3[0] + (p2[0] - p3[0]) * s
    by = p3[1] + (p2[1] - p3[1]) * s
    return (ax + (bx - ax) * t, ay + (by - ay) * t)


def _scale(color: tuple[int, int, int], factor: float) -> tuple[int, int, int]:
    return tuple(int(max(0, min(255, v * factor))) for v in color)


def _rotmats(quat_wxyz: np.ndarray) -> np.ndarray:
    import torch

    rot = quat_to_rotmat(torch.from_numpy(quat_wxyz.astype(np.float32)))
    return rot.numpy().astype(np.float64)
