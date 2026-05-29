"""
Monocular depth estimation + back-projection to a point cloud.

Default model is **Depth Anything V2** via the ``transformers`` depth-estimation
pipeline (affine-invariant relative depth; CPU/GPU). The model is lazy-loaded so
the station runs without it; ``available()``/``unavailable_reason()`` mirror the
segmentation + splat stages.

The estimator produces, per frame:
  * a metric depth map (relative depth mapped into a near..far range),
  * a colorized depth JPEG for the depth tile,
  * back-projected world points (pose + pinhole) accumulated into a voxelized
    point cloud that feeds the point-cloud tile/export, splat seeding, and
    grounded world-space segmentation.
"""

from __future__ import annotations

import io
import threading
from dataclasses import dataclass
from typing import Any

import numpy as np

DEFAULT_MODEL = "depth-anything/Depth-Anything-V2-Small-hf"

_LOCK = threading.Lock()
_PIPELINE: Any | None = None


def available() -> bool:
    return unavailable_reason() is None


def unavailable_reason() -> str | None:
    try:
        import transformers  # noqa: F401
    except Exception as exc:  # pragma: no cover - depends on environment
        return f"transformers not installed ({exc})"
    return None


def _load_pipeline(model_name: str) -> Any:
    global _PIPELINE
    with _LOCK:
        if _PIPELINE is not None:
            return _PIPELINE
        from transformers import pipeline

        _PIPELINE = pipeline(task="depth-estimation", model=model_name)
        return _PIPELINE


# Turbo-ish 5-stop colormap (blue -> cyan -> green -> yellow -> red).
_CMAP = np.array(
    [[48, 18, 130], [33, 144, 200], [60, 200, 120], [240, 220, 60], [230, 60, 40]],
    dtype=np.float64,
)


def _colorize(depth_norm: np.ndarray) -> np.ndarray:
    """depth_norm in [0,1] (1 = nearest) -> RGB uint8 [H,W,3]."""
    x = np.clip(depth_norm, 0.0, 1.0) * (len(_CMAP) - 1)
    lo = np.floor(x).astype(int)
    hi = np.clip(lo + 1, 0, len(_CMAP) - 1)
    frac = (x - lo)[..., None]
    rgb = _CMAP[lo] * (1 - frac) + _CMAP[hi] * frac
    return rgb.astype(np.uint8)


class _Cloud:
    """Accumulating point cloud. Every point is streamed to disk (full fidelity,
    nothing discarded); a bounded in-memory ring holds the most recent points for
    live display and splat seeding."""

    def __init__(self, stream_path: "Path | None" = None, display_cap: int = 1_000_000) -> None:
        self.display_cap = display_cap
        self.stream_path = stream_path
        # Contiguous ring buffer [cap, 6] (x,y,z,r,g,b); cheap at 1M (~24 MB).
        self._buf = np.zeros((display_cap, 6), dtype=np.float32)
        self._w = 0          # write head
        self._n = 0          # filled count (<= cap)
        self._total = 0      # lifetime points (also streamed to disk)
        self._fh = open(stream_path, "wb") if stream_path is not None else None

    def add(self, points: np.ndarray, colors: np.ndarray) -> None:
        if points.shape[0] == 0:
            return
        rows = np.concatenate(
            [np.asarray(points, np.float32), np.asarray(colors, np.float32).reshape(-1, 3)], axis=1
        ).astype(np.float32)
        if self._fh is not None:
            self._fh.write(rows.tobytes())   # full fidelity, nothing dropped
            self._fh.flush()
        self._total += rows.shape[0]
        cap = self.display_cap
        if rows.shape[0] > cap:
            rows = rows[-cap:]
        k = rows.shape[0]
        end = self._w + k
        if end <= cap:
            self._buf[self._w:end] = rows
        else:
            first = cap - self._w
            self._buf[self._w:] = rows[:first]
            self._buf[: k - first] = rows[first:]
        self._w = (self._w + k) % cap
        self._n = min(cap, self._n + k)

    def _ordered(self) -> np.ndarray:
        """Ring contents oldest -> newest."""
        if self._n < self.display_cap:
            return self._buf[: self._n]
        return np.concatenate([self._buf[self._w:], self._buf[: self._w]], axis=0)

    def display_arrays(self) -> tuple[np.ndarray, np.ndarray]:
        arr = self._ordered()
        if arr.shape[0] == 0:
            return np.zeros((0, 3)), np.zeros((0, 3))
        return arr[:, 0:3].astype(np.float64), arr[:, 3:6].astype(np.uint8)

    def all_arrays(self) -> tuple[np.ndarray, np.ndarray]:
        """The full streamed cloud (read back from disk) — used for export."""
        if self._fh is not None:
            self._fh.flush()
        if self.stream_path is not None and self.stream_path.exists():
            raw = np.fromfile(self.stream_path, dtype=np.float32)
            m = raw.size // 6
            if m:
                arr = raw[: m * 6].reshape(m, 6)
                return arr[:, 0:3].astype(np.float64), arr[:, 3:6].astype(np.uint8)
        return self.display_arrays()

    def snapshot(self, max_points: int) -> list[list[float]]:
        """The latest ``max_points`` points (most-recently observed)."""
        arr = self._ordered()
        if arr.shape[0] == 0:
            return []
        arr = arr[-max_points:]
        return [
            [round(float(r[0]), 2), round(float(r[1]), 2), round(float(r[2]), 2),
             int(r[3]), int(r[4]), int(r[5])]
            for r in arr
        ]

    @property
    def total(self) -> int:
        return self._total

    def close(self) -> None:
        if self._fh is not None:
            self._fh.close()
            self._fh = None


class DepthEstimator:
    def __init__(
        self,
        *,
        model_name: str = DEFAULT_MODEL,
        fov_deg: float = 75.0,
        near: float = 0.5,
        far: float = 12.0,
        stride: int = 4,
    ) -> None:
        self.model_name = model_name
        self.fov_deg = fov_deg
        self.near = near
        self.far = far
        self.stride = stride
        self._lock = threading.RLock()
        self._depth_jpeg: dict[str, bytes] = {}
        self._depth_map: dict[str, np.ndarray] = {}   # metric depth [H,W]
        self._cloud = _Cloud()

    def available(self) -> bool:
        return available()

    def status(self) -> dict[str, Any]:
        with self._lock:
            return {
                "available": available(),
                "reason": unavailable_reason(),
                "model": self.model_name,
                "points": self._cloud.total,
                "dronesWithDepth": sorted(self._depth_jpeg.keys()),
            }

    # -- per-frame ---------------------------------------------------------

    def process(self, drone_id: str, jpeg: bytes, pose: dict[str, Any] | None) -> None:
        """Estimate depth for one camera frame and accumulate the cloud.

        ENVIRONMENT-AGNOSTIC: input is a JPEG frame + the calibrated camera pose,
        nothing else. The same monocular model runs for sim and real — there is
        no privileged ground-truth path (do not add one).
        """
        depth_norm, frame_rgb = self._infer(jpeg)
        if depth_norm is None:
            return
        metric = self.far - depth_norm * (self.far - self.near)  # near where depth_norm high
        # Temporal EMA per drone: noisy frames make monocular depth jitter
        # frame-to-frame, which scatters the back-projected cloud. Blending with
        # the previous frame's depth stabilises the geometry.
        with self._lock:
            prev = self._depth_map.get(drone_id)
            if prev is not None and prev.shape == metric.shape:
                metric = (0.6 * prev + 0.4 * metric).astype(np.float64)
            colorized_src = (self.far - metric) / (self.far - self.near)
            self._depth_jpeg[drone_id] = _encode_jpeg(_colorize(colorized_src))
            self._depth_map[drone_id] = metric
        if pose is not None:
            xyz, rgb = self._backproject(metric, frame_rgb, pose)
            if xyz.shape[0]:
                with self._lock:
                    self._cloud.add(xyz, rgb)

    def _infer(self, jpeg: bytes) -> tuple[np.ndarray | None, np.ndarray | None]:
        if not jpeg or not available():
            return None, None
        try:
            from PIL import Image

            image = Image.open(io.BytesIO(jpeg)).convert("RGB")
        except Exception:
            return None, None
        try:
            pipe = _load_pipeline(self.model_name)
            result = pipe(image)
            predicted = result.get("predicted_depth")
            if predicted is None:
                depth = np.asarray(result["depth"], dtype=np.float64)
            else:
                depth = predicted.squeeze().detach().cpu().numpy().astype(np.float64)
        except Exception:
            return None, None
        # Resize the model depth to the frame size for pixel-aligned colors.
        frame = np.asarray(image, dtype=np.uint8)
        depth = _resize(depth, frame.shape[1], frame.shape[0])
        dmin, dmax = float(depth.min()), float(depth.max())
        if dmax - dmin < 1e-6:
            return None, None
        depth_norm = (depth - dmin) / (dmax - dmin)  # 1 = nearest (DA: high = near)
        return depth_norm, frame

    def _backproject(
        self,
        metric: np.ndarray,
        frame_rgb: np.ndarray,
        pose: dict[str, Any],
    ) -> tuple[np.ndarray, np.ndarray]:
        from drone_control.perception.segmentation import _pose_center, _pose_rotation

        center = _pose_center(pose)
        # The calibrated camera pose's rotation maps camera (right, down, forward)
        # rays into the world — same for sim and real.
        rotation = _pose_rotation(pose)
        if center is None or rotation is None:
            return np.zeros((0, 3)), np.zeros((0, 3))
        h, w = metric.shape
        focal = (w / 2.0) / np.tan(np.deg2rad(self.fov_deg) / 2.0)
        cx, cy = w / 2.0, h / 2.0
        ys = np.arange(0, h, self.stride)
        xs = np.arange(0, w, self.stride)
        gx, gy = np.meshgrid(xs, ys)
        gx = gx.reshape(-1)
        gy = gy.reshape(-1)
        d = metric[gy, gx]
        # Reject far-plane pixels: monocular depth has no true far signal, so a
        # pixel mapped at (or very near) the far plane is sky / "no return", not a
        # real surface. Without multi-view parallax we can't distinguish a genuine
        # distant surface from that — so we treat the far band as invalid and drop
        # it rather than spraying a false dome of points at the clip distance.
        valid = (d < self.far * 0.96) & (d > self.near * 1.02)
        gx, gy, d = gx[valid], gy[valid], d[valid]
        if gx.size == 0:
            return np.zeros((0, 3)), np.zeros((0, 3))
        rays = np.stack([(gx - cx) / focal, (gy - cy) / focal, np.ones_like(gx, dtype=float)], axis=1)
        rays = rays / (np.linalg.norm(rays, axis=1, keepdims=True) + 1e-9)
        cam = rays * d[:, None]
        world = (rotation @ cam.T).T + center
        colors = frame_rgb[gy, gx, :]
        return world, colors

    # -- accessors ---------------------------------------------------------

    def latest_depth_jpeg(self, drone_id: str) -> bytes | None:
        with self._lock:
            return self._depth_jpeg.get(drone_id)

    def latest_depth_map(self, drone_id: str) -> np.ndarray | None:
        with self._lock:
            return self._depth_map.get(drone_id)

    def cloud_snapshot(self, max_points: int = 2500) -> list[list[float]]:
        with self._lock:
            return self._cloud.snapshot(max_points)

    def cloud_arrays(self) -> tuple[np.ndarray, np.ndarray]:
        """Recent (bounded) cloud — for live splat seeding."""
        with self._lock:
            return self._cloud.display_arrays()

    def cloud_full_arrays(self) -> tuple[np.ndarray, np.ndarray]:
        """The complete streamed cloud — for export (nothing discarded)."""
        with self._lock:
            return self._cloud.all_arrays()

    def reset(self, stream_path: "Path | None" = None) -> None:
        with self._lock:
            self._depth_jpeg.clear()
            self._depth_map.clear()
            self._cloud.close()
            self._cloud = _Cloud(stream_path)


def _resize(arr: np.ndarray, w: int, h: int) -> np.ndarray:
    if arr.shape == (h, w):
        return arr
    try:
        from PIL import Image

        img = Image.fromarray(arr.astype(np.float32), mode="F").resize((w, h))
        return np.asarray(img, dtype=np.float64)
    except Exception:
        ys = (np.linspace(0, arr.shape[0] - 1, h)).astype(int)
        xs = (np.linspace(0, arr.shape[1] - 1, w)).astype(int)
        return arr[ys][:, xs]


def _encode_jpeg(rgb: np.ndarray) -> bytes:
    from PIL import Image

    buffer = io.BytesIO()
    Image.fromarray(rgb, mode="RGB").save(buffer, format="JPEG", quality=82)
    return buffer.getvalue()


def write_ply(path, xyz: np.ndarray, rgb: np.ndarray) -> None:
    """Write a coloured ASCII PLY point cloud."""
    n = xyz.shape[0]
    with open(path, "w") as handle:
        handle.write("ply\nformat ascii 1.0\n")
        handle.write(f"element vertex {n}\n")
        handle.write("property float x\nproperty float y\nproperty float z\n")
        handle.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
        handle.write("end_header\n")
        for i in range(n):
            p = xyz[i]
            c = rgb[i] if i < len(rgb) else (200, 200, 200)
            handle.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f} {int(c[0])} {int(c[1])} {int(c[2])}\n")
