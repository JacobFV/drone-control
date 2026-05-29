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


@dataclass(slots=True)
class _Cloud:
    voxel: float = 0.15
    max_points: int = 50_000
    _keys: set = None  # type: ignore
    xyz: list = None   # type: ignore
    rgb: list = None   # type: ignore

    def __post_init__(self) -> None:
        self._keys = set()
        self.xyz = []
        self.rgb = []

    def add(self, points: np.ndarray, colors: np.ndarray) -> None:
        if len(self.xyz) >= self.max_points:
            return
        keys = np.round(points / self.voxel).astype(np.int64)
        for i in range(points.shape[0]):
            if len(self.xyz) >= self.max_points:
                break
            key = (int(keys[i, 0]), int(keys[i, 1]), int(keys[i, 2]))
            if key in self._keys:
                continue
            self._keys.add(key)
            self.xyz.append(points[i])
            self.rgb.append(colors[i])

    def arrays(self) -> tuple[np.ndarray, np.ndarray]:
        if not self.xyz:
            return np.zeros((0, 3)), np.zeros((0, 3))
        return np.asarray(self.xyz, dtype=np.float64), np.asarray(self.rgb, dtype=np.uint8)

    def snapshot(self, max_points: int) -> list[list[float]]:
        n = len(self.xyz)
        if n == 0:
            return []
        step = max(1, n // max_points)
        out = []
        for i in range(0, n, step):
            p = self.xyz[i]
            c = self.rgb[i]
            out.append([round(float(p[0]), 2), round(float(p[1]), 2), round(float(p[2]), 2),
                        int(c[0]), int(c[1]), int(c[2])])
        return out

    def clear(self) -> None:
        self._keys.clear()
        self.xyz.clear()
        self.rgb.clear()


class DepthEstimator:
    def __init__(
        self,
        *,
        model_name: str = DEFAULT_MODEL,
        fov_deg: float = 75.0,
        near: float = 0.5,
        far: float = 12.0,
        stride: int = 6,
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
                "points": len(self._cloud.xyz),
                "dronesWithDepth": sorted(self._depth_jpeg.keys()),
            }

    # -- per-frame ---------------------------------------------------------

    def process(
        self,
        drone_id: str,
        jpeg: bytes,
        pose: dict[str, Any] | None,
        cam_rot: np.ndarray | None = None,
    ) -> None:
        depth_norm, frame_rgb = self._infer(jpeg)
        if depth_norm is None:
            return
        metric = self.far - depth_norm * (self.far - self.near)  # near where depth_norm high
        colorized = _colorize(depth_norm)
        jpeg_out = _encode_jpeg(colorized)
        with self._lock:
            self._depth_jpeg[drone_id] = jpeg_out
            self._depth_map[drone_id] = metric
        if pose is not None:
            xyz, rgb = self._backproject(metric, frame_rgb, pose, cam_rot)
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
        cam_rot: np.ndarray | None = None,
    ) -> tuple[np.ndarray, np.ndarray]:
        from drone_control.perception.segmentation import _pose_center, _pose_rotation

        center = _pose_center(pose)
        # cam_rot columns are world directions of camera (right, down, forward).
        # Without it, fall back to the body rotation (z-forward assumption).
        rotation = cam_rot if cam_rot is not None else _pose_rotation(pose)
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
        with self._lock:
            return self._cloud.arrays()

    def reset(self) -> None:
        with self._lock:
            self._depth_jpeg.clear()
            self._depth_map.clear()
            self._cloud.clear()


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
