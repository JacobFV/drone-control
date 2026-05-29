"""
Object segmentation: screen-space (per camera frame) and world-space (fused
into the splat/world frame).

Screen-space detections come from an instance-segmentation model. The default
is Ultralytics YOLO (``yolo11n-seg`` — instance masks + classes + boxes in one
pass, CPU-capable, GPU if present). The model is lazy-loaded so the rest of the
station runs without it; ``available()`` / ``unavailable_reason()`` mirror the
pattern in ``live_splat.py``. A custom ``model_step`` hook lets a different
model (SAM2, a detector, …) be dropped in.

World-space objects are produced by back-projecting each detection's image
centroid through the drone pose + pinhole intrinsics to a 3D ray at an assumed
depth, then fusing points across frames and drones by class + proximity. This
reuses the same pinhole/back-projection convention as ``live_splat._backproject``
so screen-space and splat-space stay consistent.
"""

from __future__ import annotations

import io
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable

import numpy as np


# ----------------------------------------------------------------- model load

_MODEL_LOCK = threading.Lock()
_MODEL: Any | None = None
_UNAVAILABLE_REASON: str | None = None

# Default model file; Ultralytics fetches it on first use if not cached.
DEFAULT_MODEL = "yolo11n-seg.pt"


def available() -> bool:
    return unavailable_reason() is None


def unavailable_reason() -> str | None:
    try:
        import ultralytics  # noqa: F401
    except Exception as exc:  # pragma: no cover - depends on environment
        return f"ultralytics not installed ({exc})"
    return None


def _load_model(model_name: str = DEFAULT_MODEL) -> Any:
    global _MODEL, _UNAVAILABLE_REASON
    with _MODEL_LOCK:
        if _MODEL is not None:
            return _MODEL
        from ultralytics import YOLO  # type: ignore

        _MODEL = YOLO(model_name)
        return _MODEL


# ----------------------------------------------------------------- data types


@dataclass(slots=True)
class ScreenDetection:
    cls: str
    score: float
    bbox: list[float]               # [x, y, w, h] in pixels
    centroid: list[float]           # [cx, cy] in pixels
    polygon: list[list[float]]      # normalized [[x,y],...] in [0,1], may be empty
    width: int
    height: int

    def as_dict(self) -> dict[str, Any]:
        return {
            "cls": self.cls,
            "score": round(self.score, 4),
            "bbox": [round(v, 2) for v in self.bbox],
            "centroid": [round(v, 2) for v in self.centroid],
            "polygon": [[round(x, 4), round(y, 4)] for x, y in self.polygon],
            "width": self.width,
            "height": self.height,
        }


@dataclass(slots=True)
class WorldObject:
    object_id: int
    cls: str
    centroid: np.ndarray            # [3]
    count: int = 1
    drones: set[str] = field(default_factory=set)
    score: float = 0.0
    last_seen: float = 0.0

    def as_dict(self) -> dict[str, Any]:
        return {
            "id": self.object_id,
            "cls": self.cls,
            "centroid": [round(float(v), 4) for v in self.centroid],
            "count": self.count,
            "drones": sorted(self.drones),
            "score": round(self.score, 4),
            "lastSeen": self.last_seen,
        }


# ----------------------------------------------------------------- segmenter


class Segmenter:
    """Runs screen-space segmentation and accumulates world-space objects."""

    def __init__(
        self,
        *,
        model_name: str = DEFAULT_MODEL,
        model_step: Callable[[np.ndarray], list[ScreenDetection]] | None = None,
        score_threshold: float = 0.35,
        default_depth: float = 4.0,
        fov_deg: float = 75.0,
        fuse_radius_m: float = 1.0,
        max_objects: int = 256,
    ) -> None:
        self.model_name = model_name
        self._model_step = model_step
        self.score_threshold = score_threshold
        self.default_depth = default_depth
        self.fov_deg = fov_deg
        self.fuse_radius_m = fuse_radius_m
        self.max_objects = max_objects

        self._lock = threading.RLock()
        self._screen: dict[str, list[ScreenDetection]] = {}
        self._objects: list[WorldObject] = []          # proximity-fused (model)
        self._next_object_id = 1

    # -- availability ------------------------------------------------------

    def available(self) -> bool:
        return self._model_step is not None or available()

    def status(self) -> dict[str, Any]:
        with self._lock:
            return {
                "available": self.available(),
                "reason": None if self.available() else unavailable_reason(),
                "model": "custom" if self._model_step is not None else self.model_name,
                "objects": len(self._objects),
                "dronesWithScreen": sorted(self._screen.keys()),
            }

    # -- screen-space ------------------------------------------------------

    def segment_frame(self, drone_id: str, jpeg: bytes) -> list[ScreenDetection]:
        """Decode a JPEG and produce screen-space detections for one drone."""

        image = _decode_jpeg(jpeg)
        if image is None:
            return []
        if self._model_step is not None:
            detections = self._model_step(image)
        elif available():
            detections = self._run_yolo(image)
        else:
            detections = []
        detections = [d for d in detections if d.score >= self.score_threshold]
        with self._lock:
            self._screen[drone_id] = detections
        return detections

    def _run_yolo(self, image: np.ndarray) -> list[ScreenDetection]:
        model = _load_model(self.model_name)
        height, width = image.shape[0], image.shape[1]
        # Ultralytics accepts an RGB ndarray.
        results = model.predict(image, verbose=False)
        detections: list[ScreenDetection] = []
        for result in results:
            names = result.names
            boxes = getattr(result, "boxes", None)
            masks = getattr(result, "masks", None)
            if boxes is None:
                continue
            xyxy = boxes.xyxy.cpu().numpy() if hasattr(boxes.xyxy, "cpu") else np.asarray(boxes.xyxy)
            confs = boxes.conf.cpu().numpy() if hasattr(boxes.conf, "cpu") else np.asarray(boxes.conf)
            cls_idx = boxes.cls.cpu().numpy() if hasattr(boxes.cls, "cpu") else np.asarray(boxes.cls)
            polygons = masks.xy if masks is not None else [None] * len(xyxy)
            for i in range(len(xyxy)):
                x1, y1, x2, y2 = (float(v) for v in xyxy[i])
                poly = polygons[i] if i < len(polygons) else None
                norm_poly: list[list[float]] = []
                if poly is not None and len(poly) > 0:
                    arr = np.asarray(poly, dtype=float)
                    step = max(1, len(arr) // 40)  # decimate to keep payload small
                    for px, py in arr[::step]:
                        norm_poly.append([px / width, py / height])
                detections.append(
                    ScreenDetection(
                        cls=str(names.get(int(cls_idx[i]), int(cls_idx[i]))),
                        score=float(confs[i]),
                        bbox=[x1, y1, x2 - x1, y2 - y1],
                        centroid=[(x1 + x2) / 2.0, (y1 + y2) / 2.0],
                        polygon=norm_poly,
                        width=width,
                        height=height,
                    )
                )
        return detections

    def latest_screen(self, drone_id: str) -> list[ScreenDetection]:
        with self._lock:
            return list(self._screen.get(drone_id, []))

    def screen_summary(self) -> dict[str, Any]:
        with self._lock:
            return {drone_id: [d.as_dict() for d in dets] for drone_id, dets in self._screen.items()}

    # -- world-space -------------------------------------------------------

    def project_to_world(
        self,
        drone_id: str,
        detections: list[ScreenDetection],
        pose: dict[str, Any] | None,
        depth_map: "np.ndarray | None" = None,
    ) -> None:
        """Back-project detection centroids through the calibrated camera pose and
        fuse. ``pose`` is the standard camera-to-world pose (``rotation_xyzw``);
        when a metric ``depth_map`` is supplied the centroid depth is sampled from
        it, else the fixed ``default_depth`` is used. Environment-agnostic — this
        runs identically for sim and real; it never sees scene ground truth.
        """

        if pose is None or not detections:
            return
        center = _pose_center(pose)
        rotation = _pose_rotation(pose)
        if center is None or rotation is None:
            return
        now = time.time()
        with self._lock:
            for det in detections:
                depth = self._sample_depth(det, depth_map)
                world_pt = self._backproject_centroid(det, center, rotation, depth=depth)
                self._fuse(drone_id, det.cls, world_pt, det.score, now)

    def _sample_depth(self, det: ScreenDetection, depth_map: "np.ndarray | None") -> float:
        if depth_map is None:
            return self.default_depth
        h, w = depth_map.shape[:2]
        u = int(min(w - 1, max(0, (det.centroid[0] / max(1, det.width)) * w)))
        v = int(min(h - 1, max(0, (det.centroid[1] / max(1, det.height)) * h)))
        depth = float(depth_map[v, u])
        return depth if depth > 0 else self.default_depth

    def _backproject_centroid(
        self,
        det: ScreenDetection,
        center: np.ndarray,
        rotation: np.ndarray,
        *,
        depth: float | None = None,
    ) -> np.ndarray:
        width, height = det.width, det.height
        focal = (width / 2.0) / np.tan(np.deg2rad(self.fov_deg) / 2.0)
        cx, cy = width / 2.0, height / 2.0
        px, py = det.centroid
        # Camera frame: x right, y down, z forward.
        ray_cam = np.array(
            [(px - cx) / focal, (py - cy) / focal, 1.0], dtype=float
        )
        ray_cam = ray_cam / (np.linalg.norm(ray_cam) + 1e-9)
        # ``rotation`` is the camera-to-world rotation (cols = world right/down/
        # forward) from the calibrated camera pose — identical for sim and real.
        ray_world = rotation @ ray_cam
        return center + ray_world * (depth if depth is not None else self.default_depth)

    def _fuse(self, drone_id: str, cls: str, world_pt: np.ndarray, score: float, now: float) -> None:
        for obj in self._objects:
            if obj.cls != cls:
                continue
            if float(np.linalg.norm(obj.centroid - world_pt)) <= self.fuse_radius_m:
                obj.centroid = (obj.centroid * obj.count + world_pt) / (obj.count + 1)
                obj.count += 1
                obj.drones.add(drone_id)
                obj.score = max(obj.score, score)
                obj.last_seen = now
                return
        self._objects.append(
            WorldObject(
                object_id=self._next_object_id,
                cls=cls,
                centroid=world_pt.astype(float),
                drones={drone_id},
                score=score,
                last_seen=now,
            )
        )
        self._next_object_id += 1

    def world_objects(self) -> list[dict[str, Any]]:
        with self._lock:
            return [obj.as_dict() for obj in self._objects]

    def reset(self) -> None:
        with self._lock:
            self._screen.clear()
            self._objects.clear()
            self._next_object_id = 1


# ----------------------------------------------------------------- helpers


def _decode_jpeg(jpeg: bytes) -> np.ndarray | None:
    if not jpeg:
        return None
    try:
        from PIL import Image

        with Image.open(io.BytesIO(jpeg)) as img:
            return np.asarray(img.convert("RGB"))
    except Exception:
        return None


def _pose_center(pose: dict[str, Any] | None) -> np.ndarray | None:
    if not pose:
        return None
    if "x" in pose and "y" in pose and "z" in pose:
        return np.array([float(pose["x"]), float(pose["y"]), float(pose["z"])], dtype=float)
    translation = pose.get("translation")
    if translation is not None and len(translation) >= 3:
        return np.array([float(v) for v in translation[:3]], dtype=float)
    return None


def _pose_rotation(pose: dict[str, Any] | None) -> np.ndarray | None:
    """Rotation matrix (3x3) from a pose dict, accepting wxyz or xyzw quats."""

    if not pose:
        return None
    if {"qw", "qx", "qy", "qz"} <= pose.keys():
        w, x, y, z = (float(pose["qw"]), float(pose["qx"]), float(pose["qy"]), float(pose["qz"]))
    elif pose.get("rotation_xyzw") is not None or pose.get("rotation") is not None:
        quat = pose.get("rotation_xyzw") or pose.get("rotation")
        if quat is None or len(quat) < 4:
            return np.eye(3)
        x, y, z, w = (float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3]))
    else:
        return np.eye(3)
    norm = (w * w + x * x + y * y + z * z) ** 0.5 or 1.0
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def rotmat_to_quat_xyzw(R: np.ndarray) -> list[float]:
    """3x3 rotation matrix -> [qx, qy, qz, qw]."""
    R = np.asarray(R, dtype=float)
    t = float(np.trace(R))
    if t > 0:
        s = (t + 1.0) ** 0.5 * 2
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = (1.0 + R[0, 0] - R[1, 1] - R[2, 2]) ** 0.5 * 2
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = (1.0 + R[1, 1] - R[0, 0] - R[2, 2]) ** 0.5 * 2
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = (1.0 + R[2, 2] - R[0, 0] - R[1, 1]) ** 0.5 * 2
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s
    return [float(qx), float(qy), float(qz), float(qw)]
