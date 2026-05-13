from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CAMERAS_PATH = REPO_ROOT / "config" / "cameras.json"


@dataclass(slots=True, frozen=True)
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int
    distortion: tuple[float, float, float, float, float]
    source: str

    def K(self) -> np.ndarray:
        return np.array(
            [[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )

    def D(self) -> np.ndarray:
        return np.array(self.distortion, dtype=np.float64)

    def as_dict(self) -> dict[str, object]:
        return {
            "fx": self.fx,
            "fy": self.fy,
            "cx": self.cx,
            "cy": self.cy,
            "width": self.width,
            "height": self.height,
            "distortion": list(self.distortion),
            "source": self.source,
        }


def estimate_intrinsics(width: int, height: int, hfov_deg: float = 78.0) -> CameraIntrinsics:
    """Heuristic intrinsics from image size and assumed horizontal FOV.

    The default 78° matches typical action-camera lenses. Valid only as a
    placeholder until a calibration target is shot.
    """
    fx = (width / 2.0) / math.tan(math.radians(hfov_deg / 2.0))
    return CameraIntrinsics(
        fx=fx,
        fy=fx,
        cx=width / 2.0,
        cy=height / 2.0,
        width=width,
        height=height,
        distortion=(0.0, 0.0, 0.0, 0.0, 0.0),
        source="estimated",
    )


def load_intrinsics(camera_id: str = "forward", *, path: Path | None = None) -> CameraIntrinsics | None:
    config_path = path or DEFAULT_CAMERAS_PATH
    if not config_path.is_file():
        return None
    try:
        data = json.loads(config_path.read_text())
    except (OSError, json.JSONDecodeError):
        return None
    cam = (data.get("cameras") or {}).get(camera_id)
    if not cam:
        return None
    distortion = list(cam.get("distortion") or [0.0, 0.0, 0.0, 0.0, 0.0])
    distortion = (distortion + [0.0] * 5)[:5]
    return CameraIntrinsics(
        fx=float(cam["fx"]),
        fy=float(cam["fy"]),
        cx=float(cam["cx"]),
        cy=float(cam["cy"]),
        width=int(cam["width"]),
        height=int(cam["height"]),
        distortion=tuple(float(v) for v in distortion),
        source="calibrated",
    )
