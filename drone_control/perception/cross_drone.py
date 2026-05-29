"""
Automatic cross-drone co-registration for the live world model.

The hard part of fusing multiple drones into one Gaussian-splat world model is
putting every drone's camera into a single shared world frame. This module
solves that the honest way: feed the *union* of all drones' frames into one
COLMAP Structure-from-Motion run, which jointly solves all cameras into one
frame. From the result we get

  * a sparse 3D point cloud (xyz + rgb) in the shared frame -> seeds the engine,
  * the COLMAP pose of every bootstrap image,
  * a per-drone similarity transform ``world_T_drone`` (estimated by Umeyama
    alignment of each drone's visual-odometry camera centres to its COLMAP
    camera centres) so that *live* VO poses arriving after bootstrap can be
    placed into the shared frame.

COLMAP is driven via its CLI (``feature_extractor`` -> ``exhaustive_matcher`` ->
``mapper``) and its binary model is parsed directly, so this does not depend on
nerfstudio. The deterministic pieces (binary parsing, Umeyama) are unit-tested;
the full COLMAP run requires real overlapping frames and is exercised via the
live bootstrap path.
"""

from __future__ import annotations

import shutil
import struct
import subprocess
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np


# --------------------------------------------------------------------------- #
# COLMAP binary model parsing (standard format)
# --------------------------------------------------------------------------- #


@dataclass(slots=True)
class ColmapImage:
    image_id: int
    qvec: np.ndarray  # (4,) qw, qx, qy, qz  (world -> camera rotation)
    tvec: np.ndarray  # (3,) world -> camera translation
    camera_id: int
    name: str

    def camera_center(self) -> np.ndarray:
        """World-frame camera centre C = -R^T t."""

        rotation = _qvec_to_rotmat(self.qvec)
        return -rotation.T @ self.tvec


@dataclass(slots=True)
class ColmapModel:
    images: dict[int, ColmapImage] = field(default_factory=dict)
    points_xyz: np.ndarray = field(default_factory=lambda: np.zeros((0, 3), dtype=np.float64))
    points_rgb: np.ndarray = field(default_factory=lambda: np.zeros((0, 3), dtype=np.float64))

    def images_by_name(self) -> dict[str, ColmapImage]:
        return {image.name: image for image in self.images.values()}


def _qvec_to_rotmat(qvec: np.ndarray) -> np.ndarray:
    """COLMAP quaternion (qw, qx, qy, qz) -> 3x3 rotation matrix."""

    qw, qx, qy, qz = (float(v) for v in qvec)
    norm = (qw * qw + qx * qx + qy * qy + qz * qz) ** 0.5 or 1.0
    qw, qx, qy, qz = qw / norm, qx / norm, qy / norm, qz / norm
    return np.array(
        [
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
        ],
        dtype=np.float64,
    )


def read_images_binary(path: Path) -> dict[int, ColmapImage]:
    images: dict[int, ColmapImage] = {}
    with Path(path).open("rb") as handle:
        (num_images,) = struct.unpack("<Q", handle.read(8))
        for _ in range(num_images):
            image_id, qw, qx, qy, qz, tx, ty, tz, camera_id = struct.unpack("<i7di", handle.read(64))
            name_chars: list[bytes] = []
            while True:
                char = handle.read(1)
                if char == b"\x00" or char == b"":
                    break
                name_chars.append(char)
            name = b"".join(name_chars).decode("utf-8", errors="replace")
            (num_points2d,) = struct.unpack("<Q", handle.read(8))
            handle.read(num_points2d * 24)  # skip (x, y, point3D_id) per 2D point
            images[image_id] = ColmapImage(
                image_id=image_id,
                qvec=np.array([qw, qx, qy, qz], dtype=np.float64),
                tvec=np.array([tx, ty, tz], dtype=np.float64),
                camera_id=camera_id,
                name=name,
            )
    return images


def read_points3d_binary(path: Path) -> tuple[np.ndarray, np.ndarray]:
    xyz: list[list[float]] = []
    rgb: list[list[float]] = []
    with Path(path).open("rb") as handle:
        (num_points,) = struct.unpack("<Q", handle.read(8))
        for _ in range(num_points):
            _point_id, x, y, z, r, g, b, _error = struct.unpack("<Q3d3Bd", handle.read(43))
            (track_length,) = struct.unpack("<Q", handle.read(8))
            handle.read(track_length * 8)  # skip (image_id, point2D_idx) track
            xyz.append([x, y, z])
            rgb.append([r / 255.0, g / 255.0, b / 255.0])
    if not xyz:
        return np.zeros((0, 3), dtype=np.float64), np.zeros((0, 3), dtype=np.float64)
    return np.asarray(xyz, dtype=np.float64), np.asarray(rgb, dtype=np.float64)


def read_colmap_model(model_dir: Path) -> ColmapModel:
    model_dir = Path(model_dir)
    images = read_images_binary(model_dir / "images.bin")
    xyz, rgb = read_points3d_binary(model_dir / "points3D.bin")
    return ColmapModel(images=images, points_xyz=xyz, points_rgb=rgb)


def find_colmap_model(sparse_root: Path) -> Path | None:
    """COLMAP writes sparse models under sparse/0, sparse/1, ... pick the largest."""

    candidates = [p for p in sorted(Path(sparse_root).glob("*")) if (p / "images.bin").is_file()]
    if not candidates:
        return None
    return max(candidates, key=lambda p: (p / "images.bin").stat().st_size)


# --------------------------------------------------------------------------- #
# Similarity alignment (Umeyama)
# --------------------------------------------------------------------------- #


def umeyama_similarity(src: np.ndarray, dst: np.ndarray) -> np.ndarray:
    """Least-squares similarity (scale, rotation, translation) mapping src -> dst.

    Returns a 4x4 matrix M with M @ [src;1] ~= [dst;1]. Requires >= 3 points.
    """

    src = np.asarray(src, dtype=np.float64)
    dst = np.asarray(dst, dtype=np.float64)
    if src.shape != dst.shape or src.shape[0] < 3:
        raise ValueError("need matching point sets with at least 3 correspondences")
    n = src.shape[0]
    src_mean = src.mean(axis=0)
    dst_mean = dst.mean(axis=0)
    src_c = src - src_mean
    dst_c = dst - dst_mean
    cov = dst_c.T @ src_c / n
    u, d, vt = np.linalg.svd(cov)
    s = np.ones(3)
    if np.linalg.det(u) * np.linalg.det(vt) < 0:
        s[-1] = -1.0
    rotation = u @ np.diag(s) @ vt
    var_src = (src_c ** 2).sum() / n
    scale = float((d * s).sum() / var_src) if var_src > 1e-12 else 1.0
    translation = dst_mean - scale * rotation @ src_mean
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = scale * rotation
    matrix[:3, 3] = translation
    return matrix


# --------------------------------------------------------------------------- #
# COLMAP CLI driver
# --------------------------------------------------------------------------- #


def colmap_available() -> bool:
    return shutil.which("colmap") is not None


def run_colmap_sparse(images_dir: Path, work_dir: Path, *, single_camera_per_drone: bool = True) -> Path:
    """Run COLMAP SfM over an image folder and return the sparse model dir."""

    if not colmap_available():
        raise RuntimeError("colmap CLI not found on PATH")
    images_dir = Path(images_dir)
    work_dir = Path(work_dir)
    work_dir.mkdir(parents=True, exist_ok=True)
    database = work_dir / "database.db"
    sparse = work_dir / "sparse"
    sparse.mkdir(parents=True, exist_ok=True)

    _run(["colmap", "feature_extractor", "--database_path", str(database), "--image_path", str(images_dir)])
    _run(["colmap", "exhaustive_matcher", "--database_path", str(database)])
    _run([
        "colmap", "mapper",
        "--database_path", str(database),
        "--image_path", str(images_dir),
        "--output_path", str(sparse),
    ])
    model = find_colmap_model(sparse)
    if model is None:
        raise RuntimeError("COLMAP produced no sparse model (insufficient overlap?)")
    return model


def _run(command: list[str]) -> None:
    result = subprocess.run(command, capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(f"command failed ({result.returncode}): {' '.join(command)}\n{result.stderr[-2000:]}")


# --------------------------------------------------------------------------- #
# Bootstrap orchestration
# --------------------------------------------------------------------------- #


@dataclass(slots=True)
class CrossDroneBootstrapResult:
    transforms: dict[str, list[list[float]]]   # drone_id -> 4x4 world_T_drone
    points_xyz: np.ndarray
    points_rgb: np.ndarray
    registered_images: int
    drones: list[str]

    def as_status(self) -> dict[str, Any]:
        return {
            "registeredImages": self.registered_images,
            "drones": self.drones,
            "points": int(self.points_xyz.shape[0]),
            "transforms": self.transforms,
        }


def build_union_images(
    drone_frames: dict[str, list[Path]],
    out_dir: Path,
) -> dict[str, str]:
    """Copy each drone's frames into one folder, prefixed by drone id.

    Returns a map of ``output_image_name -> drone_id`` so COLMAP results can be
    attributed back to the originating drone.
    """

    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    owner: dict[str, str] = {}
    for drone_id, frames in drone_frames.items():
        safe = drone_id.replace("/", "_")
        for index, frame in enumerate(frames):
            name = f"{safe}__{index:06d}.jpg"
            shutil.copy2(frame, out_dir / name)
            owner[name] = drone_id
    return owner


def bootstrap_world_model(
    engine: Any,
    drone_frames: dict[str, list[Path]],
    work_dir: Path,
    *,
    vo_centers_by_image: dict[str, np.ndarray] | None = None,
) -> CrossDroneBootstrapResult:
    """End-to-end cross-drone bootstrap: union -> COLMAP -> seed engine + transforms.

    ``engine`` is a ``LiveSplatEngine`` (duck-typed: needs ``seed_from_points`` and
    ``set_drone_transform``). Raises if COLMAP fails to register a model.
    """

    work_dir = Path(work_dir)
    images_dir = work_dir / "union_images"
    owner = build_union_images(drone_frames, images_dir)
    model_dir = run_colmap_sparse(images_dir, work_dir)
    model = read_colmap_model(model_dir)
    transforms = compute_drone_transforms(model, owner, vo_centers_by_image or {})

    if model.points_xyz.shape[0] > 0:
        engine.seed_from_points(model.points_xyz, model.points_rgb)
    for drone_id, matrix in transforms.items():
        engine.set_drone_transform(drone_id, np.asarray(matrix, dtype=float))

    registered_drones = sorted({owner[name] for name in model.images_by_name() if name in owner})
    return CrossDroneBootstrapResult(
        transforms=transforms,
        points_xyz=model.points_xyz,
        points_rgb=model.points_rgb,
        registered_images=len(model.images),
        drones=registered_drones,
    )


def compute_drone_transforms(
    model: ColmapModel,
    image_owner: dict[str, str],
    vo_centers_by_image: dict[str, np.ndarray],
) -> dict[str, list[list[float]]]:
    """Per drone, align VO camera centres to COLMAP camera centres (Umeyama).

    ``vo_centers_by_image`` maps the union image name to that frame's
    visual-odometry camera centre (in the drone's own frame). Drones with fewer
    than 3 registered correspondences fall back to identity.
    """

    by_drone_src: dict[str, list[np.ndarray]] = {}
    by_drone_dst: dict[str, list[np.ndarray]] = {}
    images_by_name = model.images_by_name()
    for name, drone_id in image_owner.items():
        image = images_by_name.get(name)
        vo_center = vo_centers_by_image.get(name)
        if image is None or vo_center is None:
            continue
        by_drone_src.setdefault(drone_id, []).append(np.asarray(vo_center, dtype=np.float64))
        by_drone_dst.setdefault(drone_id, []).append(image.camera_center())

    transforms: dict[str, list[list[float]]] = {}
    for drone_id in by_drone_src:
        src = np.asarray(by_drone_src[drone_id])
        dst = np.asarray(by_drone_dst[drone_id])
        if src.shape[0] >= 3:
            matrix = umeyama_similarity(src, dst)
        else:
            matrix = np.eye(4)
        transforms[drone_id] = matrix.tolist()
    return transforms
