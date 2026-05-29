"""
Live, cross-drone Gaussian-splat world model.

A single persistent gaussian set is maintained on the GPU in one shared world
frame. Every drone publishes camera frames (+ poses); each frame becomes a
keyframe whose camera extrinsics are placed in the shared frame via a per-drone
``world_T_drone`` transform. A background thread continuously optimises the
gaussians against all keyframes (gsplat rasterisation + photometric loss + light
densification/pruning), so frames from *different drones jointly refine one
model* — that is the cross-drone fusion.

Scope / honest limits
----------------------
* The shared world frame is established by a one-off COLMAP bootstrap over the
  union of drones' frames (see ``ReconstructionManager``) which jointly solves
  all cameras; thereafter live poses are placed relative to each drone's
  bootstrap extrinsic. Until bootstrap, drones default to identity transforms
  (correct only if their odometry already shares a frame).
* Robust live tracking, loop closure and drift correction are NOT solved here.
  Reconstruction quality depends on the upstream VO poses and frame overlap.
  This is a working live-fusion core to iterate on, not a finished SLAM system.

The engine degrades cleanly: if torch/gsplat/CUDA are unavailable it reports
``available() == False`` and the existing offline reconstruction path is
unaffected.
"""

from __future__ import annotations

import io
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

try:  # torch + gsplat are optional; the engine is gated on their presence.
    import torch
    import gsplat

    _TORCH_IMPORT_ERROR: str | None = None
except Exception as exc:  # pragma: no cover - environment dependent
    torch = None  # type: ignore[assignment]
    gsplat = None  # type: ignore[assignment]
    _TORCH_IMPORT_ERROR = str(exc)


def available() -> bool:
    return torch is not None and gsplat is not None and bool(getattr(torch, "cuda", None)) and torch.cuda.is_available()


def unavailable_reason() -> str | None:
    if torch is None or gsplat is None:
        return _TORCH_IMPORT_ERROR or "torch/gsplat not installed"
    if not torch.cuda.is_available():
        return "CUDA device not available"
    return None


@dataclass(slots=True)
class LiveSplatConfig:
    image_max_size: int = 160       # downscale frames for live-rate optimisation
    default_depth: float = 3.0      # back-projection depth for self-bootstrap (metres)
    init_stride: int = 8            # pixel stride when seeding gaussians from a frame
    max_gaussians: int = 200_000
    keyframe_translation_m: float = 0.15   # min camera motion to accept a new keyframe
    learning_rate: float = 0.01
    densify_interval: int = 200
    prune_opacity: float = 0.01
    max_keyframes: int = 240


@dataclass(slots=True)
class Keyframe:
    drone_id: str
    image: Any            # torch.Tensor [H, W, 3] in [0, 1]
    viewmat: Any          # torch.Tensor [4, 4] world->camera
    K: Any                # torch.Tensor [3, 3]
    width: int
    height: int


class LiveSplatEngine:
    def __init__(self, config: LiveSplatConfig | None = None, *, device: str | None = None) -> None:
        if not available():
            raise RuntimeError(f"live splat engine unavailable: {unavailable_reason()}")
        self.config = config or LiveSplatConfig()
        self.device = torch.device(device or "cuda")
        self._lock = threading.Lock()
        self._keyframes: list[Keyframe] = []
        self._last_kf_center: dict[str, np.ndarray] = {}
        self._world_T_drone: dict[str, np.ndarray] = {}
        self._params: dict[str, Any] | None = None
        self._optimizer: Any = None
        self._step = 0
        self._last_loss: float | None = None
        self._running = False
        self._thread: threading.Thread | None = None
        self._mean_grad_accum: Any = None

    # ------------------------------------------------------------------ #
    # Lifecycle
    # ------------------------------------------------------------------ #

    def start(self) -> None:
        with self._lock:
            if self._running:
                return
            self._running = True
            self._thread = threading.Thread(target=self._optimize_loop, name="live-splat", daemon=True)
            self._thread.start()

    def stop(self) -> None:
        with self._lock:
            self._running = False
            thread = self._thread
        if thread is not None:
            thread.join(timeout=2.0)

    def set_drone_transform(self, drone_id: str, world_T_drone: np.ndarray) -> None:
        with self._lock:
            self._world_T_drone[drone_id] = np.asarray(world_T_drone, dtype=np.float64).reshape(4, 4)

    # ------------------------------------------------------------------ #
    # Ingestion
    # ------------------------------------------------------------------ #

    def ingest(self, drone_id: str, jpeg: bytes, pose: dict[str, Any] | None) -> bool:
        """Add a keyframe from one drone. Returns True if accepted."""

        if not jpeg:
            return False
        center = _pose_center(pose)
        if not self._should_keyframe(drone_id, center):
            return False
        image = self._decode_image(jpeg)
        if image is None:
            return False
        height, width = image.shape[0], image.shape[1]
        K = _intrinsics(width, height, self.device)
        viewmat = self._viewmat(drone_id, pose)
        keyframe = Keyframe(drone_id=drone_id, image=image, viewmat=viewmat, K=K, width=width, height=height)
        with self._lock:
            if self._params is None:
                self._seed_from_keyframe(keyframe)
            self._keyframes.append(keyframe)
            if len(self._keyframes) > self.config.max_keyframes:
                self._keyframes.pop(0)
            self._last_kf_center[drone_id] = center
        return True

    def _should_keyframe(self, drone_id: str, center: np.ndarray) -> bool:
        previous = self._last_kf_center.get(drone_id)
        if previous is None:
            return True
        return float(np.linalg.norm(center - previous)) >= self.config.keyframe_translation_m

    # ------------------------------------------------------------------ #
    # Optimisation
    # ------------------------------------------------------------------ #

    def _optimize_loop(self) -> None:
        while True:
            with self._lock:
                if not self._running:
                    return
                ready = self._params is not None and bool(self._keyframes)
            if not ready:
                time.sleep(0.02)
                continue
            try:
                self._optimize_step()
            except Exception:
                time.sleep(0.05)

    def _optimize_step(self) -> None:
        with self._lock:
            keyframe = self._keyframes[self._step % len(self._keyframes)]
            params = self._params
            optimizer = self._optimizer
        assert params is not None

        render, _alpha = self._render(params, keyframe)
        loss = torch.abs(render - keyframe.image).mean()
        optimizer.zero_grad(set_to_none=True)
        loss.backward()
        with torch.no_grad():
            grad = params["means"].grad
            if grad is not None:
                norm = grad.norm(dim=1)
                # The gaussian count can change under us (re-seed / densify / prune);
                # drop a stale accumulator whose length no longer matches.
                if self._mean_grad_accum is not None and self._mean_grad_accum.shape[0] != norm.shape[0]:
                    self._mean_grad_accum = None
                self._mean_grad_accum = norm if self._mean_grad_accum is None else self._mean_grad_accum + norm
        optimizer.step()

        self._last_loss = float(loss.detach().cpu())
        self._step += 1
        if self._step % self.config.densify_interval == 0:
            with self._lock:
                self._densify_and_prune()

    def _render(self, params: dict[str, Any], keyframe: Keyframe):
        colors, alphas, _meta = gsplat.rasterization(
            means=params["means"],
            quats=torch.nn.functional.normalize(params["quats"], dim=1),
            scales=torch.exp(params["scales_log"]),
            opacities=torch.sigmoid(params["opacities_raw"]),
            colors=torch.sigmoid(params["colors_raw"]),
            viewmats=keyframe.viewmat[None],
            Ks=keyframe.K[None],
            width=keyframe.width,
            height=keyframe.height,
        )
        return colors[0], alphas[0]

    # ------------------------------------------------------------------ #
    # Gaussian set construction / densification
    # ------------------------------------------------------------------ #

    def _seed_from_keyframe(self, keyframe: Keyframe) -> None:
        means, colors = self._backproject(keyframe)
        count = means.shape[0]
        scales_log = torch.full((count, 3), float(np.log(0.05)), device=self.device)
        quats = torch.zeros((count, 4), device=self.device)
        quats[:, 0] = 1.0
        opacities_raw = torch.full((count,), -2.0, device=self.device)  # sigmoid(-2) ~ 0.12
        colors_raw = torch.logit(colors.clamp(1e-3, 1 - 1e-3))
        self._params = {
            "means": torch.nn.Parameter(means),
            "quats": torch.nn.Parameter(quats),
            "scales_log": torch.nn.Parameter(scales_log),
            "opacities_raw": torch.nn.Parameter(opacities_raw),
            "colors_raw": torch.nn.Parameter(colors_raw),
        }
        self._optimizer = torch.optim.Adam(list(self._params.values()), lr=self.config.learning_rate)
        self._mean_grad_accum = None

    def _backproject(self, keyframe: Keyframe):
        stride = max(1, self.config.init_stride)
        height, width = keyframe.height, keyframe.width
        ys, xs = torch.meshgrid(
            torch.arange(0, height, stride, device=self.device),
            torch.arange(0, width, stride, device=self.device),
            indexing="ij",
        )
        xs = xs.reshape(-1).float()
        ys = ys.reshape(-1).float()
        fx = keyframe.K[0, 0]
        fy = keyframe.K[1, 1]
        cx = keyframe.K[0, 2]
        cy = keyframe.K[1, 2]
        depth = self.config.default_depth + torch.rand(xs.shape[0], device=self.device) * 0.5
        cam = torch.stack([(xs - cx) / fx * depth, (ys - cy) / fy * depth, depth], dim=1)
        cam_to_world = torch.linalg.inv(keyframe.viewmat)
        homog = torch.cat([cam, torch.ones((cam.shape[0], 1), device=self.device)], dim=1)
        world = (cam_to_world @ homog.T).T[:, :3].contiguous()
        colors = keyframe.image[ys.long(), xs.long(), :].contiguous()
        return world, colors

    def _densify_and_prune(self) -> None:
        params = self._params
        if params is None:
            return
        with torch.no_grad():
            opacity = torch.sigmoid(params["opacities_raw"])
            keep = opacity > self.config.prune_opacity
            if keep.sum() < opacity.numel():
                self._apply_mask(keep)
                if self._mean_grad_accum is not None:
                    self._mean_grad_accum = self._mean_grad_accum[keep]

            count = params["means"].shape[0]
            if self._mean_grad_accum is None or count >= self.config.max_gaussians:
                self._mean_grad_accum = None
                return
            # Clone the highest-gradient gaussians (under-reconstructed regions).
            budget = min(count, self.config.max_gaussians - count)
            if budget <= 0:
                self._mean_grad_accum = None
                return
            k = max(1, min(budget, count // 10 or 1))
            _values, idx = torch.topk(self._mean_grad_accum, k)
            self._clone(idx)
            self._mean_grad_accum = None

    def _apply_mask(self, mask: Any) -> None:
        params = self._params
        assert params is not None
        new = {name: torch.nn.Parameter(tensor.detach()[mask].contiguous()) for name, tensor in params.items()}
        self._params = new
        self._optimizer = torch.optim.Adam(list(new.values()), lr=self.config.learning_rate)

    def _clone(self, idx: Any) -> None:
        params = self._params
        assert params is not None
        jitter = torch.randn((idx.shape[0], 3), device=self.device) * 0.02
        new_tensors: dict[str, Any] = {}
        for name, tensor in params.items():
            base = tensor.detach()[idx]
            if name == "means":
                base = base + jitter
            elif name == "scales_log":
                base = base - float(np.log(1.6))  # split: shrink the clones
            new_tensors[name] = torch.cat([tensor.detach(), base], dim=0).contiguous()
        merged = {name: torch.nn.Parameter(value) for name, value in new_tensors.items()}
        self._params = merged
        self._optimizer = torch.optim.Adam(list(merged.values()), lr=self.config.learning_rate)

    # ------------------------------------------------------------------ #
    # Output
    # ------------------------------------------------------------------ #

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            gaussians = 0 if self._params is None else int(self._params["means"].shape[0])
            per_drone: dict[str, int] = {}
            for keyframe in self._keyframes:
                per_drone[keyframe.drone_id] = per_drone.get(keyframe.drone_id, 0) + 1
            return {
                "available": True,
                "running": self._running,
                "gaussians": gaussians,
                "keyframes": len(self._keyframes),
                "keyframesByDrone": per_drone,
                "drones": sorted(per_drone),
                "steps": self._step,
                "lastLoss": self._last_loss,
            }

    def bounds(self) -> dict[str, Any] | None:
        """Centre + radius of the gaussian means, for framing a viewer camera."""
        with self._lock:
            if self._params is None:
                return None
            means = self._params["means"].detach().cpu().numpy()
        if means.shape[0] == 0:
            return None
        center = means.mean(axis=0)
        radius = float(np.linalg.norm(means - center, axis=1).max())
        return {"center": [float(v) for v in center], "radius": max(0.5, radius)}

    def export_ply(self, path: Path) -> Path:
        with self._lock:
            if self._params is None:
                raise RuntimeError("no gaussians to export yet")
            params = {name: tensor.detach().cpu().numpy() for name, tensor in self._params.items()}
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        _write_gaussian_ply(path, params)
        return path

    def _decode_image(self, jpeg: bytes):
        try:
            from PIL import Image

            with Image.open(io.BytesIO(jpeg)) as image:
                image = image.convert("RGB")
                width, height = image.size
                scale = self.config.image_max_size / max(width, height)
                if scale < 1.0:
                    image = image.resize((max(1, int(width * scale)), max(1, int(height * scale))))
                arr = np.asarray(image, dtype=np.float32) / 255.0
        except Exception:
            return None
        return torch.from_numpy(arr).to(self.device)

    def _viewmat(self, drone_id: str, pose: dict[str, Any] | None):
        c2w_drone = _camera_to_world(pose)
        world_T_drone = self._world_T_drone.get(drone_id, np.eye(4))
        c2w_world = apply_similarity_to_pose(world_T_drone, c2w_drone)
        viewmat = np.linalg.inv(c2w_world)
        return torch.from_numpy(viewmat.astype(np.float32)).to(self.device)

    def seed_from_points(self, xyz: np.ndarray, rgb: np.ndarray | None = None, scale: float = 0.05) -> int:
        """Initialise the gaussian set from a point cloud in the shared frame.

        ``rgb`` may be 0..1 or 0..255 (auto-normalised). ``scale`` is the initial
        gaussian radius in world units — too small renders as invisible specks at
        scene scale, so callers seeding a metric cloud should pass a visible size.
        Returns the gaussian count.
        """

        xyz = np.asarray(xyz, dtype=np.float32)
        if xyz.ndim != 2 or xyz.shape[1] != 3 or xyz.shape[0] == 0:
            raise ValueError("xyz must be a non-empty (N, 3) array")
        if rgb is None:
            rgb = np.full_like(xyz, 0.5)
        rgb = np.asarray(rgb, dtype=np.float32).reshape(-1, 3)
        if float(rgb.max()) > 1.5:           # passed as 0..255
            rgb = rgb / 255.0
        count = xyz.shape[0]
        with self._lock:
            means = torch.from_numpy(xyz).to(self.device)
            colors = torch.from_numpy(np.clip(rgb, 1e-3, 1 - 1e-3)).to(self.device)
            scales_log = torch.full((count, 3), float(np.log(max(1e-3, scale))), device=self.device)
            quats = torch.zeros((count, 4), device=self.device)
            quats[:, 0] = 1.0
            opacities_raw = torch.full((count,), -1.0, device=self.device)
            self._params = {
                "means": torch.nn.Parameter(means),
                "quats": torch.nn.Parameter(quats),
                "scales_log": torch.nn.Parameter(scales_log),
                "opacities_raw": torch.nn.Parameter(opacities_raw),
                "colors_raw": torch.nn.Parameter(torch.logit(colors)),
            }
            self._optimizer = torch.optim.Adam(list(self._params.values()), lr=self.config.learning_rate)
            self._mean_grad_accum = None
        return count


# --------------------------------------------------------------------------- #
# Geometry helpers
# --------------------------------------------------------------------------- #


def _pose_center(pose: dict[str, Any] | None) -> np.ndarray:
    if not pose:
        return np.zeros(3, dtype=np.float64)
    translation = pose.get("translation")
    if isinstance(translation, (list, tuple)) and len(translation) >= 3:
        return np.asarray(translation[:3], dtype=np.float64)
    return np.asarray(
        [float(pose.get("x", 0.0)), float(pose.get("y", 0.0)), float(pose.get("z", 0.0))],
        dtype=np.float64,
    )


def apply_similarity_to_pose(world_T_drone: np.ndarray, c2w_drone: np.ndarray) -> np.ndarray:
    """Place a drone-frame camera pose into the shared world frame.

    ``world_T_drone`` may be a similarity (scale*R, t) from cross-drone Umeyama
    alignment. gsplat needs a rigid extrinsic, so scale is applied only to the
    camera centre while the rotation stays orthonormal. Identity in -> pose out
    unchanged.
    """

    world_T_drone = np.asarray(world_T_drone, dtype=np.float64).reshape(4, 4)
    c2w_drone = np.asarray(c2w_drone, dtype=np.float64).reshape(4, 4)
    m3 = world_T_drone[:3, :3]
    scale = float(np.cbrt(max(abs(np.linalg.det(m3)), 1e-12)))
    r_align = m3 / scale if scale > 1e-9 else np.eye(3)
    t_align = world_T_drone[:3, 3]
    c2w_world = np.eye(4)
    c2w_world[:3, :3] = r_align @ c2w_drone[:3, :3]
    c2w_world[:3, 3] = scale * (r_align @ c2w_drone[:3, 3]) + t_align
    return c2w_world


def _camera_to_world(pose: dict[str, Any] | None) -> np.ndarray:
    c2w = np.eye(4, dtype=np.float64)
    if not pose:
        return c2w
    rotation = pose.get("rotation_xyzw") or pose.get("rotation")
    if isinstance(rotation, (list, tuple)) and len(rotation) == 4:
        qx, qy, qz, qw = (float(v) for v in rotation)
        c2w[:3, :3] = _quat_xyzw_to_matrix(qx, qy, qz, qw)
    c2w[:3, 3] = _pose_center(pose)
    return c2w


def _quat_xyzw_to_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
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


def _intrinsics(width: int, height: int, device: Any):
    focal = 0.82 * max(width, height)
    K = torch.tensor(
        [[focal, 0.0, width / 2.0], [0.0, focal, height / 2.0], [0.0, 0.0, 1.0]],
        dtype=torch.float32,
        device=device,
    )
    return K


def _write_gaussian_ply(path: Path, params: dict[str, np.ndarray]) -> None:
    """Write the standard INRIA/3DGS .ply consumed by gsplat.js viewers."""

    means = params["means"].astype(np.float32)
    count = means.shape[0]
    # SH degree-0 DC term: invert the (RGB = SH_C0 * f_dc + 0.5) convention.
    sh_c0 = 0.28209479177387814
    colors = _sigmoid(params["colors_raw"]).astype(np.float32)
    f_dc = (colors - 0.5) / sh_c0
    opacity = params["opacities_raw"].astype(np.float32).reshape(-1, 1)  # stored as logit
    scales = params["scales_log"].astype(np.float32)
    quats = params["quats"].astype(np.float32)
    quats = quats / (np.linalg.norm(quats, axis=1, keepdims=True) + 1e-9)

    header = (
        "ply\n"
        "format binary_little_endian 1.0\n"
        f"element vertex {count}\n"
        "property float x\nproperty float y\nproperty float z\n"
        "property float nx\nproperty float ny\nproperty float nz\n"
        "property float f_dc_0\nproperty float f_dc_1\nproperty float f_dc_2\n"
        "property float opacity\n"
        "property float scale_0\nproperty float scale_1\nproperty float scale_2\n"
        "property float rot_0\nproperty float rot_1\nproperty float rot_2\nproperty float rot_3\n"
        "end_header\n"
    )
    normals = np.zeros((count, 3), dtype=np.float32)
    body = np.concatenate([means, normals, f_dc, opacity, scales, quats], axis=1).astype(np.float32)
    with path.open("wb") as handle:
        handle.write(header.encode("ascii"))
        handle.write(body.tobytes())


def _sigmoid(x: np.ndarray) -> np.ndarray:
    return 1.0 / (1.0 + np.exp(-x))
