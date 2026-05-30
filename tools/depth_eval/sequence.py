"""Scripted camera-sequence generator (EVAL ONLY).

Drives a single forward-camera along a smooth, parallax-rich path through a
named sim scene and renders frames with the real ``CameraRenderer`` (same
optics + sensor noise the live sim produces). For each frame it emits exactly
what perception would see — a JPEG + a calibrated ``camera_pose`` dict — plus
the eval-side ground-truth depth from the raycast oracle.

The path matters: multi-view triangulation needs translation between views
(baseline) and the features must sweep across the image. A pure spin gives no
parallax; a pure dolly gives weak lateral motion. The default path combines a
forward dolly, a lateral sway, a height bob, and a gentle yaw sweep so every
surface is seen from genuinely different viewpoints.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from drone_control.cameras import get_camera
from drone_control.perception.segmentation import rotmat_to_quat_xyzw
from drone_control.sim.render import CameraConfig, CameraRenderer
from drone_control.sim.scenes import build_scene

from .oracle import raycast_depth


@dataclass(slots=True)
class Frame:
    index: int
    t: float
    jpeg: bytes
    pose: dict          # perception camera_pose: x,y,z + rotation_xyzw (right/down/fwd)
    center: np.ndarray  # [3]
    cam_rot: np.ndarray # [3,3] camera->world (cols: right, down, forward)
    gt_depth: np.ndarray  # [H,W] Euclidean ray length, NaN = sky


def _basis_from_forward(forward: np.ndarray) -> np.ndarray:
    """Body->world rotation (columns = body x/forward, y/right, z/up), level roll."""
    f = forward / (np.linalg.norm(forward) + 1e-12)
    world_up = np.array([0.0, 0.0, 1.0])
    if abs(float(f @ world_up)) > 0.999:  # looking near-vertical: pick a stable right
        world_up = np.array([0.0, 1.0, 0.0])
    right = np.cross(world_up, f)
    right /= np.linalg.norm(right) + 1e-12
    up = np.cross(f, right)
    return np.column_stack([f, right, up])  # body->world


def default_path(i: int, n: int) -> tuple[np.ndarray, np.ndarray]:
    """(center, forward) for frame ``i`` of ``n`` — a warehouse-aisle fly-through.

    Flies along +y down an aisle gap (x ~ -6.75) with lateral sway, height bob,
    and a yaw sweep so geometry is seen from many viewpoints.
    """
    frac = i / max(1, n - 1)
    y = -9.0 + 17.0 * frac                       # dolly down the aisle
    x = -6.75 + 0.9 * np.sin(2.0 * np.pi * frac * 1.5)   # lateral sway
    z = 1.7 + 0.25 * np.sin(2.0 * np.pi * frac * 2.0)    # height bob
    center = np.array([x, y, z])
    yaw = 0.35 * np.sin(2.0 * np.pi * frac * 1.0)        # gentle look-around
    forward = np.array([np.sin(yaw), np.cos(yaw), -0.05])  # mostly +y, slight down
    return center, forward


def generate(
    scene_name: str = "warehouse",
    n: int = 24,
    image_size: int | None = None,
    noise: str = "medium",
    dt: float = 0.2,
    path=default_path,
    seed: int = 0,
    camera_model: str = "ov2640",
) -> list[Frame]:
    scene = build_scene(scene_name)
    cam = get_camera(camera_model)
    w = int(image_size) if image_size else cam.width
    h = int(round(w / cam.aspect))
    cfg = CameraConfig(width=w, height=h, fov_deg=cam.hfov_deg)
    cfg.far = max(cfg.far, scene.far)
    renderer = CameraRenderer(cfg, scene=scene, noise=noise)
    renderer._rng = np.random.default_rng(seed)  # deterministic sensor noise
    focal = (w / 2.0) / np.tan(np.deg2rad(cam.hfov_deg) / 2.0)
    intrinsics = {"fx": focal, "fy": focal, "cx": w / 2.0, "cy": h / 2.0, "width": w, "height": h}

    frames: list[Frame] = []
    for i in range(n):
        center, forward = path(i, n)
        body = _basis_from_forward(forward)             # body->world
        # Renderer wants body->world wxyz quat.
        quat_xyzw = rotmat_to_quat_xyzw(body)
        quat_wxyz = np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]])
        goal = np.array([0.0, 0.0, -100.0])             # out of view
        t = i * dt
        jpeg = renderer.render(
            center[None, :], quat_wxyz[None, :], goal[None, :], t=t
        )[0]
        # Calibrated camera pose exactly as SimEnvironment.camera_pose builds it:
        # cols of cam_rot = body right, body down(-up), body forward.
        cam_rot = np.column_stack([body[:, 1], -body[:, 2], body[:, 0]])
        pose = {
            "x": float(center[0]), "y": float(center[1]), "z": float(center[2]),
            "R": cam_rot.tolist(),  # lossless; the optical frame is left-handed
            "rotation_xyzw": rotmat_to_quat_xyzw(cam_rot),
            "intrinsics": intrinsics,
        }
        gt = raycast_depth(scene, center, cam_rot, cfg, t_sim=t, include_dynamic=True)
        frames.append(Frame(i, t, jpeg, pose, center, cam_rot, gt))
    return frames
