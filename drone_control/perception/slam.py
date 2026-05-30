"""
Multi-view SLAM depth front-end.

WHY THIS EXISTS — a single-image monocular depth prior (Depth Anything) was
measured against a raycast oracle on sim frames and found fundamentally broken:
even after best-fit affine alignment absRel stayed ~0.3–1.8 (good is <0.1),
δ<1.25 ~0.1–0.36 (good is >0.9), and depth ordering was *negatively* correlated
with truth on most frames. A monocular prior cannot recover this geometry. The
correct front-end triangulates depth from the camera's own motion across frames
— metric and structurally correct — which the calibrated poses make feasible.

This module is the structural backbone: a feature-based, known-pose multi-view
mapper.

  * ORB features per keyframe, matched across a temporal keyframe window.
  * Each new keyframe's features are chained into multi-view tracks (a feature
    seen across ≥2 keyframes), triangulated with a multi-view linear solve.
  * Every correspondence is gated three ways before it is trusted: the known-
    pose **epipolar** (Sampson) constraint rejects bad matches, **cheirality**
    rejects points behind a camera, and **reprojection error** rejects the rest.
  * Accepted points are metric world-frame 3D — they seed the point cloud and a
    sparse per-keyframe depth map that the dense plane-sweep stage (see
    ``mvs.py``) densifies.

ENVIRONMENT-AGNOSTIC: like the monocular path it replaces, the only input is a
camera frame + its calibrated camera pose. No sim ground truth is ever read
here — that privilege lives only in the eval harness (``tools/depth_eval``).
"""

from __future__ import annotations

import io
import threading
from collections import deque
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from .segmentation import _pose_center, _pose_rotation


def available() -> bool:
    return unavailable_reason() is None


def unavailable_reason() -> str | None:
    try:
        import cv2  # noqa: F401
    except Exception as exc:  # pragma: no cover - depends on environment
        return f"opencv (cv2) not installed ({exc})"
    return None


# --------------------------------------------------------------------------- #
#  Camera + keyframe model
# --------------------------------------------------------------------------- #

@dataclass(slots=True)
class Keyframe:
    """One posed view retained in the sliding window."""

    gray: np.ndarray          # [H,W] uint8
    rgb: np.ndarray           # [H,W,3] uint8
    center: np.ndarray        # [3]  camera centre in world
    R: np.ndarray             # [3,3] camera->world (cols: right, down, forward)
    K: np.ndarray             # [3,3] intrinsics
    P: np.ndarray             # [3,4] world->image projection  K [R^T | -R^T C]
    kp_xy: np.ndarray         # [M,2] keypoint pixel coords
    desc: np.ndarray          # [M,32] ORB descriptors (uint8)
    forward: np.ndarray       # [3] camera forward axis in world (= R[:,2])


def _intrinsics(w: int, h: int, fov_deg: float) -> np.ndarray:
    focal = (w / 2.0) / np.tan(np.deg2rad(fov_deg) / 2.0)
    return np.array([[focal, 0, w / 2.0], [0, focal, h / 2.0], [0, 0, 1.0]], dtype=np.float64)


def _intrinsics_from_pose(pose: dict | None, w: int, h: int, fov_deg: float) -> np.ndarray:
    """Prefer the pose's calibrated intrinsics (true focal for the fitted lens);
    fall back to an FOV guess only when none are supplied."""
    intr = (pose or {}).get("intrinsics")
    if intr and all(k in intr for k in ("fx", "fy", "cx", "cy")):
        return np.array([[float(intr["fx"]), 0, float(intr["cx"])],
                         [0, float(intr["fy"]), float(intr["cy"])],
                         [0, 0, 1.0]], dtype=np.float64)
    return _intrinsics(w, h, fov_deg)


def _projection(K: np.ndarray, R: np.ndarray, C: np.ndarray) -> np.ndarray:
    """world->image projection for a camera->world rotation R and centre C."""
    Rcw = R.T                       # world->camera
    t = -Rcw @ C
    return K @ np.hstack([Rcw, t[:, None]])


def _skew(v: np.ndarray) -> np.ndarray:
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]], dtype=np.float64)


# --------------------------------------------------------------------------- #
#  Per-drone multi-view mapper
# --------------------------------------------------------------------------- #

class _DroneMapper:
    """Sliding-window multi-view triangulator for a single camera stream."""

    def __init__(
        self,
        K: np.ndarray,
        *,
        window: int = 8,
        min_baseline: float = 0.12,
        min_parallax_deg: float = 1.2,
        min_views: int = 3,
        near: float = 0.4,
        far: float = 30.0,
        reproj_px: float = 1.5,
        sampson_px: float = 2.0,
        ratio: float = 0.7,
        detect_scale: float = 2.0,
    ) -> None:
        import cv2

        self.K = K
        self.Kinv = np.linalg.inv(K)
        self.window = window
        self.min_baseline = min_baseline
        self.min_parallax = np.deg2rad(min_parallax_deg)
        self.min_views = min_views
        self.near = near
        self.far = far
        self.reproj_px = reproj_px
        self.sampson_px = sampson_px
        self.ratio = ratio
        self.detect_scale = detect_scale
        self.keyframes: deque[Keyframe] = deque(maxlen=window)
        # Detect on an upscaled image: the live frames are tiny (~128px) and ORB
        # needs sub-pixel-ish corner localisation for usable disparity. We scale
        # keypoint coords back to native resolution after detection.
        self._orb = cv2.ORB_create(nfeatures=2500, scaleFactor=1.2, nlevels=8, fastThreshold=6)
        self._matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
        self._cv2 = cv2

    # -- feature extraction ------------------------------------------------

    def _detect(self, gray: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        img = gray
        s = self.detect_scale
        if s and s != 1.0:
            img = self._cv2.resize(gray, None, fx=s, fy=s, interpolation=self._cv2.INTER_CUBIC)
        kps, desc = self._orb.detectAndCompute(img, None)
        if not kps or desc is None:
            return np.zeros((0, 2), np.float64), np.zeros((0, 32), np.uint8)
        xy = np.array([kp.pt for kp in kps], dtype=np.float64) / (s if s else 1.0)
        return xy, desc

    def _match(self, desc_a: np.ndarray, desc_b: np.ndarray) -> np.ndarray:
        """Lowe-ratio matches a->b. Returns [P,2] index pairs (ia, ib)."""
        if desc_a.shape[0] == 0 or desc_b.shape[0] < 2:
            return np.zeros((0, 2), np.int64)
        knn = self._matcher.knnMatch(desc_a, desc_b, k=2)
        pairs = []
        for m in knn:
            if len(m) < 2:
                continue
            if m[0].distance < self.ratio * m[1].distance:
                pairs.append((m[0].queryIdx, m[0].trainIdx))
        return np.array(pairs, dtype=np.int64) if pairs else np.zeros((0, 2), np.int64)

    # -- geometry ----------------------------------------------------------

    def _sampson_mask(self, kf_a: Keyframe, kf_b: Keyframe, xa: np.ndarray, xb: np.ndarray) -> np.ndarray:
        """Reject matches violating the known-pose epipolar constraint.

        With both poses known the fundamental matrix is fixed (no estimation):
        F = K^-T [t]_x R K^-1 for the relative pose b<-a. We threshold the
        symmetric epipolar (Sampson) distance.
        """
        R_rel = kf_b.R.T @ kf_a.R                 # a-cam -> b-cam
        t_rel = kf_b.R.T @ (kf_a.center - kf_b.center)
        E = _skew(t_rel) @ R_rel
        F = self.Kinv.T @ E @ self.Kinv
        a1 = np.hstack([xa, np.ones((xa.shape[0], 1))])
        b1 = np.hstack([xb, np.ones((xb.shape[0], 1))])
        Fa = a1 @ F.T          # epipolar lines in b
        Ftb = b1 @ F           # epipolar lines in a
        num = np.sum(b1 * Fa, axis=1) ** 2
        den = Fa[:, 0] ** 2 + Fa[:, 1] ** 2 + Ftb[:, 0] ** 2 + Ftb[:, 1] ** 2
        sampson = num / np.maximum(den, 1e-12)
        return sampson < self.sampson_px ** 2

    def _triangulate_multi(self, Ps: list[np.ndarray], pts: list[np.ndarray]) -> np.ndarray:
        """Linear multi-view DLT for one track. Ps:[3,4]*V, pts:(u,v)*V -> X[3]."""
        rows = []
        for P, (u, v) in zip(Ps, pts):
            rows.append(u * P[2] - P[0])
            rows.append(v * P[2] - P[1])
        A = np.asarray(rows, dtype=np.float64)
        _, _, vt = np.linalg.svd(A)
        X = vt[-1]
        if abs(X[3]) < 1e-12:
            return np.array([np.nan, np.nan, np.nan])
        return X[:3] / X[3]

    # -- main step ---------------------------------------------------------

    def process(self, gray: np.ndarray, rgb: np.ndarray, center: np.ndarray, R: np.ndarray):
        """Insert a keyframe (if it adds baseline) and triangulate new structure.

        Returns (xyz[N,3] world, colors[N,3] uint8, depth_obs) where ``depth_obs``
        is a list of (px, py, depth) for the NEW keyframe — the sparse metric
        depth used to seed/anchor the dense stage and to score sparse accuracy.
        """
        empty = (np.zeros((0, 3)), np.zeros((0, 3), np.uint8), [])
        xy, desc = self._detect(gray)
        if xy.shape[0] < 8:
            return empty
        P = _projection(self.K, R, center)
        forward = R[:, 2]
        kf = Keyframe(gray=gray, rgb=rgb, center=center, R=R, K=self.K, P=P,
                      kp_xy=xy, desc=desc, forward=forward)

        if not self._gate_baseline(kf):
            return empty  # too close to the last keyframe — no parallax to add

        priors = list(self.keyframes)
        self.keyframes.append(kf)
        if not priors:
            return empty

        # Build a multi-view track per new-keyframe feature: gather every prior
        # keyframe that (a) has enough baseline and (b) matches + passes epipolar.
        n_new = xy.shape[0]
        obs_views: list[list] = [[] for _ in range(n_new)]   # list of (P, (u,v))
        for kf_j in priors:
            base = np.linalg.norm(kf.center - kf_j.center)
            if base < self.min_baseline:
                continue
            pairs = self._match(desc, kf_j.desc)
            if pairs.shape[0] == 0:
                continue
            xa = xy[pairs[:, 0]]
            xb = kf_j.kp_xy[pairs[:, 1]]
            ok = self._sampson_mask(kf, kf_j, xa, xb)
            for (ia, ib), keep in zip(pairs, ok):
                if keep:
                    obs_views[ia].append((kf_j.P, tuple(kf_j.kp_xy[ib])))

        centers = {id(kf_j.P): kf_j.center for kf_j in priors}
        centers[id(P)] = center
        out_xyz, out_rgb, depth_obs = [], [], []
        for ia in range(n_new):
            views = obs_views[ia]
            # Require multi-view support: with ≥3 views the linear solve is
            # over-determined, so reprojection error genuinely reveals a bad
            # match (a 2-view point always reprojects perfectly — useless gate).
            if len(views) + 1 < self.min_views:
                continue
            Ps = [P] + [v[0] for v in views]
            pts = [tuple(xy[ia])] + [v[1] for v in views]
            view_centers = [center] + [centers[id(v[0])] for v in views]
            X = self._triangulate_multi(Ps, pts)
            if not np.all(np.isfinite(X)):
                continue
            if not self._accept(X, Ps, pts, view_centers):
                continue
            depth = float((X - center) @ forward)
            px, py = xy[ia]
            color = rgb[int(round(py)) % rgb.shape[0], int(round(px)) % rgb.shape[1]]
            out_xyz.append(X)
            out_rgb.append(color)
            depth_obs.append((float(px), float(py), depth))

        if not out_xyz:
            return empty
        return (np.asarray(out_xyz, np.float64),
                np.asarray(out_rgb, np.uint8).reshape(-1, 3),
                depth_obs)

    def _gate_baseline(self, kf: Keyframe) -> bool:
        if not self.keyframes:
            return True
        last = self.keyframes[-1]
        if np.linalg.norm(kf.center - last.center) >= self.min_baseline:
            return True
        # Also accept a keyframe on substantial rotation (new viewpoint content).
        cos = float(np.clip(kf.forward @ last.forward, -1.0, 1.0))
        return np.arccos(cos) >= np.deg2rad(8.0)

    def _accept(self, X: np.ndarray, Ps: list[np.ndarray], pts: list[tuple],
                centers: list[np.ndarray]) -> bool:
        # Cheirality + depth range + reprojection error across ALL views.
        max_err = 0.0
        for P, (u, v) in zip(Ps, pts):
            xh = P @ np.append(X, 1.0)
            if xh[2] <= 1e-6:
                return False                      # behind this camera
            uu, vv = xh[0] / xh[2], xh[1] / xh[2]
            err = np.hypot(uu - u, vv - v)
            if err > max_err:
                max_err = err
        if max_err > self.reproj_px:
            return False
        # Depth range vs the anchor (first P is the new keyframe).
        depth = (Ps[0] @ np.append(X, 1.0))[2]
        if not (self.near < depth < self.far):
            return False
        # Parallax gate: the widest angle subtended at X by any pair of camera
        # centres must clear a threshold, else depth is numerically unconstrained
        # (the failure mode for far points under a short baseline).
        rays = [(X - c) / (np.linalg.norm(X - c) + 1e-12) for c in centers]
        max_ang = 0.0
        for i in range(len(rays)):
            for j in range(i + 1, len(rays)):
                cos = float(np.clip(rays[i] @ rays[j], -1.0, 1.0))
                max_ang = max(max_ang, np.arccos(cos))
        return max_ang >= self.min_parallax


# --------------------------------------------------------------------------- #
#  Public front-end (mirrors DepthEstimator's surface)
# --------------------------------------------------------------------------- #

from .depth import _Cloud, _colorize, _encode_jpeg  # reuse cloud + colour helpers
from . import mvs


class _VoxelGrid:
    """Voxel-fused point cloud — the key to a cloud that STAYS STABLE as dozens
    of frames stream in.

    Naively appending every frame's points makes re-observed surfaces pile up
    and, because each frame's depth is slightly noisy, grow fuzzier and heavier
    without end. Instead each voxel keeps a running mean of the positions/colours
    that fall in it plus an observation count. Re-observing a real surface just
    refines its single fused point (noise averages out → it converges and stops
    moving); a transient bad point lands in a voxel that never gets confirmed
    again. Displaying only voxels seen ``min_obs+`` times yields a bounded,
    denoised, temporally stable cloud.
    """

    def __init__(self, voxel: float = 0.1, min_obs: int = 2, max_voxels: int = 1_500_000,
                 strong_obs: int = 4, stale_frames: int = 20) -> None:
        self.voxel = voxel
        self.min_obs = min_obs
        self.max_voxels = max_voxels
        # A voxel is "permanent" once seen strong_obs+ times; weakly-seen voxels
        # not re-observed within stale_frames are evicted — that is how moving
        # objects (forklifts/AGV) get cleaned up instead of smearing a permanent
        # ghost trail through the cloud as the sim runs for dozens of frames.
        self.strong_obs = strong_obs
        self.stale_frames = stale_frames
        # key (ix,iy,iz) -> [sx, sy, sz, sr, sg, sb, n, last_seen]
        self._acc: dict[tuple, np.ndarray] = {}

    def add(self, xyz: np.ndarray, rgb: np.ndarray, frame: int = 0) -> None:
        if xyz.shape[0] == 0:
            return
        keys = np.floor(np.asarray(xyz) / self.voxel).astype(np.int64)
        rgb = np.asarray(rgb, dtype=np.float64).reshape(-1, 3)
        # Aggregate within this batch first (one dict op per occupied voxel), so
        # a frame that hits a voxel 50× costs one update, not 50.
        uniq, inv = np.unique(keys, axis=0, return_inverse=True)
        sums = np.zeros((uniq.shape[0], 6))
        np.add.at(sums, inv, np.concatenate([xyz, rgb], axis=1))
        counts = np.bincount(inv, minlength=uniq.shape[0]).astype(np.float64)
        for k, s, c in zip(map(tuple, uniq), sums, counts):
            a = self._acc.get(k)
            if a is None:
                self._acc[k] = np.array([*s, c, frame])
            else:
                a[:6] += s
                a[6] += c
                a[7] = frame
        if len(self._acc) > self.max_voxels:
            self._evict()

    def decay(self, frame: int) -> None:
        """Evict weakly-observed voxels that have gone stale (moving-object trails
        and one-off noise); keep everything seen strong_obs+ times."""
        stale = frame - self.stale_frames
        self._acc = {
            k: v for k, v in self._acc.items()
            if v[6] >= self.strong_obs or v[7] >= stale
        }

    def _evict(self) -> None:
        # Drop the least-observed voxels first (most likely noise) down to 90%.
        target = int(self.max_voxels * 0.9)
        items = sorted(self._acc.items(), key=lambda kv: kv[1][6], reverse=True)
        self._acc = dict(items[:target])

    def arrays(self) -> tuple[np.ndarray, np.ndarray]:
        if not self._acc:
            return np.zeros((0, 3)), np.zeros((0, 3), np.uint8)
        data = np.array([v for v in self._acc.values()])
        keep = data[:, 6] >= self.min_obs
        data = data[keep]
        if data.shape[0] == 0:
            return np.zeros((0, 3)), np.zeros((0, 3), np.uint8)
        n = data[:, 6:7]
        xyz = data[:, 0:3] / n
        rgb = np.clip(data[:, 3:6] / n, 0, 255).astype(np.uint8)
        return xyz, rgb

    def snapshot(self, max_points: int) -> list[list[float]]:
        xyz, rgb = self.arrays()
        if xyz.shape[0] > max_points:
            idx = np.linspace(0, xyz.shape[0] - 1, max_points).astype(int)
            xyz, rgb = xyz[idx], rgb[idx]
        return [[round(float(p[0]), 2), round(float(p[1]), 2), round(float(p[2]), 2),
                 int(c[0]), int(c[1]), int(c[2])] for p, c in zip(xyz, rgb)]

    @property
    def occupied(self) -> int:
        return len(self._acc)

    def clear(self) -> None:
        self._acc.clear()


@dataclass(slots=True)
class _Window:
    """Per-drone parallax-gated keyframe window for dense MVS."""

    grays: deque = field(default_factory=lambda: deque(maxlen=8))
    rgbs: deque = field(default_factory=lambda: deque(maxlen=8))
    Rs: deque = field(default_factory=lambda: deque(maxlen=8))
    Cs: deque = field(default_factory=lambda: deque(maxlen=8))


class MultiViewSLAM:
    """Multi-view SLAM depth front-end — drop-in for ``DepthEstimator``.

    Fed only frames + calibrated poses, it maintains a parallax-gated keyframe
    window per drone and runs a dense **plane-sweep MVS** stage
    (``mvs.plane_sweep`` + ``mvs.densify``) that produces the per-pixel metric
    depth map and the metric point cloud.

    Plane-sweep carries the depth because on the real input (noisy, JPEG-crushed,
    repetitive frames) discrete feature matching collapses — every descriptor has
    a near-twin — whereas per-pixel photo-consistency aggregated across the posed
    window stays robust. (The sparse ``_DroneMapper`` tracker remains for offline
    structural mapping but is not in the live depth path: it adds almost nothing
    on this imagery.) Depth is computed at a capped internal width then upsampled;
    the dense map feeds the depth tile + segmentation grounding, and the confident
    plane-sweep pixels back-project into the cloud after outlier removal.
    """

    def __init__(self, *, fov_deg: float = 75.0, near: float = 0.4, far: float = 25.0,
                 window: int = 8, min_baseline: float = 0.12,
                 n_depths: int = 64, census_radius: int = 3, patch: int = 7,
                 aggregate: int = 9, uniqueness: float = 0.93, max_cost: float = 0.6,
                 cloud_stride: int = 2, depth_width: int = 160,
                 outlier_k: int = 8, outlier_std: float = 2.0,
                 cloud_min_conf: float = 0.12, cloud_far_frac: float = 0.9,
                 cloud_max_depth: float = 8.0,
                 voxel_size: float = 0.1, voxel_min_obs: int = 2) -> None:
        self.fov_deg = fov_deg
        self.near = near
        self.far = far
        self.window = window
        self.min_baseline = min_baseline
        self.n_depths = n_depths
        self.census_radius = census_radius
        self.patch = patch
        self.aggregate = aggregate
        self.uniqueness = uniqueness
        self.max_cost = max_cost
        self.cloud_stride = cloud_stride
        # Depth is computed at a capped internal width: the ESP32 JPEG stream is
        # compression/noise-limited, so plane-sweep at the full sensor resolution
        # is slower AND noisier than at ~160 px (measured). The dense result is
        # upsampled back to the frame for the tile + segmentation grounding.
        self.depth_width = depth_width
        self.outlier_k = outlier_k        # statistical outlier removal: neighbours
        self.outlier_std = outlier_std    # drop points beyond mean+std*σ neighbour dist
        # Cloud-only gating (the depth MAP keeps everything for the tile): the
        # cloud should be clean metric structure, so we drop low-confidence pixels
        # and the unreliable far band where disparity is too small to trust —
        # that band is what sprays points through/past distant walls.
        self.cloud_min_conf = cloud_min_conf
        self.cloud_far_frac = cloud_far_frac
        # Absolute near cap for cloud points: beyond this, MVS disparity at this
        # resolution/noise is too small to trust and the points scatter (and
        # never stabilise). The depth MAP still keeps far values for the tile.
        self.cloud_max_depth = cloud_max_depth
        self._lock = threading.RLock()
        self._windows: dict[str, _Window] = {}
        self._K: np.ndarray | None = None
        self._depth_jpeg: dict[str, bytes] = {}
        self._depth_map: dict[str, np.ndarray] = {}   # dense Euclidean depth [H,W]
        self._conf: dict[str, np.ndarray] = {}
        self._cloud = _Cloud()                                   # full-fidelity disk stream (export)
        self._voxels = _VoxelGrid(voxel_size, voxel_min_obs)     # fused, stable display cloud
        self._frame_idx = 0                                      # drives voxel staleness/decay

    def available(self) -> bool:
        return available()

    def status(self) -> dict[str, Any]:
        with self._lock:
            return {
                "available": available(),
                "reason": unavailable_reason(),
                "method": "multi-view-slam",
                "points": self._cloud.total,           # lifetime observations (streamed)
                "fusedVoxels": self._voxels.occupied,  # stable fused-cloud size
                "keyframes": {d: len(w.grays) for d, w in self._windows.items()},
                "dronesWithDepth": sorted(self._depth_jpeg.keys()),
            }

    def _decode(self, jpeg: bytes):
        from PIL import Image
        img = Image.open(io.BytesIO(jpeg)).convert("RGB")
        rgb = np.asarray(img, dtype=np.uint8)
        gray = np.asarray(img.convert("L"), dtype=np.uint8)
        return gray, rgb

    def process(self, drone_id: str, jpeg: bytes, pose: dict[str, Any] | None) -> None:
        if not jpeg or not available() or pose is None:
            return
        center = _pose_center(pose)
        R = _pose_rotation(pose)
        if center is None or R is None:
            return
        try:
            gray, rgb = self._decode(jpeg)
        except Exception:
            return
        h, w = gray.shape
        with self._lock:
            if self._K is None:
                self._K = _intrinsics_from_pose(pose, w, h, self.fov_deg)
            K = self._K
            win = self._windows.setdefault(drone_id, _Window(
                deque(maxlen=self.window), deque(maxlen=self.window),
                deque(maxlen=self.window), deque(maxlen=self.window)))

        # Downscale to the internal compute width (cheaper + de-noises). The
        # compute-resolution intrinsics K_c scale with the resize factor.
        gray_c, rgb_c, K_c, scale = self._downscale(gray, rgb, K)
        gf = gray_c.astype(np.float32)

        # Parallax-gate the dense window: only keep a keyframe that adds baseline,
        # so the plane-sweep always has translation to triangulate against.
        if win.grays and np.linalg.norm(center - win.Cs[-1]) < self.min_baseline:
            return
        win.grays.append(gf); win.rgbs.append(rgb_c); win.Rs.append(R); win.Cs.append(center)
        if len(win.grays) < 3:
            return

        views = [(win.grays[i], win.Rs[i], win.Cs[i]) for i in range(len(win.grays))]
        ref = len(views) - 1
        try:
            z, conf = mvs.plane_sweep(
                views, ref, K_c, near=self.near, far=self.far, n_depths=self.n_depths,
                patch=self.patch, aggregate=self.aggregate, cost_mode="census",
                census_radius=self.census_radius, uniqueness=self.uniqueness,
                max_cost=self.max_cost)
        except Exception:
            return

        # Confident pixels (pre-densification) → metric cloud points. Gate to
        # high-confidence, non-far-band pixels (the far band's tiny disparity is
        # what sprays points past distant walls), then strip statistical outliers
        # (isolated triangulation flyers) so the cloud stays clean.
        far_cap = min(self.far * self.cloud_far_frac, self.cloud_max_depth)
        cloud_z = np.where((conf >= self.cloud_min_conf) & (z < far_cap), z, np.nan)
        cxyz, ccol = mvs.backproject_zdepth(cloud_z, rgb_c, K_c, R, center, stride=self.cloud_stride)
        cxyz, ccol = self._filter_outliers(cxyz, ccol)
        dense_z = mvs.densify(z, gf, near=self.near, far=self.far)
        # Upsample the metric z-depth back to the full frame (z-depth is
        # resolution-invariant in value) for the tile + segmentation grounding.
        euclid = mvs.zdepth_to_euclidean(self._upsample(dense_z, w, h), K)
        with self._lock:
            self._frame_idx += 1
            if cxyz.shape[0]:
                self._cloud.add(cxyz, ccol)                       # full fidelity → disk (export)
                self._voxels.add(cxyz, ccol, self._frame_idx)     # fused → stable live cloud
            # Periodically retire stale, weakly-seen voxels (moving-object trails)
            # so the live cloud stays stable as the sim runs for dozens of frames.
            if self._frame_idx % 5 == 0:
                self._voxels.decay(self._frame_idx)
            self._depth_map[drone_id] = euclid
            self._conf[drone_id] = self._upsample(conf, w, h)
            colorized = (self.far - np.nan_to_num(euclid, nan=self.far)) / (self.far - self.near)
            self._depth_jpeg[drone_id] = _encode_jpeg(_colorize(np.clip(colorized, 0, 1)))

    def _downscale(self, gray: np.ndarray, rgb: np.ndarray, K: np.ndarray):
        import cv2
        h, w = gray.shape
        if w <= self.depth_width:
            return gray, rgb, K, 1.0
        s = self.depth_width / float(w)
        wc, hc = int(round(w * s)), int(round(h * s))
        g = cv2.resize(gray, (wc, hc), interpolation=cv2.INTER_AREA)
        r = cv2.resize(rgb, (wc, hc), interpolation=cv2.INTER_AREA)
        Kc = K.copy()
        Kc[0, 0] *= s; Kc[1, 1] *= s; Kc[0, 2] *= s; Kc[1, 2] *= s
        return g, r, Kc, s

    def _upsample(self, depth: np.ndarray, w: int, h: int) -> np.ndarray:
        import cv2
        if depth.shape == (h, w):
            return depth
        return cv2.resize(depth, (w, h), interpolation=cv2.INTER_LINEAR)

    def _filter_outliers(self, xyz: np.ndarray, colors: np.ndarray):
        """Statistical outlier removal: drop points whose mean distance to their
        k nearest neighbours exceeds the batch mean by ``outlier_std`` sigma."""
        if xyz.shape[0] < self.outlier_k + 1:
            return xyz, colors
        try:
            from scipy.spatial import cKDTree
        except Exception:
            return xyz, colors
        tree = cKDTree(xyz)
        d, _ = tree.query(xyz, k=self.outlier_k + 1)
        mean_d = d[:, 1:].mean(axis=1)            # exclude self (distance 0)
        thresh = mean_d.mean() + self.outlier_std * mean_d.std()
        keep = mean_d <= thresh
        return xyz[keep], colors[keep]

    # -- accessors ---------------------------------------------------------

    def latest_depth_jpeg(self, drone_id: str) -> bytes | None:
        with self._lock:
            return self._depth_jpeg.get(drone_id)

    def latest_depth_map(self, drone_id: str) -> np.ndarray | None:
        with self._lock:
            return self._depth_map.get(drone_id)

    def latest_confidence(self, drone_id: str) -> np.ndarray | None:
        with self._lock:
            return self._conf.get(drone_id)

    def cloud_snapshot(self, max_points: int = 2500) -> list[list[float]]:
        # The live UI cloud is the FUSED voxel cloud — stable + denoised across
        # dozens of frames, not the raw ever-growing append stream.
        with self._lock:
            return self._voxels.snapshot(max_points)

    def cloud_arrays(self):
        with self._lock:
            return self._voxels.arrays()      # fused display cloud (splat seeding)

    def cloud_full_arrays(self):
        with self._lock:
            return self._cloud.all_arrays()   # full-fidelity stream (export)

    def reset(self, stream_path=None) -> None:
        with self._lock:
            self._windows.clear()
            self._depth_jpeg.clear()
            self._depth_map.clear()
            self._conf.clear()
            self._voxels.clear()
            self._frame_idx = 0
            self._cloud.close()
            self._cloud = _Cloud(stream_path)
