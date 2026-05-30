"""
Live multi-drone visual odometry — the drones' *estimated* world trajectory.

The station already renders each drone's **objective** trajectory (ground-truth
sim state, or the runtime's pose for real hardware). This module produces the
complementary view: where each drone *thinks* it is, computed purely from its
camera stream by the project's monocular VO (``drone_control.pose_estimator``).

  estimator input  ── camera frame + calibrated intrinsics only
  estimator output ── a scale-free world track (correct shape, arbitrary gauge:
                      it starts at the origin and translation is unit-norm per
                      step, because absolute scale is unobservable from one
                      monocular camera with no altitude/IMU source).

To draw the estimate against the truth we similarity-align (Umeyama: scale +
rotation + translation) the VO track onto the concurrent ground-truth samples
and report the post-alignment drift (APE). This mirrors how every VO/SLAM
trajectory is evaluated, and — crucially — the ground truth is used ONLY for the
alignment + drift overlay, exactly the same privilege the depth eval harness has
(``tools/depth_eval``). It is NEVER fed back into the estimator, so the estimator
itself stays environment-agnostic per the rule in ``session_service._perceive``.

For real environments there is no oracle, so the raw (origin-anchored) estimate
is shown unaligned with no drift number.
"""

from __future__ import annotations

import threading
from collections import deque
from pathlib import Path
from typing import Any

import numpy as np

from drone_control.intrinsics import CameraIntrinsics
from drone_control.pose_estimator import PoseEstimator, _quat_from_R, estimator_available


# Only these states correspond to an *advancing* VO step (a fresh relative
# motion was integrated); other samples sit at the previous pose and would pile
# up at the origin during init, biasing the similarity fit.
_ADVANCING = {"tracking", "degraded"}


def _umeyama_similarity(src: np.ndarray, dst: np.ndarray) -> tuple[float, np.ndarray, np.ndarray]:
    """Least-squares similarity mapping ``src`` onto ``dst`` (Umeyama 1991).

    Solves ``dst ≈ s · R · src + t`` for scale ``s``, rotation ``R`` and
    translation ``t`` over paired point sets ``src``/``dst`` ([N,3]).
    """
    n = src.shape[0]
    mu_s = src.mean(axis=0)
    mu_d = dst.mean(axis=0)
    sc = src - mu_s
    dc = dst - mu_d
    cov = (dc.T @ sc) / n
    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1.0
    R = U @ S @ Vt
    var_s = float((sc ** 2).sum() / n)
    if var_s < 1e-9:
        return 1.0, np.eye(3), mu_d - mu_s
    scale = float(np.trace(np.diag(D) @ S) / var_s)
    t = mu_d - scale * (R @ mu_s)
    return scale, R, t


class _Track:
    """One drone's live VO estimator plus the paired (estimate, truth) buffer."""

    __slots__ = ("estimator", "est", "gt", "last_fp", "frames")

    def __init__(self, estimator: PoseEstimator, history: int) -> None:
        self.estimator = estimator
        self.est: deque[tuple[float, float, float, float, float, float, float, int]] = deque(maxlen=history)
        self.gt: deque[np.ndarray | None] = deque(maxlen=history)
        self.last_fp: tuple[int, bytes] | None = None
        self.frames = 0


class LiveVisualOdometry:
    """Per-drone monocular VO over the live session, with a GT-aligned overlay."""

    def __init__(self, work_root: Path, *, history: int = 600) -> None:
        self._work_root = work_root
        self._history = history
        self._lock = threading.RLock()
        self._tracks: dict[str, _Track] = {}
        self._session_id: str | None = None

    # -- lifecycle ---------------------------------------------------------

    @property
    def available(self) -> bool:
        return estimator_available()

    @property
    def reason(self) -> str | None:
        return None if estimator_available() else "opencv (cv2) not installed"

    def reset(self, session_id: str | None) -> None:
        with self._lock:
            self._tracks.clear()
            self._session_id = session_id

    # -- ingestion ---------------------------------------------------------

    def process(
        self,
        drone_id: str,
        jpeg: bytes,
        intrinsics: dict[str, Any] | None,
        gt_center: np.ndarray | None,
    ) -> None:
        """Feed one frame. ``gt_center`` is the concurrent ground-truth position
        (sim only) used solely for the alignment overlay — never for estimation."""
        if not jpeg or not estimator_available():
            return
        track = self._track(drone_id, intrinsics)
        # Skip byte-identical repeats (the camera frame hasn't advanced) so we
        # don't waste ORB on zero-parallax pairs.
        fp = (len(jpeg), jpeg[:32])
        if track.last_fp == fp:
            return
        track.last_fp = fp
        idx = track.frames
        track.frames += 1
        sample = track.estimator.process(idx, float(idx), jpeg)
        if sample is None or sample.state not in _ADVANCING:
            return
        with self._lock:
            track.est.append(
                (sample.x, sample.y, sample.z, sample.qw, sample.qx, sample.qy, sample.qz, idx)
            )
            track.gt.append(None if gt_center is None else np.asarray(gt_center, dtype=float))

    def _track(self, drone_id: str, intrinsics: dict[str, Any] | None) -> _Track:
        with self._lock:
            track = self._tracks.get(drone_id)
            if track is None:
                out = self._work_root / "vo" / (self._session_id or "live") / f"{_safe(drone_id)}.jsonl"
                estimator = PoseEstimator(
                    out_path=out,
                    intrinsics=_intrinsics(intrinsics),
                    history_limit=self._history,
                )
                track = _Track(estimator, self._history)
                self._tracks[drone_id] = track
            elif track.estimator.intrinsics is None and intrinsics is not None:
                track.estimator.intrinsics = _intrinsics(intrinsics)
            return track

    # -- query -------------------------------------------------------------

    def estimate(self, drone_id: str) -> dict[str, Any]:
        """The drone's estimated trajectory, similarity-aligned to ground truth
        when available, plus drift metrics. Safe to call every status tick."""
        with self._lock:
            track = self._tracks.get(drone_id)
            if track is None or not track.est:
                return self._empty()
            est = np.array([(e[0], e[1], e[2]) for e in track.est], dtype=float)
            quats = [(e[3], e[4], e[5], e[6]) for e in track.est]
            idxs = [e[7] for e in track.est]
            gt = list(track.gt)
            status = track.estimator.status()

        # Pair estimate samples with concurrent ground truth (sim only).
        pairs = [(est[i], gt[i]) for i in range(len(est)) if gt[i] is not None]
        scale, R, t, aligned = 1.0, np.eye(3), np.zeros(3), False
        if len(pairs) >= 4:
            src = np.array([p[0] for p in pairs])
            dst = np.array([p[1] for p in pairs])
            if float(dst.var(axis=0).sum()) > 1e-6 and float(src.var(axis=0).sum()) > 1e-9:
                scale, R, t = _umeyama_similarity(src, dst)
                aligned = True

        poses: list[dict[str, Any]] = []
        for (px, py, pz), (qw, qx, qy, qz), fi in zip(est, quats, idxs):
            p = scale * (R @ np.array([px, py, pz])) + t if aligned else np.array([px, py, pz])
            qw2, qx2, qy2, qz2 = _rotate_quat(R, qw, qx, qy, qz) if aligned else (qw, qx, qy, qz)
            poses.append({
                "x": round(float(p[0]), 4), "y": round(float(p[1]), 4), "z": round(float(p[2]), 4),
                "qw": round(float(qw2), 5), "qx": round(float(qx2), 5),
                "qy": round(float(qy2), 5), "qz": round(float(qz2), 5),
                "frameIndex": int(fi),
            })

        drift_rmse: float | None = None
        drift_final: float | None = None
        if aligned and pairs:
            errs = np.array([
                np.linalg.norm(scale * (R @ p[0]) + t - p[1]) for p in pairs
            ])
            drift_rmse = round(float(np.sqrt((errs ** 2).mean())), 3)
            drift_final = round(float(errs[-1]), 3)

        return {
            "poses": poses,
            "aligned": aligned,
            "scale": round(float(scale), 4) if aligned else None,
            "driftRmse": drift_rmse,
            "driftFinal": drift_final,
            "state": status.state,
            "confidence": round(float(status.confidence), 3),
            "keyframes": status.keyframes,
        }

    def _empty(self) -> dict[str, Any]:
        return {
            "poses": [], "aligned": False, "scale": None,
            "driftRmse": None, "driftFinal": None,
            "state": "initializing" if estimator_available() else "no_estimator",
            "confidence": 0.0, "keyframes": 0,
        }


def _intrinsics(intr: dict[str, Any] | None) -> CameraIntrinsics | None:
    """Build calibrated intrinsics from a camera-pose ``intrinsics`` block.

    These are pure calibration (the lens focal/centre) — the same values the
    depth front-end already consumes — not scene geometry.
    """
    if not intr or not all(k in intr for k in ("fx", "fy", "cx", "cy")):
        return None
    return CameraIntrinsics(
        fx=float(intr["fx"]), fy=float(intr["fy"]),
        cx=float(intr["cx"]), cy=float(intr["cy"]),
        width=int(intr.get("width") or round(float(intr["cx"]) * 2)),
        height=int(intr.get("height") or round(float(intr["cy"]) * 2)),
        distortion=(0.0, 0.0, 0.0, 0.0, 0.0),
        source="calibrated",
    )


def _rotate_quat(R: np.ndarray, qw: float, qx: float, qy: float, qz: float) -> tuple[float, float, float, float]:
    """Pre-rotate a quaternion's orientation by the alignment rotation ``R``."""
    n = (qw * qw + qx * qx + qy * qy + qz * qz) ** 0.5 or 1.0
    qw, qx, qy, qz = qw / n, qx / n, qy / n, qz / n
    Rq = np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ])
    return _quat_from_R(R @ Rq)


def _safe(name: str) -> str:
    return "".join(c if c.isalnum() or c in "-_" else "_" for c in name)
