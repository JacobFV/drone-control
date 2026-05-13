from __future__ import annotations

import enum
import json
import math
import queue
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

try:  # cv2 is heavy; estimator is inert without it
    import cv2
except ImportError:  # pragma: no cover - exercised by deployment shape
    cv2 = None  # type: ignore[assignment]

from drone_control.intrinsics import CameraIntrinsics, estimate_intrinsics


class TrackingState(str, enum.Enum):
    NO_ESTIMATOR = "no_estimator"
    INITIALIZING = "initializing"
    AWAITING_PARALLAX = "awaiting_parallax"
    TRACKING = "tracking"
    DEGRADED = "degraded"
    LOST = "lost"


@dataclass(slots=True)
class PoseSample:
    frame_index: int
    t: float
    x: float
    y: float
    z: float
    qw: float
    qx: float
    qy: float
    qz: float
    confidence: float
    state: str

    def as_dict(self) -> dict[str, object]:
        return {
            "frameIndex": self.frame_index,
            "t": self.t,
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "qw": self.qw,
            "qx": self.qx,
            "qy": self.qy,
            "qz": self.qz,
            "confidence": self.confidence,
            "state": self.state,
        }


@dataclass(slots=True)
class EstimatorStatus:
    state: str
    frames_seen: int
    frames_processed: int
    keyframes: int
    confidence: float
    scale_locked: bool
    intrinsics_source: str
    fps: float
    last_error: str | None

    def as_dict(self) -> dict[str, object]:
        return {
            "state": self.state,
            "framesSeen": self.frames_seen,
            "framesProcessed": self.frames_processed,
            "keyframes": self.keyframes,
            "confidence": self.confidence,
            "scaleLocked": self.scale_locked,
            "intrinsicsSource": self.intrinsics_source,
            "fps": self.fps,
            "lastError": self.last_error,
        }


def _quat_from_R(R: np.ndarray) -> tuple[float, float, float, float]:
    trace = float(R[0, 0] + R[1, 1] + R[2, 2])
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return (float(w), float(x), float(y), float(z))


class PoseEstimator:
    """Phase-0 monocular visual odometry.

    Pairwise ORB features → essential matrix → recoverPose, chained relative
    motion. Translation is unit-norm per step (scale unobservable from a single
    camera with no altitude source), so the trajectory has correct shape but
    arbitrary scale. State machine surfaces tracking health.
    """

    def __init__(
        self,
        *,
        out_path: Path,
        intrinsics: CameraIntrinsics | None = None,
        max_features: int = 1500,
        min_matches: int = 30,
    ) -> None:
        self.out_path = out_path
        self.out_path.parent.mkdir(parents=True, exist_ok=True)
        self.intrinsics = intrinsics
        self.max_features = max_features
        self.min_matches = min_matches

        self._lock = threading.RLock()
        self._poses: list[PoseSample] = []
        self._pose_world = np.eye(4, dtype=np.float64)
        self._prev_kp = None
        self._prev_des = None
        self._orb = None
        self._matcher = None
        self._last_t = time.monotonic()
        self._fps_alpha = 0.2

        self._status = EstimatorStatus(
            state=TrackingState.INITIALIZING.value if cv2 is not None else TrackingState.NO_ESTIMATOR.value,
            frames_seen=0,
            frames_processed=0,
            keyframes=0,
            confidence=0.0,
            scale_locked=False,
            intrinsics_source=intrinsics.source if intrinsics else "missing",
            fps=0.0,
            last_error=None,
        )

        if cv2 is not None:
            self._orb = cv2.ORB_create(self.max_features, scaleFactor=1.2, nlevels=8, fastThreshold=12)
            self._matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
            # Truncate any prior session file to avoid concatenated runs.
            self.out_path.write_text("")

    @property
    def available(self) -> bool:
        return cv2 is not None

    def status(self) -> EstimatorStatus:
        with self._lock:
            s = self._status
            return EstimatorStatus(
                state=s.state,
                frames_seen=s.frames_seen,
                frames_processed=s.frames_processed,
                keyframes=s.keyframes,
                confidence=s.confidence,
                scale_locked=s.scale_locked,
                intrinsics_source=s.intrinsics_source,
                fps=s.fps,
                last_error=s.last_error,
            )

    def poses(self, since_index: int = -1) -> list[PoseSample]:
        with self._lock:
            if since_index < 0:
                return list(self._poses)
            return [p for p in self._poses if p.frame_index > since_index]

    def process(self, frame_index: int, t: float, jpeg: bytes) -> PoseSample | None:
        if cv2 is None:
            return None
        with self._lock:
            self._status.frames_seen += 1
        try:
            return self._process_frame(frame_index, t, jpeg)
        except BaseException as exc:
            with self._lock:
                self._status.last_error = f"{type(exc).__name__}: {exc}"
                self._status.state = TrackingState.LOST.value
            return None

    def _process_frame(self, frame_index: int, t: float, jpeg: bytes) -> PoseSample | None:
        arr = np.frombuffer(jpeg, dtype=np.uint8)
        gray = cv2.imdecode(arr, cv2.IMREAD_GRAYSCALE)
        if gray is None:
            with self._lock:
                self._status.last_error = "frame decode failed"
            return None

        h, w = gray.shape[:2]
        if self.intrinsics is None:
            self.intrinsics = estimate_intrinsics(w, h)
            with self._lock:
                self._status.intrinsics_source = self.intrinsics.source

        kp, des = self._orb.detectAndCompute(gray, None)
        if des is None or len(kp) < self.min_matches:
            return self._record(frame_index, t, confidence=0.0, state=TrackingState.DEGRADED, advance=False, prev=(kp, des))

        if self._prev_des is None:
            return self._record(frame_index, t, confidence=0.0, state=TrackingState.AWAITING_PARALLAX, advance=False, prev=(kp, des))

        matches = self._matcher.match(self._prev_des, des)
        if len(matches) < self.min_matches:
            return self._record(frame_index, t, confidence=0.0, state=TrackingState.DEGRADED, advance=False, prev=(kp, des))

        matches = sorted(matches, key=lambda m: m.distance)[: max(self.min_matches * 4, 200)]
        pts_prev = np.float32([self._prev_kp[m.queryIdx].pt for m in matches])
        pts_curr = np.float32([kp[m.trainIdx].pt for m in matches])

        K = self.intrinsics.K()
        E, mask = cv2.findEssentialMat(
            pts_curr, pts_prev, K, method=cv2.RANSAC, prob=0.999, threshold=1.0
        )
        if E is None or mask is None:
            return self._record(frame_index, t, confidence=0.0, state=TrackingState.LOST, advance=False, prev=(kp, des))

        inliers = int(mask.sum())
        confidence = inliers / max(1, len(matches))
        if inliers < self.min_matches:
            return self._record(frame_index, t, confidence=confidence, state=TrackingState.AWAITING_PARALLAX, advance=False, prev=(kp, des))

        _, R, tvec, _ = cv2.recoverPose(E, pts_curr, pts_prev, K, mask=mask)

        T_rel = np.eye(4, dtype=np.float64)
        T_rel[:3, :3] = R
        T_rel[:3, 3] = tvec.flatten()
        # recoverPose gives the pose of the previous frame expressed in the
        # current frame; invert to step the world-from-camera transform.
        self._pose_world = self._pose_world @ np.linalg.inv(T_rel)

        state = TrackingState.TRACKING if confidence >= 0.5 else TrackingState.DEGRADED
        sample = self._record(frame_index, t, confidence=confidence, state=state, advance=True, prev=(kp, des))
        with self._lock:
            self._status.keyframes += 1
        return sample

    def _record(
        self,
        frame_index: int,
        t: float,
        *,
        confidence: float,
        state: TrackingState,
        advance: bool,
        prev: tuple[object, object],
    ) -> PoseSample:
        self._prev_kp, self._prev_des = prev
        T = self._pose_world
        position = T[:3, 3]
        qw, qx, qy, qz = _quat_from_R(T[:3, :3])
        sample = PoseSample(
            frame_index=frame_index,
            t=t,
            x=float(position[0]),
            y=float(position[1]),
            z=float(position[2]),
            qw=qw,
            qx=qx,
            qy=qy,
            qz=qz,
            confidence=float(confidence),
            state=state.value,
        )
        now = time.monotonic()
        dt = now - self._last_t
        self._last_t = now
        with self._lock:
            self._poses.append(sample)
            self._status.state = state.value
            self._status.confidence = float(confidence)
            if advance:
                self._status.frames_processed += 1
            if dt > 0:
                instant = 1.0 / dt
                self._status.fps = (1.0 - self._fps_alpha) * self._status.fps + self._fps_alpha * instant
            try:
                with self.out_path.open("a") as handle:
                    handle.write(json.dumps(sample.as_dict()) + "\n")
            except OSError as exc:
                self._status.last_error = f"pose write failed: {exc}"
        return sample


class AsyncPoseEstimator:
    """Threaded wrapper that consumes frames off a bounded queue."""

    def __init__(self, *, out_path: Path, intrinsics: CameraIntrinsics | None = None, queue_size: int = 64) -> None:
        self.estimator = PoseEstimator(out_path=out_path, intrinsics=intrinsics)
        self._queue: queue.Queue[tuple[int, float, bytes] | None] = queue.Queue(maxsize=queue_size)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._dropped = 0

    @property
    def available(self) -> bool:
        return self.estimator.available

    def start(self) -> None:
        if not self.available or self._thread is not None:
            return
        self._thread = threading.Thread(target=self._run, name="pose-estimator", daemon=True)
        self._thread.start()

    def push(self, frame_index: int, t: float, jpeg: bytes) -> bool:
        if not self.available:
            return False
        try:
            self._queue.put_nowait((frame_index, t, jpeg))
            return True
        except queue.Full:
            self._dropped += 1
            return False

    def stop(self, timeout: float = 4.0) -> None:
        self._stop.set()
        try:
            self._queue.put_nowait(None)
        except queue.Full:
            pass
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=timeout)
        self._thread = None

    def status(self) -> EstimatorStatus:
        snapshot = self.estimator.status()
        if self._dropped:
            note = f"dropped {self._dropped} frame(s) at estimator queue"
            snapshot.last_error = note if snapshot.last_error is None else snapshot.last_error
        return snapshot

    def poses(self, since_index: int = -1) -> list[PoseSample]:
        return self.estimator.poses(since_index)

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                item = self._queue.get(timeout=0.5)
            except queue.Empty:
                continue
            if item is None:
                break
            frame_index, t, jpeg = item
            self.estimator.process(frame_index, t, jpeg)


def replay_directory(frame_dir: Path, out_path: Path, *, fps: float = 12.0, intrinsics: CameraIntrinsics | None = None) -> EstimatorStatus:
    """Synchronously process every JPEG in ``frame_dir``. Used for review-mode
    recompute on stored frame records."""
    if cv2 is None:
        raise RuntimeError("cv2 is required for pose estimation")
    estimator = PoseEstimator(out_path=out_path, intrinsics=intrinsics)
    frames = sorted(frame_dir.glob("*.jpg"))
    interval = 1.0 / max(1.0, fps)
    for index, frame in enumerate(frames):
        estimator.process(index, index * interval, frame.read_bytes())
    return estimator.status()


def load_pose_track(path: Path) -> list[dict[str, object]]:
    if not path.is_file():
        return []
    poses: list[dict[str, object]] = []
    with path.open() as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            try:
                poses.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return poses
