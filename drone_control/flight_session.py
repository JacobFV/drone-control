from __future__ import annotations

import threading
import time
import uuid
from dataclasses import dataclass
from pathlib import Path

from drone_control.intrinsics import CameraIntrinsics
from drone_control.live_video import FrameSource
from drone_control.pose_estimator import AsyncPoseEstimator


@dataclass(slots=True)
class FlightSessionStatus:
    flight_id: str
    session_id: str
    running: bool
    source: str
    frame_dir: str
    frames: int
    bytes: int
    started_at: float
    stopped_at: float | None = None
    error: str | None = None
    record_id: str | None = None
    pose_record_id: str | None = None
    pose_path: str | None = None

    @property
    def duration_seconds(self) -> float:
        end = self.stopped_at if self.stopped_at is not None else time.time()
        return max(0.0, end - self.started_at)

    def as_dict(self) -> dict[str, object]:
        return {
            "flightId": self.flight_id,
            "sessionId": self.session_id,
            "running": self.running,
            "source": self.source,
            "frameDir": self.frame_dir,
            "frames": self.frames,
            "bytes": self.bytes,
            "durationSeconds": self.duration_seconds,
            "error": self.error,
            "recordId": self.record_id,
            "poseRecordId": self.pose_record_id,
            "posePath": self.pose_path,
        }


class FlightSession:
    def __init__(
        self,
        *,
        flight_id: str,
        source_name: str,
        frame_source: FrameSource,
        work_root: Path,
        read_timeout: float = 0.5,
        max_frames: int | None = None,
        intrinsics: CameraIntrinsics | None = None,
        enable_pose_estimation: bool = True,
    ) -> None:
        self.flight_id = flight_id
        self.source_name = source_name
        self.frame_source = frame_source
        self.read_timeout = read_timeout
        self.max_frames = max_frames
        self.session_id = f"session-{uuid.uuid4().hex[:12]}"
        self.session_root = work_root / self.session_id
        self.frame_dir = self.session_root / "frames"
        self.frame_dir.mkdir(parents=True, exist_ok=True)
        self.pose_path = self.session_root / "pose.jsonl"
        self.estimator: AsyncPoseEstimator | None = None
        if enable_pose_estimation:
            self.estimator = AsyncPoseEstimator(out_path=self.pose_path, intrinsics=intrinsics)
        self.status = FlightSessionStatus(
            flight_id=flight_id,
            session_id=self.session_id,
            running=False,
            source=source_name,
            frame_dir=str(self.frame_dir),
            frames=0,
            bytes=0,
            started_at=time.time(),
            pose_path=str(self.pose_path) if enable_pose_estimation else None,
        )
        self._stop = threading.Event()
        self._lock = threading.RLock()
        self._thread: threading.Thread | None = None
        self._monotonic_start = time.monotonic()

    def start(self) -> None:
        with self._lock:
            if self._thread is not None:
                return
            self.status.running = True
            self._thread = threading.Thread(target=self._run, name=f"flight-session-{self.session_id}", daemon=True)
            self._thread.start()

    def stop(self, timeout: float = 3.0) -> FlightSessionStatus:
        self._stop.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=timeout)
        with self._lock:
            if self.status.running:
                self.status.running = False
                self.status.stopped_at = time.time()
            return self.snapshot()

    def snapshot(self) -> FlightSessionStatus:
        with self._lock:
            return FlightSessionStatus(
                flight_id=self.status.flight_id,
                session_id=self.status.session_id,
                running=self.status.running,
                source=self.status.source,
                frame_dir=self.status.frame_dir,
                frames=self.status.frames,
                bytes=self.status.bytes,
                started_at=self.status.started_at,
                stopped_at=self.status.stopped_at,
                error=self.status.error,
                record_id=self.status.record_id,
                pose_record_id=self.status.pose_record_id,
                pose_path=self.status.pose_path,
            )

    def set_record_id(self, record_id: str) -> None:
        with self._lock:
            self.status.record_id = record_id

    def set_pose_record_id(self, record_id: str) -> None:
        with self._lock:
            self.status.pose_record_id = record_id

    def estimator_status(self) -> dict[str, object] | None:
        if self.estimator is None:
            return None
        return self.estimator.status().as_dict()

    def estimator_poses(self, since_index: int = -1) -> list[dict[str, object]]:
        if self.estimator is None:
            return []
        return [p.as_dict() for p in self.estimator.poses(since_index)]

    def _run(self) -> None:
        try:
            self.frame_source.start()
            if self.estimator is not None:
                self.estimator.start()
            while not self._stop.is_set():
                if self.max_frames is not None and self.status.frames >= self.max_frames:
                    break
                frame = self.frame_source.read(timeout=self.read_timeout)
                if frame is None:
                    continue
                with self._lock:
                    index = self.status.frames
                path = self.frame_dir / f"frame_{index:06d}.jpg"
                path.write_bytes(frame.data)
                with self._lock:
                    self.status.frames += 1
                    self.status.bytes += len(frame.data)
                if self.estimator is not None:
                    self.estimator.push(index, time.monotonic() - self._monotonic_start, frame.data)
        except BaseException as exc:
            with self._lock:
                self.status.error = str(exc)
        finally:
            if self.estimator is not None:
                self.estimator.stop()
            self.frame_source.stop()
            with self._lock:
                self.status.running = False
                self.status.stopped_at = time.time()


class FlightSessionManager:
    def __init__(self, work_root: Path) -> None:
        self.work_root = work_root
        self.work_root.mkdir(parents=True, exist_ok=True)
        self._lock = threading.RLock()
        self._sessions: dict[str, FlightSession] = {}

    def start(self, session: FlightSession) -> FlightSessionStatus:
        with self._lock:
            existing = self._sessions.get(session.flight_id)
            if existing and existing.snapshot().running:
                raise RuntimeError("flight already has an active session")
            self._sessions[session.flight_id] = session
            session.start()
            return session.snapshot()

    def get(self, flight_id: str) -> FlightSession | None:
        with self._lock:
            return self._sessions.get(flight_id)

    def status(self, flight_id: str) -> FlightSessionStatus | None:
        session = self.get(flight_id)
        return session.snapshot() if session else None

    def stop(self, flight_id: str) -> FlightSessionStatus | None:
        session = self.get(flight_id)
        if session is None:
            return None
        return session.stop()

    def stop_all(self) -> None:
        with self._lock:
            sessions = list(self._sessions.values())
        for session in sessions:
            session.stop()
