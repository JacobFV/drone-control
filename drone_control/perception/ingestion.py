"""
Bridge a live camera ``FrameSource`` into the runtime's frame consumers.

A :class:`FrameIngestor` runs one background thread per drone that pulls JPEG
frames from any ``drone_control.live_video.FrameSource`` (the live RTP/JPEG drone
camera, or a ``DirectoryFrameSource`` for replay/testing) and hands each frame —
together with the drone's current pose — to a sink callback. The manager's sink
is :meth:`RuntimeManager.ingest_frame`, which publishes the bytes to the shared
``LiveFrameRegistry`` (for the batched diffusion VLA) and ingests them into the
live splat world model. One capture, both consumers.
"""

from __future__ import annotations

import threading
from typing import Any, Callable, Protocol


class FrameLike(Protocol):
    data: bytes
    metadata: dict[str, Any]


class FrameSourceLike(Protocol):
    def start(self) -> None: ...
    def stop(self) -> None: ...
    def read(self, timeout: float | None = None) -> FrameLike | None: ...


Sink = Callable[[str, bytes, dict[str, Any] | None], None]
PoseProvider = Callable[[], dict[str, Any] | None]


class FrameIngestor:
    def __init__(
        self,
        drone_id: str,
        source: FrameSourceLike,
        sink: Sink,
        *,
        pose_provider: PoseProvider | None = None,
        read_timeout: float = 0.5,
    ) -> None:
        self.drone_id = drone_id
        self.source = source
        self.sink = sink
        self.pose_provider = pose_provider
        self.read_timeout = read_timeout
        self.frames = 0
        self.last_error: str | None = None
        self._running = False
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self._running:
            return
        self.source.start()
        self._running = True
        self._thread = threading.Thread(target=self._loop, name=f"ingest-{self.drone_id}", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        try:
            self.source.stop()
        except Exception:
            pass
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2.0)
        self._thread = None

    def _loop(self) -> None:
        while self._running:
            try:
                frame = self.source.read(timeout=self.read_timeout)
            except Exception as exc:
                self.last_error = str(exc)
                continue
            if frame is None or not getattr(frame, "data", None):
                continue
            pose = None
            if self.pose_provider is not None:
                try:
                    pose = self.pose_provider()
                except Exception:
                    pose = None
            try:
                self.sink(self.drone_id, frame.data, pose)
                self.frames += 1
            except Exception as exc:
                self.last_error = str(exc)

    def status(self) -> dict[str, Any]:
        return {
            "droneId": self.drone_id,
            "running": self._running,
            "frames": self.frames,
            "lastError": self.last_error,
        }
