from __future__ import annotations

import threading
import time
from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class FrameBytes:
    """A single decoded-on-demand JPEG frame for a drone."""

    drone_id: str
    jpeg: bytes
    timestamp: float
    width: int | None = None
    height: int | None = None


class LiveFrameRegistry:
    """
    Thread-safe map of ``drone_id -> latest JPEG bytes``.

    Each drone's live video source publishes the most recent JPEG here; the
    batched VLA hub and the live splat engine both read from it so a frame is
    only decoded once downstream. In dry-run / no-camera mode no frames are ever
    published and :meth:`latest` returns ``None``, which callers treat as "feed a
    blank image" so the control loop still runs.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._frames: dict[str, FrameBytes] = {}

    def publish(
        self,
        drone_id: str,
        jpeg: bytes,
        *,
        timestamp: float | None = None,
        width: int | None = None,
        height: int | None = None,
    ) -> None:
        if not jpeg:
            return
        frame = FrameBytes(
            drone_id=drone_id,
            jpeg=jpeg,
            timestamp=time.time() if timestamp is None else timestamp,
            width=width,
            height=height,
        )
        with self._lock:
            self._frames[drone_id] = frame

    def latest(self, drone_id: str, *, max_age_seconds: float | None = None) -> FrameBytes | None:
        with self._lock:
            frame = self._frames.get(drone_id)
        if frame is None:
            return None
        if max_age_seconds is not None and (time.time() - frame.timestamp) > max_age_seconds:
            return None
        return frame

    def drone_ids(self) -> list[str]:
        with self._lock:
            return list(self._frames)

    def clear(self, drone_id: str | None = None) -> None:
        with self._lock:
            if drone_id is None:
                self._frames.clear()
            else:
                self._frames.pop(drone_id, None)
