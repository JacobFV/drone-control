from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol

from .state import FrameMetadata


class FrameMetadataSource(Protocol):
    def latest(self) -> FrameMetadata | None:
        ...


@dataclass(slots=True)
class StaticFrameSource:
    frame: FrameMetadata | None = None

    def latest(self) -> FrameMetadata | None:
        return self.frame


class DirectoryMetadataSource:
    def __init__(self, path: Path | str, *, fps: float = 12.0) -> None:
        self.path = Path(path)
        self.fps = max(1.0, float(fps))
        self._started_at = time.monotonic()
        self._frames = sorted(self.path.glob("*.jpg"))

    def latest(self) -> FrameMetadata | None:
        if not self._frames:
            return None
        elapsed = max(0.0, time.monotonic() - self._started_at)
        index = min(len(self._frames) - 1, int(elapsed * self.fps))
        return FrameMetadata(index=index, timestamp=time.time(), source=str(self._frames[index]))


class LiveFrameAdapter:
    def __init__(self, source: object) -> None:
        self.source = source

    def latest(self) -> FrameMetadata | None:
        latest = getattr(self.source, "latest", None)
        if callable(latest):
            value = latest()
            if isinstance(value, FrameMetadata):
                return value
        return None

