from __future__ import annotations

import io
import tempfile
import time
import unittest
from pathlib import Path

import numpy as np
from PIL import Image

from drone_control.live_video import DirectoryFrameSource
from drone_control.perception.ingestion import FrameIngestor
from drone_control.runtime.manager import RuntimeManager


def _make_frames(directory: Path, count: int = 5) -> None:
    rng = np.random.default_rng(0)
    for i in range(count):
        arr = (rng.random((48, 64, 3)) * 255).astype("uint8")
        Image.fromarray(arr).save(directory / f"frame_{i:03d}.jpg", quality=85)


class FrameIngestorTest(unittest.TestCase):
    def test_pumps_frames_to_sink(self) -> None:
        received: list[tuple[str, int, dict | None]] = []

        def sink(drone_id: str, jpeg: bytes, pose: dict | None) -> None:
            received.append((drone_id, len(jpeg), pose))

        with tempfile.TemporaryDirectory() as tmp:
            _make_frames(Path(tmp))
            source = DirectoryFrameSource(tmp, fps=50.0)
            ingestor = FrameIngestor("drone-a", source, sink, pose_provider=lambda: {"translation": [1, 2, 3]})
            ingestor.start()
            deadline = time.time() + 2.0
            while time.time() < deadline and len(received) < 3:
                time.sleep(0.05)
            ingestor.stop()

        self.assertGreaterEqual(len(received), 3)
        self.assertEqual(received[0][0], "drone-a")
        self.assertGreater(received[0][1], 0)
        self.assertEqual(received[0][2], {"translation": [1, 2, 3]})

    def test_manager_attach_publishes_to_registry(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            _make_frames(Path(tmp))
            manager = RuntimeManager()
            source = DirectoryFrameSource(tmp, fps=50.0)
            try:
                manager.attach_frame_source("drone-x", source)
                deadline = time.time() + 2.0
                while time.time() < deadline and manager.frame_registry.latest("drone-x") is None:
                    time.sleep(0.05)
                frame = manager.frame_registry.latest("drone-x")
                self.assertIsNotNone(frame)
                self.assertGreater(len(frame.jpeg), 0)
                status = manager.ingestion_status()
                self.assertEqual(len(status), 1)
                self.assertGreater(status[0]["frames"], 0)
            finally:
                manager.stop_ingestion()


if __name__ == "__main__":
    unittest.main()
