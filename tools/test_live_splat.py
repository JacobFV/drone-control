from __future__ import annotations

import io
import math
import tempfile
import time
import unittest
from pathlib import Path

from drone_control.perception import live_splat
from drone_control.perception.live_splat import LiveSplatConfig, LiveSplatEngine


def _synthetic_jpeg(seed: int) -> bytes:
    from PIL import Image
    import numpy as np

    rng = np.random.default_rng(seed)
    base = rng.integers(40, 60, size=(96, 128, 3), dtype=np.uint8)
    # Add a bright structured block so there is real signal to fit.
    base[20:60, 30:90, 0] = 220
    base[20:60, 30:90, 1] = 120
    buffer = io.BytesIO()
    Image.fromarray(base).save(buffer, format="JPEG", quality=85)
    return buffer.getvalue()


def _pose(x: float, y: float = 0.0, z: float = 0.0) -> dict:
    return {"translation": [x, y, z], "rotation_xyzw": [0.0, 0.0, 0.0, 1.0], "confidence": 0.6}


@unittest.skipUnless(live_splat.available(), f"gsplat/CUDA unavailable: {live_splat.unavailable_reason()}")
class LiveSplatTest(unittest.TestCase):
    def test_cross_drone_fusion_and_export(self) -> None:
        engine = LiveSplatEngine(LiveSplatConfig(image_max_size=96, init_stride=6, densify_interval=50))
        engine.start()
        try:
            # Two drones contribute frames from slightly different vantage points
            # into one shared gaussian set.
            accepted = 0
            for i in range(6):
                if engine.ingest("drone-a", _synthetic_jpeg(i), _pose(0.3 * i)):
                    accepted += 1
                if engine.ingest("drone-b", _synthetic_jpeg(100 + i), _pose(0.3 * i, y=1.0)):
                    accepted += 1
            self.assertGreater(accepted, 0)

            # Let the optimiser run a while.
            deadline = time.time() + 6.0
            first_loss = None
            while time.time() < deadline:
                snap = engine.snapshot()
                if snap["lastLoss"] is not None and first_loss is None:
                    first_loss = snap["lastLoss"]
                if snap["steps"] > 300:
                    break
                time.sleep(0.1)

            snap = engine.snapshot()
            self.assertGreater(snap["gaussians"], 0)
            self.assertGreaterEqual(len(snap["drones"]), 2)  # genuine cross-drone fusion
            self.assertGreater(snap["steps"], 0)
            self.assertIsNotNone(snap["lastLoss"])

            with tempfile.TemporaryDirectory() as tmp:
                out = engine.export_ply(Path(tmp) / "world.ply")
                self.assertTrue(out.is_file())
                self.assertGreater(out.stat().st_size, 0)
                self.assertTrue(out.read_bytes().startswith(b"ply"))
        finally:
            engine.stop()


@unittest.skipUnless(live_splat.available(), f"gsplat/CUDA unavailable: {live_splat.unavailable_reason()}")
class WorldModelManagerWiringTest(unittest.TestCase):
    def test_manager_world_model_lifecycle(self) -> None:
        from drone_control.runtime.manager import RuntimeManager

        manager = RuntimeManager()
        status = manager.start_world_model()
        self.assertTrue(status.get("available", True))
        try:
            for i in range(4):
                manager.ingest_frame("drone-a", _synthetic_jpeg(i), _pose(0.3 * i))
                manager.ingest_frame("drone-b", _synthetic_jpeg(100 + i), _pose(0.3 * i, y=1.0))
            time.sleep(2.0)
            status = manager.world_model_status()
            self.assertGreaterEqual(len(status["drones"]), 2)
            self.assertGreater(status["gaussians"], 0)
            with tempfile.TemporaryDirectory() as tmp:
                out = manager.export_world_model(Path(tmp) / "w.ply")
                self.assertIsNotNone(out)
                self.assertTrue(Path(out).is_file())
        finally:
            manager.stop_world_model()


if __name__ == "__main__":
    unittest.main()
