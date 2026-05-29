from __future__ import annotations

import io
import unittest

import numpy as np

from drone_control.sim.render import CameraConfig, CameraRenderer


def _decode(jpeg: bytes) -> np.ndarray:
    from PIL import Image

    with Image.open(io.BytesIO(jpeg)) as im:
        return np.asarray(im.convert("RGB"))


class RenderTest(unittest.TestCase):
    def setUp(self) -> None:
        self.r = CameraRenderer(CameraConfig(width=128, height=96))

    def test_produces_valid_jpeg(self) -> None:
        pos = np.array([[0.0, 0.0, 2.0]])
        quat = np.array([[1.0, 0.0, 0.0, 0.0]])  # level
        goal = np.array([[6.0, 0.0, 2.0]])
        frames = self.r.render(pos, quat, goal)
        self.assertEqual(len(frames), 1)
        self.assertTrue(frames[0].startswith(b"\xff\xd8"))  # JPEG SOI
        img = _decode(frames[0])
        self.assertEqual(img.shape, (96, 128, 3))

    def test_goal_direction_signal(self) -> None:
        # A level drone with the goal to the right vs left should put the magenta
        # marker on opposite sides of the image -> real, learnable signal.
        pos = np.array([[0.0, 0.0, 2.0]])
        quat = np.array([[1.0, 0.0, 0.0, 0.0]])
        right_goal = self.r.render(pos, quat, np.array([[6.0, 3.0, 2.0]]))[0]
        left_goal = self.r.render(pos, quat, np.array([[6.0, -3.0, 2.0]]))[0]
        self.assertNotEqual(right_goal, left_goal)

        ir = _decode(right_goal).astype(int)
        il = _decode(left_goal).astype(int)

        def magenta_centroid_x(img: np.ndarray) -> float:
            mask = (img[:, :, 0] > 150) & (img[:, :, 2] > 150) & (img[:, :, 1] < 120)
            xs = np.where(mask)[1]
            return float(xs.mean()) if xs.size else 64.0

        # right (+y == camera right) marker should be at larger x than left goal.
        self.assertGreater(magenta_centroid_x(ir), magenta_centroid_x(il))

    def test_batch_render(self) -> None:
        k = 6
        pos = np.random.default_rng(0).normal(size=(k, 3))
        pos[:, 2] = 3.0
        quat = np.tile([1.0, 0.0, 0.0, 0.0], (k, 1))
        goals = np.random.default_rng(1).normal(size=(k, 3))
        frames = self.r.render(pos, quat, goals)
        self.assertEqual(len(frames), k)
        for f in frames:
            self.assertTrue(f.startswith(b"\xff\xd8"))


if __name__ == "__main__":
    unittest.main()
