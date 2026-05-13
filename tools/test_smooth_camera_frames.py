#!/usr/bin/env python3
from __future__ import annotations

import sys
import unittest
from pathlib import Path

from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parent))

from smooth_camera_frames import block_artifact_filter


class BlockArtifactFilterTest(unittest.TestCase):
    def test_replaces_corrupted_block_with_temporal_median(self) -> None:
        previous = Image.new("RGB", (8, 8), (10, 20, 30))
        current = previous.copy()
        next_frame = Image.new("RGB", (8, 8), (12, 21, 33))
        current.paste((250, 0, 200), (0, 0, 4, 4))

        filtered, replaced_pct, detected_count = block_artifact_filter(
            previous,
            current,
            next_frame,
            block_size=4,
            outlier_threshold=50,
            stable_threshold=5,
            min_changed_pct=90.0,
            min_mean_diff=50.0,
        )

        self.assertEqual(detected_count, 1)
        self.assertAlmostEqual(replaced_pct, 25.0)
        self.assertEqual(filtered.getpixel((1, 1)), (12, 20, 33))
        self.assertEqual(filtered.getpixel((5, 5)), (10, 20, 30))

    def test_ignores_block_when_neighbors_disagree(self) -> None:
        previous = Image.new("RGB", (8, 8), (10, 20, 30))
        current = previous.copy()
        next_frame = Image.new("RGB", (8, 8), (200, 20, 30))
        current.paste((250, 0, 200), (0, 0, 4, 4))

        filtered, replaced_pct, detected_count = block_artifact_filter(
            previous,
            current,
            next_frame,
            block_size=4,
            outlier_threshold=50,
            stable_threshold=5,
            min_changed_pct=90.0,
            min_mean_diff=50.0,
        )

        self.assertEqual(detected_count, 0)
        self.assertEqual(replaced_pct, 0.0)
        self.assertEqual(filtered.getpixel((1, 1)), (250, 0, 200))


if __name__ == "__main__":
    unittest.main()
