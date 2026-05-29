from __future__ import annotations

import struct
import tempfile
import unittest
from pathlib import Path

import numpy as np

from drone_control.perception import cross_drone, live_splat
from drone_control.perception.cross_drone import (
    ColmapImage,
    ColmapModel,
    compute_drone_transforms,
    read_images_binary,
    read_points3d_binary,
    umeyama_similarity,
)
from drone_control.perception.live_splat import apply_similarity_to_pose


def _write_images_bin(path: Path, images: list[tuple[int, list[float], list[float], int, str]]) -> None:
    with path.open("wb") as handle:
        handle.write(struct.pack("<Q", len(images)))
        for image_id, qvec, tvec, camera_id, name in images:
            handle.write(struct.pack("<i7di", image_id, *qvec, *tvec, camera_id))
            handle.write(name.encode("utf-8") + b"\x00")
            handle.write(struct.pack("<Q", 0))  # zero 2D points


def _write_points3d_bin(path: Path, points: list[tuple[int, list[float], list[int]]]) -> None:
    with path.open("wb") as handle:
        handle.write(struct.pack("<Q", len(points)))
        for point_id, xyz, rgb in points:
            handle.write(struct.pack("<Q3d3Bd", point_id, *xyz, *rgb, 0.5))
            handle.write(struct.pack("<Q", 0))  # zero-length track


class ColmapParseTest(unittest.TestCase):
    def test_images_binary_roundtrip(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "images.bin"
            _write_images_bin(
                path,
                [
                    (1, [1.0, 0.0, 0.0, 0.0], [0.1, 0.2, 0.3], 1, "drone-a__000000.jpg"),
                    (2, [0.7071, 0.0, 0.7071, 0.0], [1.0, 0.0, -1.0], 1, "drone-b__000001.jpg"),
                ],
            )
            images = read_images_binary(path)
            self.assertEqual(set(images), {1, 2})
            self.assertEqual(images[2].name, "drone-b__000001.jpg")
            np.testing.assert_allclose(images[1].tvec, [0.1, 0.2, 0.3])
            # identity rotation -> camera centre == -t
            np.testing.assert_allclose(images[1].camera_center(), [-0.1, -0.2, -0.3], atol=1e-9)

    def test_points3d_binary_roundtrip(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "points3D.bin"
            _write_points3d_bin(path, [(1, [1.0, 2.0, 3.0], [255, 128, 0]), (2, [-1.0, 0.0, 5.0], [0, 0, 255])])
            xyz, rgb = read_points3d_binary(path)
            self.assertEqual(xyz.shape, (2, 3))
            np.testing.assert_allclose(xyz[0], [1.0, 2.0, 3.0])
            np.testing.assert_allclose(rgb[0], [1.0, 128 / 255.0, 0.0], atol=1e-6)


class UmeyamaTest(unittest.TestCase):
    def test_recovers_known_similarity(self) -> None:
        rng = np.random.default_rng(0)
        src = rng.normal(size=(12, 3))
        # Known transform: scale 2.5, a rotation about z by 30deg, translation.
        theta = np.deg2rad(30)
        R = np.array([[np.cos(theta), -np.sin(theta), 0], [np.sin(theta), np.cos(theta), 0], [0, 0, 1]])
        scale = 2.5
        t = np.array([1.0, -2.0, 0.5])
        dst = (scale * (R @ src.T).T) + t
        M = umeyama_similarity(src, dst)
        recovered = (M[:3, :3] @ src.T).T + M[:3, 3]
        np.testing.assert_allclose(recovered, dst, atol=1e-6)

    def test_compute_drone_transforms_per_drone(self) -> None:
        # Two drones, each with 3 registered images; VO centres differ from COLMAP
        # by a per-drone similarity. compute_drone_transforms should recover them.
        images = {}
        owner = {}
        vo_centers = {}
        rng = np.random.default_rng(1)
        for d, drone in enumerate(["a", "b"]):
            scale = 1.5 + d
            offset = np.array([d * 3.0, 0.0, 0.0])
            for i in range(4):
                name = f"{drone}__{i:06d}.jpg"
                vo = rng.normal(size=3)
                colmap_center = scale * vo + offset
                # build an image whose camera_center() == colmap_center (identity R -> t=-center)
                images[d * 10 + i] = ColmapImage(
                    image_id=d * 10 + i,
                    qvec=np.array([1.0, 0.0, 0.0, 0.0]),
                    tvec=-colmap_center,
                    camera_id=1,
                    name=name,
                )
                owner[name] = drone
                vo_centers[name] = vo
        model = ColmapModel(images=images)
        transforms = compute_drone_transforms(model, owner, vo_centers)
        self.assertEqual(set(transforms), {"a", "b"})
        # Verify drone a transform maps a VO centre to its COLMAP centre.
        Ma = np.asarray(transforms["a"])
        some = next(n for n in owner if owner[n] == "a")
        mapped = Ma[:3, :3] @ vo_centers[some] + Ma[:3, 3]
        np.testing.assert_allclose(mapped, images_by_name(model)[some].camera_center(), atol=1e-6)


def images_by_name(model: ColmapModel):
    return model.images_by_name()


class SimilarityPoseTest(unittest.TestCase):
    def test_identity_leaves_pose_unchanged(self) -> None:
        c2w = np.eye(4)
        c2w[:3, 3] = [1.0, 2.0, 3.0]
        out = apply_similarity_to_pose(np.eye(4), c2w)
        np.testing.assert_allclose(out, c2w, atol=1e-9)

    def test_scale_applies_to_centre_not_rotation(self) -> None:
        c2w = np.eye(4)
        c2w[:3, 3] = [1.0, 0.0, 0.0]
        sim = np.eye(4) * 1.0
        sim[:3, :3] = 2.0 * np.eye(3)  # pure scale 2
        sim[3, 3] = 1.0
        out = apply_similarity_to_pose(sim, c2w)
        # rotation stays orthonormal (identity), centre scaled by 2
        np.testing.assert_allclose(out[:3, :3], np.eye(3), atol=1e-9)
        np.testing.assert_allclose(out[:3, 3], [2.0, 0.0, 0.0], atol=1e-9)


@unittest.skipUnless(live_splat.available(), f"gsplat/CUDA unavailable: {live_splat.unavailable_reason()}")
class SeedFromPointsTest(unittest.TestCase):
    def test_seed_creates_gaussians_and_exports(self) -> None:
        engine = live_splat.LiveSplatEngine()
        rng = np.random.default_rng(2)
        xyz = rng.normal(size=(500, 3)).astype(np.float32)
        rgb = rng.random(size=(500, 3)).astype(np.float32)
        count = engine.seed_from_points(xyz, rgb)
        self.assertEqual(count, 500)
        snap = engine.snapshot()
        self.assertEqual(snap["gaussians"], 500)
        with tempfile.TemporaryDirectory() as tmp:
            out = engine.export_ply(Path(tmp) / "seed.ply")
            self.assertTrue(out.is_file())
            self.assertTrue(out.read_bytes().startswith(b"ply"))


@unittest.skipUnless(cross_drone.colmap_available(), "colmap CLI not installed")
class ColmapDriverIntegrationTest(unittest.TestCase):
    def test_pipeline_invokes_colmap_binaries(self) -> None:
        # Exercises the real feature_extractor -> matcher -> mapper chain. Random
        # non-overlapping frames will not register, so we assert the driver runs
        # and surfaces the no-model condition rather than silently passing.
        from PIL import Image

        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            imdir = tmp / "imgs"
            imdir.mkdir()
            rng = np.random.default_rng(0)
            for i in range(4):
                arr = (rng.random((240, 320, 3)) * 255).astype("uint8")
                Image.fromarray(arr).save(imdir / f"f_{i:03d}.jpg", quality=92)
            with self.assertRaises(RuntimeError):
                cross_drone.run_colmap_sparse(imdir, tmp / "work")


if __name__ == "__main__":
    unittest.main()
