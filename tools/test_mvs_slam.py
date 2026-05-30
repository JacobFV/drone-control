"""Multi-view SLAM / dense MVS front-end tests, plus the eval-harness oracle.

These pin the two things that were subtly broken during bring-up: the plane-sweep
homography sign and the lossy quaternion encoding of the (left-handed) optical
frame. They also assert the headline property — multi-view depth is metric and
positively correlated with raycast ground truth — so a regression that silently
inverts or de-scales depth fails loudly.
"""

from __future__ import annotations

import unittest
import warnings

import numpy as np


class PoseRotationTest(unittest.TestCase):
    def test_matrix_passthrough_is_lossless(self) -> None:
        # The optical frame (right, down, forward) is left-handed (det -1) and a
        # quaternion cannot carry it — the pose must ship the matrix directly.
        from drone_control.perception.segmentation import _pose_rotation
        R = np.array([[-1.0, 0.0, 0.0], [0.0, -0.05, 0.999], [0.0, -0.999, -0.05]])
        out = _pose_rotation({"x": 0, "y": 0, "z": 0, "R": R.tolist()})
        self.assertTrue(np.allclose(out, R, atol=1e-6))

    def test_sim_camera_pose_carries_matrix(self) -> None:
        import time
        from drone_control.environment.sim_env import SimEnvironment
        from drone_control.perception.segmentation import _pose_rotation
        from drone_control.sim.session import SimSessionConfig
        env = SimEnvironment(SimSessionConfig(num_drones=1, scene="warehouse",
                                              render=False, max_speed=True))
        env.start(); time.sleep(0.4)
        pose = env.camera_pose("sim-0"); env.stop()
        self.assertIn("R", pose)
        R = _pose_rotation(pose)
        # Reconstructed frame must be the true (improper) optical frame.
        self.assertAlmostEqual(float(np.linalg.det(R)), -1.0, places=3)


class OracleTest(unittest.TestCase):
    def test_floor_depth_matches_height(self) -> None:
        # A camera looking straight down from height h sees the floor at depth h.
        from drone_control.sim.render import CameraConfig
        from drone_control.sim.scenes import build_scene
        from tools.depth_eval.oracle import raycast_depth
        scene = build_scene("open_field")  # no boxes: floor depth is unambiguous
        cfg = CameraConfig(width=64, height=48)
        # camera->world columns (right, down, forward): forward = -z so the
        # camera looks straight down; right = +x, down = +y.
        R = np.column_stack([[1, 0, 0], [0, 1, 0], [0, 0, -1]]).astype(float)
        depth = raycast_depth(scene, np.array([0.0, 0.0, 3.0]), R, cfg, include_dynamic=False)
        center_depth = depth[24, 32]
        self.assertTrue(np.isfinite(center_depth))
        self.assertAlmostEqual(center_depth, 3.0, delta=0.2)


class GeometryTest(unittest.TestCase):
    def setUp(self) -> None:
        from tools.depth_eval.sequence import generate
        self.frames = generate("warehouse", n=8, noise="off", image_size=96, seed=0)

    def test_homography_reprojection_is_exact(self) -> None:
        from drone_control.perception.mvs import _homography
        from drone_control.perception.slam import _intrinsics, _projection
        K = _intrinsics(96, 72, 75.0); Kinv = np.linalg.inv(K)
        fr, fs = self.frames[5], self.frames[2]
        P_s = _projection(K, fs.cam_rot, fs.center)
        gt = fr.gt_depth
        ys, xs = np.where(np.isfinite(gt))
        errs = []
        for i in range(0, len(xs), max(1, len(xs) // 50)):
            px, py, d = xs[i], ys[i], gt[ys[i], xs[i]]
            ray = np.array([(px - 48) / K[0, 0], (py - 36) / K[0, 0], 1.0])
            ray /= np.linalg.norm(ray)
            X = fr.center + d * (fr.cam_rot @ ray)
            z = float((X - fr.center) @ fr.cam_rot[:, 2])
            h = P_s @ np.append(X, 1.0); ps_true = h[:2] / h[2]
            hp = _homography(K, Kinv, fr.cam_rot, fr.center, fs.cam_rot, fs.center, z)
            hp = hp @ np.array([px, py, 1.0]); ps_h = hp[:2] / hp[2]
            errs.append(np.linalg.norm(ps_true - ps_h))
        self.assertLess(max(errs), 1e-3)


class MultiViewDepthTest(unittest.TestCase):
    def test_dense_depth_is_metric_and_correlated(self) -> None:
        from tools.depth_eval.metrics import metrics
        from tools.depth_eval.sequence import generate
        from drone_control.perception.slam import MultiViewSLAM
        warnings.simplefilter("ignore")
        frames = generate("warehouse", n=22, noise="medium", image_size=128, seed=0)
        slam = MultiViewSLAM(far=22, near=0.5)
        rows = []
        for f in frames:
            slam.process("sim-0", f.jpeg, f.pose)
            dm = slam.latest_depth_map("sim-0")
            if dm is not None and len(slam._windows["sim-0"].grays) >= 6:
                rows.append(metrics(dm, f.gt_depth))
        self.assertGreater(len(rows), 5)
        corr = np.nanmean([r["corr"] for r in rows])
        abs_rel = np.nanmean([r["absRel"] for r in rows])
        # Metric multi-view depth: positively correlated and far better than the
        # broken monocular prior (raw absRel ~1.0). Loose thresholds vs the
        # observed corr~0.65 / absRel~0.43 so the test is stable, not flaky.
        self.assertGreater(corr, 0.4)
        self.assertLess(abs_rel, 0.65)
        # Cloud has real structure (gating/outlier removal keeps it lean + clean).
        self.assertGreater(slam.status()["points"], 400)


class CloudStabilityTest(unittest.TestCase):
    def test_fused_cloud_is_stable_under_revisit(self) -> None:
        """Re-observing the same surfaces for dozens of frames must NOT grow the
        cloud without bound. Across 3 orbit loops the raw append stream keeps
        growing every loop, while the voxel-fused cloud plateaus after loop 1 and
        its centroid stays put (no drift / blow-up)."""
        from tools.depth_eval.sequence import generate
        from drone_control.perception.slam import MultiViewSLAM
        warnings.simplefilter("ignore")

        def orbit(i, n):
            ang = 2 * np.pi * 3 * i / max(1, n - 1)   # 3 loops
            center = np.array([0.0, 0.0, 1.6])
            cam = center + np.array([5.0 * np.cos(ang), 5.0 * np.sin(ang), 0.3 * np.sin(ang * 2)])
            fwd = center - cam
            return cam, fwd / np.linalg.norm(fwd)

        frames = generate("warehouse", n=60, noise="medium", seed=4,
                          image_size=160, camera_model="ov2640", path=orbit)
        slam = MultiViewSLAM(far=22, near=0.5)
        fused, naive, cents = [], [], []
        for f in frames:
            slam.process("sim-0", f.jpeg, f.pose)
            xyz, _ = slam.cloud_arrays()
            fused.append(xyz.shape[0]); naive.append(slam.cloud_full_arrays()[0].shape[0])
            if xyz.shape[0] > 50:
                cents.append(xyz.mean(0))
        l1, l3 = len(frames) // 3, len(frames) - 1
        naive_growth = naive[l3] / max(1, naive[l1])
        fused_growth = fused[l3] / max(1, fused[l1])
        # The raw append stream keeps growing loop over loop; the fused cloud is
        # bounded — it grows far slower (voxel fusion + decay) and converges.
        self.assertGreater(naive_growth, 2.0)
        self.assertLess(fused_growth, naive_growth * 0.7, "fused cloud not stabilising vs naive")
        self.assertGreater(fused[l3], 150, "fused cloud should have real structure")
        # The fused centroid stays put (no drift / blow-up) across the revisits.
        cents = np.array(cents)
        self.assertLess(float(np.linalg.norm(cents[-1] - cents[len(cents) // 2])), 0.6)


if __name__ == "__main__":
    unittest.main()
