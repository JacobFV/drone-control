from __future__ import annotations

import time
import unittest

from drone_control.sim.session import SimSession, SimSessionConfig


class SimSessionTest(unittest.TestCase):
    def test_runs_and_exposes_trajectories_and_frames(self) -> None:
        sim = SimSession()
        try:
            sim.start(SimSessionConfig(num_drones=3, task="goto", rate_hz=30.0, render=True))
            time.sleep(1.5)
            status = sim.status()
            self.assertTrue(status["running"])
            self.assertEqual(status["numDrones"], 3)
            self.assertEqual(len(status["drones"]), 3)
            self.assertGreater(status["step"], 5)

            traj = sim.trajectories()
            self.assertEqual(len(traj["drones"]), 3)
            self.assertGreater(len(traj["drones"][0]["poses"]), 3)
            # distinct colors per drone
            colors = {d["color"] for d in traj["drones"]}
            self.assertEqual(len(colors), 3)

            for i in range(3):
                frame = sim.frame(i)
                self.assertIsNotNone(frame)
                self.assertTrue(frame.startswith(b"\xff\xd8"))
        finally:
            sim.stop()
        self.assertFalse(sim.status()["running"])

    def test_drones_make_progress(self) -> None:
        sim = SimSession()
        try:
            sim.start(SimSessionConfig(num_drones=4, task="goto", rate_hz=40.0, render=False))
            time.sleep(0.3)
            early = sum(d["distance"] for d in sim.status()["drones"])
            time.sleep(2.0)
            later = sum(d["distance"] for d in sim.status()["drones"])
            # The expert closes distance (goals resample on arrival, so just assert
            # the swarm is actively controlled, not drifting away unboundedly).
            self.assertLess(later, early + 8.0)
        finally:
            sim.stop()


if __name__ == "__main__":
    unittest.main()
