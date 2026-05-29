from __future__ import annotations

import unittest

import torch

from drone_control.sim.dynamics import QuadParams
from drone_control.sim.env import OBS_DIM, EnvConfig, SwarmEnv
from drone_control.sim.expert import ExpertController


class SwarmEnvTest(unittest.TestCase):
    def test_reset_shapes(self) -> None:
        env = SwarmEnv(EnvConfig(num_envs=8, num_drones=2, task="goto", seed=0))
        obs = env.reset()
        self.assertEqual(obs.pos.shape, (16, 3))
        self.assertEqual(obs.flat.shape, (16, OBS_DIM))

    def test_expert_reaches_goal(self) -> None:
        params = QuadParams()
        env = SwarmEnv(EnvConfig(num_envs=32, num_drones=1, task="goto", max_steps=600, seed=1), params=params)
        expert = ExpertController(params)
        obs = env.reset()
        start_dist = obs.goal_rel.norm(dim=1).mean().item()
        for _ in range(400):
            cmd = expert.command(obs)
            obs, _reward, _done = env.step(cmd)
        end_dist = obs.goal_rel.norm(dim=1).mean().item()
        self.assertLess(end_dist, 0.5)  # expert converges to within 0.5 m
        self.assertLess(end_dist, start_dist * 0.2)

    def test_set_goals_and_byte_commands(self) -> None:
        env = SwarmEnv(EnvConfig(num_envs=4, num_drones=1, task="hover", seed=2))
        env.reset()
        env.set_goals(torch.tensor([[0.0, 0.0, 3.0]] * 4))
        # neutral byte command should not crash and should integrate.
        bytes_cmd = torch.tensor([[128.0, 128.0, 128.0, 128.0]] * 4)
        obs, reward, done = env.step(bytes_cmd, as_bytes=True)
        self.assertEqual(obs.pos.shape, (4, 3))
        self.assertEqual(reward.shape, (4,))
        self.assertEqual(done.shape, (4,))

    def test_reset_done_respawns(self) -> None:
        env = SwarmEnv(EnvConfig(num_envs=4, num_drones=1, task="goto", max_steps=5, seed=3))
        env.reset()
        done = torch.tensor([True, False, True, False])
        before = env.state.pos.clone()
        env.reset_done(done)
        after = env.state.pos
        # done bodies moved (respawned), others unchanged.
        self.assertFalse(torch.allclose(before[0], after[0]))
        self.assertTrue(torch.allclose(before[1], after[1]))


if __name__ == "__main__":
    unittest.main()
