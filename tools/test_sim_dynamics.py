from __future__ import annotations

import unittest

import torch

from drone_control.sim.dynamics import (
    QuadParams,
    QuadrotorDynamics,
    byte_to_norm,
    norm_to_byte,
    quat_to_euler,
)


def cmd(k: int, roll=0.0, pitch=0.0, throttle=0.5, yaw=0.0) -> torch.Tensor:
    c = torch.zeros((k, 4))
    c[:, 0] = roll
    c[:, 1] = pitch
    c[:, 2] = throttle
    c[:, 3] = yaw
    return c


class DynamicsTest(unittest.TestCase):
    def setUp(self) -> None:
        self.dyn = QuadrotorDynamics(QuadParams())

    def test_hover_holds_altitude(self) -> None:
        state = self.dyn.zeros(4)
        state.pos[:, 2] = 5.0
        c = cmd(4, throttle=0.5)  # 0.5 throttle == hover thrust
        for _ in range(100):  # 2 s
            state = self.dyn.step(state, c)
        self.assertTrue(torch.allclose(state.pos[:, 2], torch.full((4,), 5.0), atol=0.05))
        self.assertLess(state.omega.abs().max().item(), 1e-3)

    def test_full_throttle_climbs(self) -> None:
        state = self.dyn.zeros(2)
        state.pos[:, 2] = 1.0
        c = cmd(2, throttle=1.0)
        for _ in range(50):
            state = self.dyn.step(state, c)
        self.assertGreater(state.pos[:, 2].min().item(), 1.5)

    def test_zero_throttle_rests_on_ground(self) -> None:
        state = self.dyn.zeros(3)
        state.pos[:, 2] = 2.0
        c = cmd(3, throttle=0.0)
        for _ in range(200):
            state = self.dyn.step(state, c)
        self.assertTrue((state.pos[:, 2] >= -1e-5).all())
        self.assertLess(state.pos[:, 2].max().item(), 0.05)
        self.assertLess(state.vel[:, 2].abs().max().item(), 1e-3)

    def test_roll_command_tilts_and_drifts(self) -> None:
        state = self.dyn.zeros(1)
        state.pos[:, 2] = 10.0
        c = cmd(1, roll=1.0, throttle=0.6)
        for _ in range(60):
            state = self.dyn.step(state, c)
        roll = quat_to_euler(state.quat)[0, 0].item()
        self.assertGreater(roll, 0.2)  # tilted toward commanded roll
        self.assertGreater(state.pos[:, 1].abs().item(), 0.1)  # drifted horizontally

    def test_byte_norm_roundtrip(self) -> None:
        bytes_in = torch.tensor([[128.0, 128.0, 128.0, 128.0], [0.0, 255.0, 255.0, 0.0]])
        norm = byte_to_norm(bytes_in)
        self.assertTrue(torch.allclose(norm[0], torch.tensor([0.0, 0.0, 128 / 255, 0.0]), atol=1e-3))
        back = norm_to_byte(norm)
        self.assertTrue(torch.allclose(back, bytes_in.round(), atol=1.0))

    def test_batched_many_bodies(self) -> None:
        k = 4096
        state = self.dyn.zeros(k)
        state.pos[:, 2] = 3.0
        c = cmd(k, throttle=0.5)
        state = self.dyn.step(state, c)
        self.assertEqual(state.pos.shape, (k, 3))
        self.assertFalse(torch.isnan(state.pos).any())


if __name__ == "__main__":
    unittest.main()
