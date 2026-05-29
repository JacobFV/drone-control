from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np  # noqa: E402

try:
    import torch

    _CUDA = torch.cuda.is_available()
except Exception:  # pragma: no cover
    _CUDA = False


@unittest.skipUnless(_CUDA, "CUDA required for the train+rollout policy test")
class SimPolicyLoopTest(unittest.TestCase):
    """End-to-end: collect sim demos -> train goal-conditioned VLA -> verify the
    learned policy flies toward goals in closed loop better than untrained."""

    def test_trained_policy_beats_untrained(self) -> None:
        from collect_sim_data import collect
        from train_diffusion_vla import train
        from diffusion_vla_model import DiffusionVLAPolicy, build_batch_tensors, unit_to_action_bytes
        from drone_control.sim.env import EnvConfig, SwarmEnv
        from drone_control.sim.rollout import run_policy_rollout

        device = torch.device("cuda")
        with tempfile.TemporaryDirectory() as tmp:
            data = str(Path(tmp) / "demos.jsonl")
            ckpt = str(Path(tmp) / "policy.pt")
            collect(data, num_envs=12, num_drones=1, steps=180, log_every=5, task="goto", render=True, device="cpu", seed=0)
            train([data], ckpt, epochs=15, batch_size=64, lr=2e-4, device_str="cuda")

            def make_policy(path):
                m = DiffusionVLAPolicy()
                if path:
                    m.load_state_dict(torch.load(path, map_location=device)["model"])
                m.to(device).eval()

                def step(payloads):
                    imgs, prop = build_batch_tensors(payloads, device)
                    units = m.sample(imgs, prop, steps=10).cpu().numpy()
                    return np.stack([list(unit_to_action_bytes(u).values()) for u in units])

                return step

            env_u = SwarmEnv(EnvConfig(num_envs=12, num_drones=1, task="goto", max_steps=300, device="cuda", seed=5))
            untrained = run_policy_rollout(env_u, make_policy(None), steps=180, render=True)
            env_t = SwarmEnv(EnvConfig(num_envs=12, num_drones=1, task="goto", max_steps=300, device="cuda", seed=5))
            trained = run_policy_rollout(env_t, make_policy(ckpt), steps=180, render=True)

            # Trained must meaningfully close the gap; untrained should not.
            self.assertGreater(trained.improvement, 0.4)
            self.assertGreater(trained.improvement, untrained.improvement + 0.3)


if __name__ == "__main__":
    unittest.main()
