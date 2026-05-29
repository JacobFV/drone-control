#!/usr/bin/env python3
"""
Collect teacher demonstrations from the swarm sim into diffusion-VLA JSONL.

Runs the batched env under the analytic expert (drone_control.sim.expert),
renders each logged step's forward camera, and writes one transition per body
per logged step in exactly the format consumed by
``tools/diffusion_vla_model.py`` / ``tools/train_diffusion_vla.py``:

  {"droneId", "observation": {...}, "frameJpegB64", "recentActions": [...],
   "goalRel": [dx,dy,dz], "style": [...], "action": {roll,pitch,throttle,yaw}}

Action labels are the expert command in DroneAction byte space, so a policy
trained on this data drives the real stack directly.

    python tools/collect_sim_data.py --out data/sim/goto.jsonl \
        --num-envs 48 --steps 500 --log-every 4 --task goto
"""

from __future__ import annotations

import argparse
import base64
import json
import sys
from collections import deque
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import torch  # noqa: E402

from drone_control.sim.dynamics import QuadParams, norm_to_byte  # noqa: E402
from drone_control.sim.env import EnvConfig, SwarmEnv  # noqa: E402
from drone_control.sim.expert import ExpertController  # noqa: E402
from drone_control.sim.render import CameraConfig, CameraRenderer  # noqa: E402


def action_dict(byte_row: torch.Tensor) -> dict:
    return {
        "roll": int(byte_row[0]),
        "pitch": int(byte_row[1]),
        "throttle": int(byte_row[2]),
        "yaw": int(byte_row[3]),
    }


def collect(
    out: str,
    num_envs: int,
    num_drones: int,
    steps: int,
    log_every: int,
    task: str,
    render: bool,
    device: str,
    seed: int,
) -> int:
    params = QuadParams()
    env = SwarmEnv(
        EnvConfig(num_envs=num_envs, num_drones=num_drones, task=task, max_steps=steps + 1, device=device, seed=seed),
        params=params,
    )
    expert = ExpertController(params)
    renderer = CameraRenderer(CameraConfig()) if render else None
    k = env.k

    histories: list[deque] = [deque(maxlen=20) for _ in range(k)]
    out_path = Path(out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    written = 0

    obs = env.reset()
    with out_path.open("w", encoding="utf-8") as handle:
        for step in range(steps):
            command_norm = expert.command(obs)
            command_bytes = norm_to_byte(command_norm)

            if step % log_every == 0:
                frames = None
                if renderer is not None:
                    frames = renderer.render(
                        env.state.pos.cpu().numpy(),
                        env.state.quat.cpu().numpy(),
                        env.goals.cpu().numpy(),
                    )
                pos = env.state.pos.cpu()
                goal_rel = obs.goal_rel.cpu()
                for i in range(k):
                    record = {
                        "droneId": f"sim-{i}",
                        "observation": {
                            "pose": {
                                "translation": [float(v) for v in pos[i]],
                                "confidence": 1.0,
                            },
                            "linkState": "dry_run",
                            "confidence": 1.0,
                        },
                        "frameJpegB64": base64.b64encode(frames[i]).decode("ascii") if frames else None,
                        "recentActions": list(histories[i]),
                        "goalRel": [float(v) for v in goal_rel[i]],
                        "style": [0.0, 0.0, 0.0, 0.0],
                        "action": action_dict(command_bytes[i]),
                    }
                    handle.write(json.dumps(record, separators=(",", ":")) + "\n")
                    written += 1

            for i in range(k):
                histories[i].append(action_dict(command_bytes[i]))

            obs, _reward, done = env.step(command_norm)
            env.reset_done(done)
            obs = env._observe()  # refresh after respawn

    print(f"wrote {written} transitions -> {out_path}", file=sys.stderr)
    return written


def main() -> None:
    parser = argparse.ArgumentParser(description="Collect swarm-sim teacher demonstrations")
    parser.add_argument("--out", default="data/sim/goto.jsonl")
    parser.add_argument("--num-envs", type=int, default=48)
    parser.add_argument("--num-drones", type=int, default=1)
    parser.add_argument("--steps", type=int, default=500)
    parser.add_argument("--log-every", type=int, default=4)
    parser.add_argument("--task", default="goto")
    parser.add_argument("--no-render", action="store_true", help="skip camera frames (proprio-only)")
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()
    collect(
        args.out,
        args.num_envs,
        args.num_drones,
        args.steps,
        args.log_every,
        args.task,
        not args.no_render,
        args.device,
        args.seed,
    )


if __name__ == "__main__":
    main()
