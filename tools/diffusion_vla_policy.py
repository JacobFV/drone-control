#!/usr/bin/env python3
"""
Batched reverse-diffusion VLA policy process.

Speaks the batched JSON-lines protocol consumed by
``drone_control.controllers.local_vla.BatchLocalVLAClient``:

  stdin  (one line per control window):  {"batch": [<payload>, ...]}
  stdout (one line per control window):  {"results": [<result>, ...]}

Each ``<payload>`` carries ``droneId``, ``observation``, ``frameJpegB64``,
``recentActions``, ``constraints`` and ``mission``. Each ``<result>`` echoes
``droneId`` and an ``action`` (roll/pitch/throttle/yaw bytes + flags),
``confidence`` and ``reason`` — exactly what ``controllers.vla.parse_vla_output``
validates.

Run untrained (safe, neutral output) or with trained weights:

    python tools/diffusion_vla_policy.py
    python tools/diffusion_vla_policy.py --checkpoint runs/vla.pt --steps 10

Swap in your own model by replacing this process with anything that honours the
same protocol.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import torch  # noqa: E402

from diffusion_vla_model import (  # noqa: E402
    DiffusionVLAPolicy,
    build_batch_tensors,
    unit_to_action_bytes,
)


def load_policy(checkpoint: str | None, device: torch.device) -> tuple[DiffusionVLAPolicy, bool]:
    model = DiffusionVLAPolicy()
    trained = False
    if checkpoint:
        state = torch.load(checkpoint, map_location=device)
        model.load_state_dict(state["model"] if "model" in state else state)
        trained = True
    model.to(device).eval()
    return model, trained


def run(checkpoint: str | None, sample_steps: int | None, device_str: str) -> None:
    device = torch.device(device_str)
    model, trained = load_policy(checkpoint, device)
    reason = "diffusion_vla" if trained else "diffusion_vla_untrained"
    base_confidence = 0.6 if trained else 0.05

    # Pay CUDA/kernel cold-start before serving so the first real tick is fast.
    warm_images, warm_proprio = build_batch_tensors([{"droneId": "_warmup"}], device)
    model.sample(warm_images, warm_proprio, steps=sample_steps)

    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            message = json.loads(line)
            payloads = message.get("batch") or []
        except json.JSONDecodeError:
            sys.stdout.write(json.dumps({"results": []}) + "\n")
            sys.stdout.flush()
            continue

        results = []
        if payloads:
            images, proprio = build_batch_tensors(payloads, device)
            units = model.sample(images, proprio, steps=sample_steps).cpu().numpy()
            for payload, unit in zip(payloads, units):
                action = unit_to_action_bytes(unit)
                results.append(
                    {
                        "droneId": payload.get("droneId"),
                        "action": action,
                        "confidence": base_confidence,
                        "reason": reason,
                    }
                )
        sys.stdout.write(json.dumps({"results": results}, separators=(",", ":")) + "\n")
        sys.stdout.flush()


def main() -> None:
    parser = argparse.ArgumentParser(description="Batched reverse-diffusion VLA policy")
    parser.add_argument("--checkpoint", default=None, help="path to a trained checkpoint (.pt)")
    parser.add_argument("--steps", type=int, default=None, help="reverse diffusion steps (default: full schedule)")
    parser.add_argument(
        "--device",
        default="cuda" if torch.cuda.is_available() else "cpu",
        help="torch device (default: cuda if available)",
    )
    args = parser.parse_args()
    run(args.checkpoint, args.steps, args.device)


if __name__ == "__main__":
    main()
