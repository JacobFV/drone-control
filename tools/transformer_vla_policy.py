#!/usr/bin/env python3
"""
Batched Transformer VLA policy process (same JSON-lines protocol as
diffusion_vla_policy.py). Serves the larger ViT+transformer policy.

    python tools/transformer_vla_policy.py --checkpoint runs/transformer_vla.pt
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import torch  # noqa: E402

from transformer_vla_model import TransformerVLA, build_batch_tensors, unit_to_action_bytes  # noqa: E402


def run(checkpoint: str | None, device_str: str) -> None:
    device = torch.device(device_str)
    model = TransformerVLA()
    trained = False
    if checkpoint:
        state = torch.load(checkpoint, map_location=device)
        model.load_state_dict(state["model"] if "model" in state else state)
        trained = True
    model.to(device).eval()
    reason = "transformer_vla" if trained else "transformer_vla_untrained"
    confidence = 0.7 if trained else 0.05

    warm_i, warm_p = build_batch_tensors([{"droneId": "_warmup"}], device)
    model.sample(warm_i, warm_p)

    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            payloads = json.loads(line).get("batch") or []
        except json.JSONDecodeError:
            sys.stdout.write(json.dumps({"results": []}) + "\n")
            sys.stdout.flush()
            continue
        results = []
        if payloads:
            images, proprio = build_batch_tensors(payloads, device)
            units = model.sample(images, proprio).cpu().numpy()
            for payload, unit in zip(payloads, units):
                results.append(
                    {
                        "droneId": payload.get("droneId"),
                        "action": unit_to_action_bytes(unit),
                        "confidence": confidence,
                        "reason": reason,
                    }
                )
        sys.stdout.write(json.dumps({"results": results}, separators=(",", ":")) + "\n")
        sys.stdout.flush()


def main() -> None:
    parser = argparse.ArgumentParser(description="Batched transformer VLA policy")
    parser.add_argument("--checkpoint", default=None)
    parser.add_argument("--steps", type=int, default=None, help="ignored (protocol compatibility)")
    parser.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    args = parser.parse_args()
    run(args.checkpoint, args.device)


if __name__ == "__main__":
    main()
