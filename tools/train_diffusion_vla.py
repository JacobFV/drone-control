#!/usr/bin/env python3
"""
Train the reverse-diffusion VLA policy from logged flight data.

Consumes JSONL produced by the runtime action logger (see
``RuntimeManager(..., vla_log_path=...)``). Each line is one transition:

  {"droneId": "...", "observation": {...}, "frameJpegB64": "...",
   "recentActions": [...], "action": {"roll":.., "pitch":.., "throttle":.., "yaw":..}}

The ``action`` field is the supervision target (the command actually sent).
Standard DDPM training: add noise to the normalised target action and learn to
predict it, conditioned on the frame + proprioceptive features.

    python tools/train_diffusion_vla.py logs/*.jsonl --out runs/vla.pt --epochs 20

The resulting checkpoint plugs straight into ``diffusion_vla_policy.py --checkpoint``.
"""

from __future__ import annotations

import argparse
import glob
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import torch  # noqa: E402
from torch.utils.data import DataLoader, Dataset  # noqa: E402

from diffusion_vla_model import (  # noqa: E402
    DiffusionVLAPolicy,
    action_to_unit,
    decode_frame,
    encode_proprio,
)


class TransitionDataset(Dataset):
    def __init__(self, paths: list[str]) -> None:
        self.records: list[dict] = []
        for path in paths:
            for file in sorted(glob.glob(path)):
                with open(file, "r", encoding="utf-8") as handle:
                    for line in handle:
                        line = line.strip()
                        if not line:
                            continue
                        try:
                            record = json.loads(line)
                        except json.JSONDecodeError:
                            continue
                        if isinstance(record, dict) and isinstance(record.get("action"), dict):
                            self.records.append(record)
        if not self.records:
            raise SystemExit("no usable transitions found in the provided logs")

    def __len__(self) -> int:
        return len(self.records)

    def __getitem__(self, index: int):
        record = self.records[index]
        image = decode_frame(record)
        proprio = encode_proprio(record)
        action = action_to_unit(record["action"])
        return (
            torch.from_numpy(image),
            torch.from_numpy(proprio),
            torch.from_numpy(action),
        )


def train(paths: list[str], out: str, epochs: int, batch_size: int, lr: float, device_str: str) -> None:
    device = torch.device(device_str)
    dataset = TransitionDataset(paths)
    loader = DataLoader(dataset, batch_size=batch_size, shuffle=True, drop_last=False)
    model = DiffusionVLAPolicy().to(device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=lr)

    print(f"training on {len(dataset)} transitions, device={device}", file=sys.stderr)
    model.train()
    for epoch in range(epochs):
        total = 0.0
        count = 0
        for images, proprio, actions in loader:
            images = images.to(device)
            proprio = proprio.to(device)
            actions = actions.to(device)
            loss = model.training_loss(images, proprio, actions)
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            total += float(loss.item()) * images.shape[0]
            count += images.shape[0]
        print(f"epoch {epoch + 1}/{epochs}  loss={total / max(1, count):.5f}", file=sys.stderr)

    Path(out).parent.mkdir(parents=True, exist_ok=True)
    torch.save({"model": model.state_dict()}, out)
    print(f"saved checkpoint -> {out}", file=sys.stderr)


def main() -> None:
    parser = argparse.ArgumentParser(description="Train the diffusion VLA policy from logged transitions")
    parser.add_argument("logs", nargs="+", help="JSONL log files or globs")
    parser.add_argument("--out", default="runs/vla.pt", help="output checkpoint path")
    parser.add_argument("--epochs", type=int, default=20)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--lr", type=float, default=2e-4)
    parser.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    args = parser.parse_args()
    train(args.logs, args.out, args.epochs, args.batch_size, args.lr, args.device)


if __name__ == "__main__":
    main()
