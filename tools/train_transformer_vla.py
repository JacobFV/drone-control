#!/usr/bin/env python3
"""
Train the Transformer VLA on logged transitions (same JSONL as the diffusion
VLA). Weighted loss toward orientation (yaw) + directive-following (roll/pitch).

    python tools/train_transformer_vla.py data/sim/vla_train.jsonl --out runs/transformer_vla.pt --epochs 30
"""

from __future__ import annotations

import argparse
import glob
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import torch  # noqa: E402
from torch.utils.data import DataLoader, Dataset  # noqa: E402

from diffusion_vla_model import action_to_unit, decode_frame, encode_proprio  # noqa: E402
from transformer_vla_model import TransformerVLA  # noqa: E402

AXIS_WEIGHTS = torch.tensor([1.2, 1.2, 0.8, 1.5])  # roll, pitch, throttle, yaw


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
            raise SystemExit("no usable transitions found")

    def __len__(self) -> int:
        return len(self.records)

    def __getitem__(self, index: int):
        record = self.records[index]
        return (
            torch.from_numpy(decode_frame(record)),
            torch.from_numpy(encode_proprio(record)),
            torch.from_numpy(action_to_unit(record["action"])),
        )


def train(paths: list[str], out: str, epochs: int, batch_size: int, lr: float, device_str: str) -> None:
    device = torch.device(device_str)
    dataset = TransitionDataset(paths)
    loader = DataLoader(dataset, batch_size=batch_size, shuffle=True, drop_last=False)
    model = TransformerVLA().to(device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=lr, weight_decay=1e-4)
    n_params = sum(p.numel() for p in model.parameters())
    axis_weights = AXIS_WEIGHTS.to(device)
    print(f"training transformer VLA ({n_params/1e6:.1f}M params) on {len(dataset)} transitions, device={device}", file=sys.stderr)
    model.train()
    for epoch in range(epochs):
        total, count = 0.0, 0
        for images, proprio, actions in loader:
            images, proprio, actions = images.to(device), proprio.to(device), actions.to(device)
            sample_weights = 1.0 + 2.0 * actions.abs().mean(dim=1)
            loss = model.training_loss(images, proprio, actions, axis_weights, sample_weights)
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
    parser = argparse.ArgumentParser(description="Train the transformer VLA")
    parser.add_argument("logs", nargs="+")
    parser.add_argument("--out", default="runs/transformer_vla.pt")
    parser.add_argument("--epochs", type=int, default=30)
    parser.add_argument("--batch-size", type=int, default=128)
    parser.add_argument("--lr", type=float, default=3e-4)
    parser.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    args = parser.parse_args()
    train(args.logs, args.out, args.epochs, args.batch_size, args.lr, args.device)


if __name__ == "__main__":
    main()
