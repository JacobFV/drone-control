"""
Transformer VLA — a larger, higher-capacity image→action policy.

A ViT-style patch encoder over the camera frame + a proprio/goal/style token,
fused by a small transformer encoder and regressed to a continuous 4-axis action
in [-1, 1]. Shares the exact feature interface (decode_frame / encode_proprio /
action byte convention) with the tiny diffusion VLA, so it drops into the same
batched serving protocol and trains on the same simulator + real trajectories.

~8M params — meant as the "serious" policy with more headroom for orientation,
directive-following and swarm behaviour than the tiny baseline.
"""

from __future__ import annotations

from typing import Any

import numpy as np
import torch
import torch.nn as nn

# Reuse the shared feature extraction so both policies see identical inputs.
from diffusion_vla_model import (  # noqa: E402
    ACTION_DIM,
    IMAGE_SIZE,
    PROPRIO_DIM,
    decode_frame,
    encode_proprio,
)

PATCH = 8
N_PATCHES = (IMAGE_SIZE // PATCH) ** 2  # 64 patches at 64/8


class PatchEmbed(nn.Module):
    def __init__(self, dim: int) -> None:
        super().__init__()
        self.proj = nn.Conv2d(3, dim, kernel_size=PATCH, stride=PATCH)

    def forward(self, images: torch.Tensor) -> torch.Tensor:
        x = self.proj(images)                       # [B, dim, H/P, W/P]
        return x.flatten(2).transpose(1, 2)          # [B, N, dim]


class TransformerVLA(nn.Module):
    def __init__(self, dim: int = 256, depth: int = 6, heads: int = 8, proprio_dim: int = PROPRIO_DIM) -> None:
        super().__init__()
        self.patch = PatchEmbed(dim)
        self.pos = nn.Parameter(torch.zeros(1, N_PATCHES + 1, dim))
        nn.init.trunc_normal_(self.pos, std=0.02)
        self.cls = nn.Parameter(torch.zeros(1, 1, dim))
        self.proprio = nn.Sequential(nn.Linear(proprio_dim, dim), nn.SiLU(), nn.Linear(dim, dim))
        layer = nn.TransformerEncoderLayer(dim, heads, dim * 4, dropout=0.0, batch_first=True, activation="gelu")
        self.encoder = nn.TransformerEncoder(layer, depth)
        self.head = nn.Sequential(nn.LayerNorm(dim), nn.Linear(dim, dim), nn.SiLU(), nn.Linear(dim, ACTION_DIM))
        # Zero-init the final layer -> untrained model emits ~neutral action.
        nn.init.zeros_(self.head[-1].weight)
        nn.init.zeros_(self.head[-1].bias)

    def forward(self, images: torch.Tensor, proprio: torch.Tensor) -> torch.Tensor:
        b = images.shape[0]
        patches = self.patch(images)                              # [B, N, dim]
        cls = self.cls.expand(b, -1, -1) + self.proprio(proprio).unsqueeze(1)
        x = torch.cat([cls, patches], dim=1) + self.pos
        x = self.encoder(x)
        return torch.tanh(self.head(x[:, 0]))                     # [B, ACTION_DIM] in [-1,1]

    @torch.no_grad()
    def sample(self, images: torch.Tensor, proprio: torch.Tensor, steps: int | None = None) -> torch.Tensor:
        # Deterministic regressor; ``steps`` ignored (protocol compatibility).
        return self.forward(images, proprio).clamp(-1.0, 1.0)

    def training_loss(
        self,
        images: torch.Tensor,
        proprio: torch.Tensor,
        actions: torch.Tensor,
        axis_weights: torch.Tensor | None = None,
        sample_weights: torch.Tensor | None = None,
    ) -> torch.Tensor:
        pred = self.forward(images, proprio)
        err = (pred - actions) ** 2
        if axis_weights is not None:
            err = err * axis_weights.to(err.device).view(1, -1)
        per_sample = err.mean(dim=1)
        if sample_weights is not None:
            w = sample_weights.to(err.device)
            return (per_sample * w).sum() / w.sum().clamp_min(1e-6)
        return per_sample.mean()


def build_batch_tensors(payloads: list[dict[str, Any]], device: torch.device) -> tuple[torch.Tensor, torch.Tensor]:
    images = np.stack([decode_frame(p) for p in payloads], axis=0)
    proprio = np.stack([encode_proprio(p) for p in payloads], axis=0)
    return torch.from_numpy(images).to(device), torch.from_numpy(proprio).to(device)


def unit_to_action_bytes(unit: np.ndarray) -> dict[str, int]:
    b = np.clip((unit + 1.0) / 2.0 * 255.0, 0, 255).round().astype(int)
    return {"roll": int(b[0]), "pitch": int(b[1]), "throttle": int(b[2]), "yaw": int(b[3])}
