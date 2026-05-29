"""
Reverse-diffusion image -> action policy (shared model definition).

This module is imported by both ``diffusion_vla_policy.py`` (the inference
process that speaks the batched JSON-lines protocol) and
``train_diffusion_vla.py`` (offline training from logged flight data).

Architecture
------------
A conditional DDPM that denoises a 4-axis continuous action
``(roll, pitch, throttle, yaw)`` in ``[-1, 1]``, conditioned on:
  * a vision embedding from the latest camera frame (small CNN), and
  * a proprioceptive vector (pose, confidence, link state, recent action).

The output projection of the noise predictor is **zero-initialised**, so an
*untrained* model predicts ~zero noise and the reverse process returns ~the
prior mean -> a neutral, centred action (byte 128). That makes the reference
model safe to run before any training (it will not fly well, but it will not
emit wild commands), and training simply teaches it the residual.

Action byte convention: ``128`` is neutral on every axis; ``[-1, 1]`` maps to
``[0, 255]`` via ``(x + 1) / 2 * 255``.
"""

from __future__ import annotations

import base64
import io
import math
from dataclasses import dataclass, field
from typing import Any

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F


ACTION_DIM = 4  # roll, pitch, throttle, yaw (continuous, [-1, 1])
IMAGE_SIZE = 64
PROPRIO_DIM = 12


# --------------------------------------------------------------------------- #
# Feature extraction (shared by inference and training)
# --------------------------------------------------------------------------- #


def decode_frame(payload: dict[str, Any]) -> np.ndarray:
    """Return an ``(3, IMAGE_SIZE, IMAGE_SIZE)`` float array in ``[0, 1]``.

    Falls back to a mid-grey image when no frame is present (dry-run / no
    camera) so the batch shape is always well-defined.
    """

    b64 = payload.get("frameJpegB64")
    if not b64:
        return np.full((3, IMAGE_SIZE, IMAGE_SIZE), 0.5, dtype=np.float32)
    try:
        from PIL import Image

        raw = base64.b64decode(b64)
        with Image.open(io.BytesIO(raw)) as image:
            image = image.convert("RGB").resize((IMAGE_SIZE, IMAGE_SIZE))
            arr = np.asarray(image, dtype=np.float32) / 255.0
        return np.transpose(arr, (2, 0, 1)).copy()
    except Exception:
        return np.full((3, IMAGE_SIZE, IMAGE_SIZE), 0.5, dtype=np.float32)


_LINK_STATES = ("dry_run", "connected", "missing", "error", "stopped", "unknown")


def encode_proprio(payload: dict[str, Any]) -> np.ndarray:
    """Build a fixed-width proprioceptive vector from an observation payload."""

    obs = payload.get("observation", {}) or {}
    pose = obs.get("pose") or {}
    translation = pose.get("translation") or (0.0, 0.0, 0.0)
    if not isinstance(translation, (list, tuple)) or len(translation) < 3:
        translation = (0.0, 0.0, 0.0)
    confidence = float(obs.get("confidence", 0.0) or 0.0)
    pose_conf = float(pose.get("confidence", 0.0) or 0.0)
    battery = obs.get("battery")
    battery = float(battery) if isinstance(battery, (int, float)) else 0.0

    link_state = str(obs.get("linkState", "unknown"))
    link_onehot = [1.0 if link_state == name else 0.0 for name in _LINK_STATES]

    recent = payload.get("recentActions") or []
    last = recent[-1] if recent else {}
    last_axes = [
        (float(last.get("roll", 128)) - 128.0) / 128.0,
        (float(last.get("pitch", 128)) - 128.0) / 128.0,
        (float(last.get("throttle", 128)) - 128.0) / 128.0,
        (float(last.get("yaw", 128)) - 128.0) / 128.0,
    ]

    vector = [
        float(translation[0]),
        float(translation[1]),
        float(translation[2]),
        confidence,
        pose_conf,
        battery,
        *last_axes,
    ]
    vector.extend(link_onehot)
    arr = np.asarray(vector, dtype=np.float32)
    if arr.shape[0] < PROPRIO_DIM:
        arr = np.pad(arr, (0, PROPRIO_DIM - arr.shape[0]))
    return arr[:PROPRIO_DIM]


def action_to_unit(action: dict[str, Any]) -> np.ndarray:
    return np.asarray(
        [
            (float(action.get("roll", 128)) - 128.0) / 128.0,
            (float(action.get("pitch", 128)) - 128.0) / 128.0,
            (float(action.get("throttle", 128)) - 128.0) / 128.0,
            (float(action.get("yaw", 128)) - 128.0) / 128.0,
        ],
        dtype=np.float32,
    ).clip(-1.0, 1.0)


def unit_to_action_bytes(unit: np.ndarray) -> dict[str, int]:
    bytes_ = np.clip((unit + 1.0) / 2.0 * 255.0, 0, 255).round().astype(int)
    return {
        "roll": int(bytes_[0]),
        "pitch": int(bytes_[1]),
        "throttle": int(bytes_[2]),
        "yaw": int(bytes_[3]),
    }


# --------------------------------------------------------------------------- #
# Diffusion schedule
# --------------------------------------------------------------------------- #


@dataclass
class DiffusionSchedule:
    timesteps: int = 50
    betas: torch.Tensor = field(default=None, repr=False)
    alphas_cumprod: torch.Tensor = field(default=None, repr=False)

    def __post_init__(self) -> None:
        betas = _cosine_beta_schedule(self.timesteps)
        alphas = 1.0 - betas
        self.betas = betas
        self.alphas_cumprod = torch.cumprod(alphas, dim=0)

    def to(self, device: torch.device) -> "DiffusionSchedule":
        self.betas = self.betas.to(device)
        self.alphas_cumprod = self.alphas_cumprod.to(device)
        return self


def _cosine_beta_schedule(timesteps: int, s: float = 0.008) -> torch.Tensor:
    steps = timesteps + 1
    x = torch.linspace(0, timesteps, steps)
    alphas_cumprod = torch.cos(((x / timesteps) + s) / (1 + s) * math.pi * 0.5) ** 2
    alphas_cumprod = alphas_cumprod / alphas_cumprod[0]
    betas = 1 - (alphas_cumprod[1:] / alphas_cumprod[:-1])
    return torch.clip(betas, 0.0001, 0.999)


def _timestep_embedding(t: torch.Tensor, dim: int) -> torch.Tensor:
    half = dim // 2
    freqs = torch.exp(-math.log(10000) * torch.arange(half, device=t.device) / max(1, half - 1))
    args = t.float().unsqueeze(1) * freqs.unsqueeze(0)
    emb = torch.cat([torch.cos(args), torch.sin(args)], dim=1)
    if dim % 2 == 1:
        emb = F.pad(emb, (0, 1))
    return emb


# --------------------------------------------------------------------------- #
# Model
# --------------------------------------------------------------------------- #


class VisionEncoder(nn.Module):
    def __init__(self, out_dim: int = 128) -> None:
        super().__init__()
        self.net = nn.Sequential(
            nn.Conv2d(3, 16, 3, stride=2, padding=1), nn.GroupNorm(4, 16), nn.SiLU(),
            nn.Conv2d(16, 32, 3, stride=2, padding=1), nn.GroupNorm(8, 32), nn.SiLU(),
            nn.Conv2d(32, 64, 3, stride=2, padding=1), nn.GroupNorm(8, 64), nn.SiLU(),
            nn.AdaptiveAvgPool2d(1), nn.Flatten(),
        )
        self.proj = nn.Linear(64, out_dim)

    def forward(self, images: torch.Tensor) -> torch.Tensor:
        return self.proj(self.net(images))


class ActionDenoiser(nn.Module):
    """Predicts the *clean* normalised action x0 from a noisy one, given the condition.

    We use the x0 (sample) parameterisation rather than ε-prediction so that a
    zero-initialised output head yields x0 == 0 -> a neutral action (byte 128).
    That keeps the untrained reference model safe; training teaches the residual.
    """

    def __init__(self, cond_dim: int, time_dim: int = 64, hidden: int = 256) -> None:
        super().__init__()
        self.time_dim = time_dim
        self.time_mlp = nn.Sequential(nn.Linear(time_dim, time_dim), nn.SiLU(), nn.Linear(time_dim, time_dim))
        self.net = nn.Sequential(
            nn.Linear(ACTION_DIM + cond_dim + time_dim, hidden), nn.SiLU(),
            nn.Linear(hidden, hidden), nn.SiLU(),
        )
        self.out = nn.Linear(hidden, ACTION_DIM)
        # Zero-init -> untrained model predicts ~zero noise -> neutral action.
        nn.init.zeros_(self.out.weight)
        nn.init.zeros_(self.out.bias)

    def forward(self, x: torch.Tensor, t: torch.Tensor, cond: torch.Tensor) -> torch.Tensor:
        temb = self.time_mlp(_timestep_embedding(t, self.time_dim))
        return self.out(self.net(torch.cat([x, cond, temb], dim=1)))


class DiffusionVLAPolicy(nn.Module):
    def __init__(self, vision_dim: int = 128, proprio_dim: int = PROPRIO_DIM, timesteps: int = 50) -> None:
        super().__init__()
        self.vision = VisionEncoder(vision_dim)
        self.proprio = nn.Sequential(nn.Linear(proprio_dim, 64), nn.SiLU(), nn.Linear(64, 64))
        self.denoiser = ActionDenoiser(cond_dim=vision_dim + 64)
        self.schedule = DiffusionSchedule(timesteps)

    def condition(self, images: torch.Tensor, proprio: torch.Tensor) -> torch.Tensor:
        return torch.cat([self.vision(images), self.proprio(proprio)], dim=1)

    @torch.no_grad()
    def sample(self, images: torch.Tensor, proprio: torch.Tensor, steps: int | None = None) -> torch.Tensor:
        device = images.device
        self.schedule.to(device)
        batch = images.shape[0]
        cond = self.condition(images, proprio)
        x = torch.randn(batch, ACTION_DIM, device=device)
        timesteps = self.schedule.timesteps
        for t in reversed(range(timesteps)):
            t_batch = torch.full((batch,), t, device=device, dtype=torch.long)
            x0 = self.denoiser(x, t_batch, cond).clamp(-1.0, 1.0)
            if t == 0:
                x = x0
                break
            alpha_cumprod = self.schedule.alphas_cumprod[t]
            alpha_cumprod_prev = self.schedule.alphas_cumprod[t - 1]
            beta = self.schedule.betas[t]
            noise = torch.randn_like(x)
            coef_x0 = torch.sqrt(alpha_cumprod_prev) * beta / (1 - alpha_cumprod)
            coef_xt = torch.sqrt(1 - beta) * (1 - alpha_cumprod_prev) / (1 - alpha_cumprod)
            x = coef_x0 * x0 + coef_xt * x + torch.sqrt(beta) * noise
        return x.clamp(-1.0, 1.0)

    def training_loss(self, images: torch.Tensor, proprio: torch.Tensor, actions: torch.Tensor) -> torch.Tensor:
        device = images.device
        self.schedule.to(device)
        batch = images.shape[0]
        cond = self.condition(images, proprio)
        t = torch.randint(0, self.schedule.timesteps, (batch,), device=device)
        alpha_cumprod = self.schedule.alphas_cumprod[t].unsqueeze(1)
        noise = torch.randn_like(actions)
        noisy = torch.sqrt(alpha_cumprod) * actions + torch.sqrt(1 - alpha_cumprod) * noise
        predicted_x0 = self.denoiser(noisy, t, cond)
        return F.mse_loss(predicted_x0, actions)


def build_batch_tensors(payloads: list[dict[str, Any]], device: torch.device) -> tuple[torch.Tensor, torch.Tensor]:
    images = np.stack([decode_frame(p) for p in payloads], axis=0)
    proprio = np.stack([encode_proprio(p) for p in payloads], axis=0)
    return (
        torch.from_numpy(images).to(device),
        torch.from_numpy(proprio).to(device),
    )
