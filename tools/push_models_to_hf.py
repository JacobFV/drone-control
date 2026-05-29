#!/usr/bin/env python3
"""
Publish trained VLA checkpoints to public Hugging Face repos (with model cards).

Requires a Hugging Face token with write access:  `hf auth login`  (or set
HF_TOKEN). Creates/updates the public repos referenced by the in-app model
registry (drone_control/models.py) and uploads the checkpoint + model card so
the app's Models tab can download them.

    python tools/push_models_to_hf.py                 # push all trained models
    python tools/push_models_to_hf.py transformer-vla # push one
"""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from drone_control.models import REGISTRY  # noqa: E402


def push(model_id: str) -> None:
    from huggingface_hub import HfApi

    spec = next((m for m in REGISTRY if m.id == model_id), None)
    if spec is None:
        print(f"unknown model: {model_id}", file=sys.stderr)
        return
    checkpoint = ROOT / (spec.local_default or f"runs/{spec.checkpoint_file}")
    if not checkpoint.is_file():
        print(f"skip {model_id}: no local checkpoint at {checkpoint} (train it first)", file=sys.stderr)
        return
    card = ROOT / "model_cards" / spec.id / "README.md"

    api = HfApi()
    api.create_repo(spec.hf_repo, repo_type="model", private=False, exist_ok=True)
    api.upload_file(path_or_fileobj=str(checkpoint), path_in_repo=spec.checkpoint_file, repo_id=spec.hf_repo)
    if card.is_file():
        api.upload_file(path_or_fileobj=str(card), path_in_repo="README.md", repo_id=spec.hf_repo)
    print(f"pushed {model_id} -> https://huggingface.co/{spec.hf_repo}", file=sys.stderr)


def main() -> None:
    ids = sys.argv[1:] or [m.id for m in REGISTRY]
    for model_id in ids:
        push(model_id)


if __name__ == "__main__":
    main()
