"""
VLA model registry: the medium-frequency policies the station can run.

Policies are explicit — there is **no default/fallback**. Until the operator
downloads and selects one, the VLA tier is off (drones get no medium-level
actions). Models live in public Hugging Face repos and can be pulled into the
app on demand; each is served as a subprocess speaking the batched VLA protocol.
"""

from __future__ import annotations

import json
import shutil
import sys
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass(slots=True)
class ModelSpec:
    id: str
    name: str
    kind: str                 # "diffusion" | "transformer"
    description: str
    hf_repo: str              # public Hugging Face repo id
    checkpoint_file: str      # the weights file inside the repo / local dir
    serve_script: str         # tools/ script implementing the batched VLA protocol
    gh_url: str
    params: str               # human-readable parameter count
    # A local-training fallback path: the tiny model is trained in-repo, so it is
    # "present" without a hub download when this file exists.
    local_default: str | None = None


REGISTRY: list[ModelSpec] = [
    ModelSpec(
        id="tiny-diffusion-vla",
        name="Tiny Diffusion VLA",
        kind="diffusion",
        description=(
            "Compact image→action reverse-diffusion policy (CNN vision + proprio, "
            "~0.2M params). Trained in-repo on simulator swarm trajectories. Fast, "
            "CPU/GPU, the baseline policy."
        ),
        hf_repo="drone-control/tiny-diffusion-vla",
        checkpoint_file="vla.pt",
        serve_script="tools/diffusion_vla_policy.py",
        gh_url="https://github.com/jacobfv/drone-control",
        params="~0.2M",
        local_default="runs/vla.pt",
    ),
    ModelSpec(
        id="transformer-vla",
        name="Transformer VLA",
        kind="transformer",
        description=(
            "Larger ViT-style vision encoder + transformer action decoder with "
            "goal/style conditioning (~8M params), fine-tuned on simulator + real "
            "trajectories. Higher capacity for orientation, directive-following "
            "and swarm behaviour."
        ),
        hf_repo="drone-control/transformer-vla",
        checkpoint_file="transformer_vla.pt",
        serve_script="tools/transformer_vla_policy.py",
        gh_url="https://github.com/jacobfv/drone-control",
        params="~8M",
        local_default="runs/transformer_vla.pt",
    ),
]

_BY_ID = {m.id: m for m in REGISTRY}


def _dir_size(path: Path) -> int:
    if path.is_file():
        return path.stat().st_size
    if not path.is_dir():
        return 0
    return sum(p.stat().st_size for p in path.rglob("*") if p.is_file())


class ModelStore:
    """Downloads (HF Hub), tracks, and selects the active VLA policy."""

    def __init__(self, repo_root: Path, runtime: Any, *, selection_path: Path | None = None) -> None:
        self.repo_root = repo_root
        self.runtime = runtime
        self.models_dir = repo_root / "models"
        self.selection_path = selection_path or (repo_root / "config" / "active_model.local.json")
        self._lock = threading.RLock()
        self._active: str | None = None
        self._load_selection()

    # -- checkpoint resolution --------------------------------------------

    def _checkpoint_path(self, spec: ModelSpec) -> Path | None:
        hub = self.models_dir / spec.id / spec.checkpoint_file
        if hub.is_file():
            return hub
        if spec.local_default:
            local = self.repo_root / spec.local_default
            if local.is_file():
                return local
        return None

    def _downloaded(self, spec: ModelSpec) -> bool:
        return self._checkpoint_path(spec) is not None

    def _size(self, spec: ModelSpec) -> int:
        path = self._checkpoint_path(spec)
        return _dir_size(path) if path else 0

    # -- public API --------------------------------------------------------

    def list(self) -> list[dict[str, Any]]:
        with self._lock:
            out = []
            for spec in REGISTRY:
                out.append(
                    {
                        "id": spec.id,
                        "name": spec.name,
                        "kind": spec.kind,
                        "description": spec.description,
                        "hfRepo": spec.hf_repo,
                        "ghUrl": spec.gh_url,
                        "params": spec.params,
                        "downloaded": self._downloaded(spec),
                        "sizeBytes": self._size(spec),
                        "active": self._active == spec.id,
                    }
                )
            return {"models": out, "active": self._active}

    def download(self, model_id: str) -> dict[str, Any]:
        spec = _BY_ID.get(model_id)
        if spec is None:
            raise KeyError(f"unknown model: {model_id}")
        try:
            from huggingface_hub import hf_hub_download
        except Exception as exc:  # pragma: no cover
            raise RuntimeError(f"huggingface_hub not installed: {exc}") from exc
        target_dir = self.models_dir / spec.id
        target_dir.mkdir(parents=True, exist_ok=True)
        path = hf_hub_download(repo_id=spec.hf_repo, filename=spec.checkpoint_file, local_dir=str(target_dir))
        return {"id": spec.id, "path": str(path), "sizeBytes": _dir_size(Path(path))}

    def serve_command(self, model_id: str) -> list[str] | None:
        spec = _BY_ID.get(model_id)
        if spec is None:
            return None
        checkpoint = self._checkpoint_path(spec)
        if checkpoint is None:
            return None
        return [sys.executable, str(self.repo_root / spec.serve_script), "--checkpoint", str(checkpoint), "--steps", "8"]

    def select(self, model_id: str | None) -> dict[str, Any]:
        with self._lock:
            if model_id is None:
                self._active = None
                self.runtime.set_batched_vla_command(None)
            else:
                spec = _BY_ID.get(model_id)
                if spec is None:
                    raise KeyError(f"unknown model: {model_id}")
                command = self.serve_command(model_id)
                if command is None:
                    raise RuntimeError(f"{spec.name} is not downloaded yet — download it first")
                self._active = model_id
                self.runtime.set_batched_vla_command(command)
            self._save_selection()
        return self.list()

    # -- persistence -------------------------------------------------------

    def _load_selection(self) -> None:
        if self.selection_path.is_file():
            try:
                data = json.loads(self.selection_path.read_text())
            except (OSError, json.JSONDecodeError):
                return
            model_id = data.get("active")
            if model_id and self.serve_command(model_id):
                self._active = model_id
                self.runtime.set_batched_vla_command(self.serve_command(model_id))

    def _save_selection(self) -> None:
        self.selection_path.parent.mkdir(parents=True, exist_ok=True)
        self.selection_path.write_text(json.dumps({"active": self._active}))
