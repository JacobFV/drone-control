from __future__ import annotations

import json
import math
import os
import shutil
import subprocess
import sys
import threading
import time
import uuid
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from PIL import Image

from drone_control.pose_estimator import load_pose_track
from drone_control.store import ControlStationStore


SPLAT_EXTENSIONS = (".splat", ".ply", ".spz")


@dataclass(slots=True)
class ReconstructionJob:
    id: str
    flight_id: str
    source_record_id: str
    pose_record_id: str | None
    work_dir: Path
    dataset_dir: Path
    processed_dir: Path
    output_dir: Path
    export_dir: Path
    log_path: Path
    state: str = "queued"
    stage: str = "queued"
    started_at: float = field(default_factory=time.time)
    updated_at: float = field(default_factory=time.time)
    ended_at: float | None = None
    error: str | None = None
    current_command: list[str] = field(default_factory=list)
    dataset_record_id: str | None = None
    splat_record_id: str | None = None
    config_path: str | None = None
    return_code: int | None = None
    max_images: int | None = None
    max_iterations: int = 30000
    fps: float = 12.0

    @property
    def active(self) -> bool:
        return self.state in {"queued", "running", "stopping"}

    def as_dict(self, *, log_tail: str = "") -> dict[str, object]:
        return {
            "id": self.id,
            "flightId": self.flight_id,
            "sourceRecordId": self.source_record_id,
            "poseRecordId": self.pose_record_id,
            "state": self.state,
            "stage": self.stage,
            "active": self.active,
            "startedAt": self.started_at,
            "updatedAt": self.updated_at,
            "endedAt": self.ended_at,
            "error": self.error,
            "currentCommand": self.current_command,
            "workDir": str(self.work_dir),
            "datasetDir": str(self.dataset_dir),
            "processedDir": str(self.processed_dir),
            "outputDir": str(self.output_dir),
            "exportDir": str(self.export_dir),
            "logPath": str(self.log_path),
            "datasetRecordId": self.dataset_record_id,
            "splatRecordId": self.splat_record_id,
            "configPath": self.config_path,
            "returnCode": self.return_code,
            "maxImages": self.max_images,
            "maxIterations": self.max_iterations,
            "fps": self.fps,
            "logTail": log_tail,
        }


class ReconstructionManager:
    def __init__(self, *, store: ControlStationStore, work_root: Path) -> None:
        self.store = store
        self.work_root = work_root
        self.work_root.mkdir(parents=True, exist_ok=True)
        self._lock = threading.RLock()
        self._jobs: dict[str, ReconstructionJob] = {}
        self._latest_by_flight: dict[str, str] = {}
        self._processes: dict[str, subprocess.Popen[str]] = {}
        self._stop_events: dict[str, threading.Event] = {}

    def tools_status(self) -> dict[str, object]:
        tools = {
            "nsProcessData": self._which_tool("ns-process-data"),
            "nsTrain": self._which_tool("ns-train"),
            "nsExport": self._which_tool("ns-export"),
            "colmap": self._which_tool("colmap"),
        }
        ready = bool(tools["nsProcessData"] and tools["nsTrain"] and tools["nsExport"])
        return {
            "ready": ready,
            "tools": tools,
            "method": "nerfstudio/splatfacto",
            "viewer": "gsplat.js",
            "cudaVisibleDevices": os.environ.get("CUDA_VISIBLE_DEVICES", ""),
        }

    def start(
        self,
        *,
        flight_id: str,
        frame_record: dict[str, object],
        pose_record: dict[str, object] | None,
        max_images: int | None = None,
        max_iterations: int | None = None,
        fps: float = 12.0,
    ) -> ReconstructionJob:
        with self._lock:
            current_id = self._latest_by_flight.get(flight_id)
            current = self._jobs.get(current_id or "")
            if current and current.active:
                raise RuntimeError("reconstruction already running for this flight")

            job_id = f"recon-{time.strftime('%Y%m%d-%H%M%S')}-{uuid.uuid4().hex[:8]}"
            work_dir = self.work_root / job_id
            job = ReconstructionJob(
                id=job_id,
                flight_id=flight_id,
                source_record_id=str(frame_record["id"]),
                pose_record_id=str(pose_record["id"]) if pose_record else None,
                work_dir=work_dir,
                dataset_dir=work_dir / "dataset",
                processed_dir=work_dir / "nerfstudio",
                output_dir=work_dir / "outputs",
                export_dir=work_dir / "splat",
                log_path=work_dir / "reconstruction.log",
                max_images=max_images,
                max_iterations=max_iterations or 30000,
                fps=fps,
            )
            self._jobs[job_id] = job
            self._latest_by_flight[flight_id] = job_id
            self._stop_events[job_id] = threading.Event()

        thread = threading.Thread(
            target=self._run,
            args=(job.id, frame_record, pose_record),
            name=f"reconstruction-{job.id}",
            daemon=True,
        )
        thread.start()
        return job

    def status(self, flight_id: str) -> dict[str, object] | None:
        with self._lock:
            job_id = self._latest_by_flight.get(flight_id)
            job = self._jobs.get(job_id or "")
            if job is None:
                return None
            return job.as_dict(log_tail=tail_text(job.log_path))

    def stop(self, flight_id: str) -> dict[str, object] | None:
        with self._lock:
            job_id = self._latest_by_flight.get(flight_id)
            job = self._jobs.get(job_id or "")
            if job is None:
                return None
            event = self._stop_events.get(job.id)
            process = self._processes.get(job.id)
            if job.active:
                job.state = "stopping"
                job.updated_at = time.time()
                if event is not None:
                    event.set()
                if process is not None and process.poll() is None:
                    process.terminate()
            return job.as_dict(log_tail=tail_text(job.log_path))

    def stop_all(self) -> None:
        with self._lock:
            flights = list(self._latest_by_flight)
        for flight_id in flights:
            self.stop(flight_id)

    def _run(
        self,
        job_id: str,
        frame_record: dict[str, object],
        pose_record: dict[str, object] | None,
    ) -> None:
        job = self._jobs[job_id]
        try:
            job.work_dir.mkdir(parents=True, exist_ok=True)
            self._set_job(job, state="running", stage="dataset")
            self._prepare_dataset(job, frame_record, pose_record, max_images=job.max_images, fps=job.fps)
            job.dataset_record_id = self.store.import_record(
                job.flight_id,
                "splat",
                "reconstruction-dataset",
                f"Scene dataset {job.id}",
                "application/vnd.nerfstudio.dataset",
                job.dataset_dir,
            )
            training_data_dir = job.dataset_dir

            tools = self.tools_status()
            if not tools["ready"]:
                missing = [name for name, path in dict(tools["tools"]).items() if name.startswith("ns") and not path]
                raise RuntimeError(f"missing reconstruction tool(s): {', '.join(missing)}")

            if not (job.dataset_dir / "transforms.json").is_file():
                self._set_job(job, stage="colmap")
                self._run_command(job, [
                    self._require_tool("ns-process-data"),
                    "images",
                    "--data",
                    str(job.dataset_dir / "images"),
                    "--output-dir",
                    str(job.processed_dir),
                ])
                training_data_dir = job.processed_dir

            self._set_job(job, stage="train")
            self._run_command(job, [
                self._require_tool("ns-train"),
                "splatfacto",
                "--vis",
                "tensorboard",
                "--viewer.quit-on-train-completion",
                "True",
                "--max-num-iterations",
                str(job.max_iterations),
                "--steps-per-save",
                str(max(1, job.max_iterations)),
                "--save-only-latest-checkpoint",
                "True",
                "--data",
                str(training_data_dir),
                "--output-dir",
                str(job.output_dir),
            ])

            config_path = find_latest_config(job.output_dir)
            if config_path is None:
                raise RuntimeError("nerfstudio training completed without a config.yml")
            job.config_path = str(config_path)

            self._set_job(job, stage="export")
            self._run_command(job, [
                self._require_tool("ns-export"),
                "gaussian-splat",
                "--load-config",
                str(config_path),
                "--output-dir",
                str(job.export_dir),
            ])

            splat = find_splat_artifact(job.export_dir)
            if splat is None:
                raise RuntimeError("gaussian splat export produced no .ply, .splat, or .spz artifact")
            job.splat_record_id = self.store.import_record(
                job.flight_id,
                "splat",
                "gaussian-splat",
                f"Gaussian splat {job.id}",
                "model/vnd.gaussian-splat",
                job.export_dir,
            )
            self._set_job(job, state="completed", stage="completed", ended_at=time.time())
        except ReconstructionStopped:
            self._set_job(job, state="stopped", stage="stopped", ended_at=time.time())
        except BaseException as exc:
            self._append_log(job, f"\nERROR: {type(exc).__name__}: {exc}\n")
            self._set_job(job, state="failed", stage="failed", error=f"{type(exc).__name__}: {exc}", ended_at=time.time())
        finally:
            with self._lock:
                self._processes.pop(job.id, None)
                self._stop_events.pop(job.id, None)

    def _prepare_dataset(
        self,
        job: ReconstructionJob,
        frame_record: dict[str, object],
        pose_record: dict[str, object] | None,
        *,
        max_images: int | None,
        fps: float,
    ) -> None:
        frame_dir = self.store.record_path(str(frame_record["id"]))
        if frame_dir is None or not frame_dir.is_dir():
            raise FileNotFoundError("frame record blob missing")
        frames = sorted(frame_dir.glob("*.jpg"))
        if not frames:
            raise RuntimeError("frame record has no .jpg frames")
        if max_images is not None and max_images > 0 and len(frames) > max_images:
            frames = sample_evenly(frames, max_images)

        images_dir = job.dataset_dir / "images"
        images_dir.mkdir(parents=True, exist_ok=True)
        copied: list[tuple[int, Path]] = []
        for out_index, frame in enumerate(frames):
            source_index = parse_frame_index(frame.name, out_index)
            out = images_dir / f"frame_{out_index:06d}.jpg"
            shutil.copy2(frame, out)
            copied.append((source_index, out))

        pose_path = self.store.record_path(str(pose_record["id"])) if pose_record else None
        poses = load_pose_track(pose_path) if pose_path else []
        transforms = build_transforms_json(copied, poses)
        if transforms:
            (job.dataset_dir / "transforms.json").write_text(json.dumps(transforms, indent=2))

        manifest = {
            "flightId": job.flight_id,
            "jobId": job.id,
            "sourceRecordId": frame_record["id"],
            "poseRecordId": pose_record["id"] if pose_record else None,
            "frameCount": len(copied),
            "fps": fps,
            "poseSource": "visual-odometry" if poses else "none",
            "pipeline": "nerfstudio/splatfacto",
        }
        (job.dataset_dir / "manifest.json").write_text(json.dumps(manifest, indent=2))

    def _run_command(self, job: ReconstructionJob, command: list[str]) -> None:
        event = self._stop_events[job.id]
        self._append_log(job, f"\n$ {' '.join(command)}\n")
        with self._lock:
            job.current_command = command
            job.updated_at = time.time()
        process = subprocess.Popen(
            command,
            cwd=job.work_dir,
            env=self._tool_env(),
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
        )
        with self._lock:
            self._processes[job.id] = process
        assert process.stdout is not None
        for line in process.stdout:
            self._append_log(job, line)
            if event.is_set() and process.poll() is None:
                process.terminate()
        return_code = process.wait()
        with self._lock:
            job.return_code = return_code
            job.current_command = []
            job.updated_at = time.time()
        if event.is_set():
            raise ReconstructionStopped()
        if return_code != 0:
            raise RuntimeError(f"command failed with exit code {return_code}: {' '.join(command)}")

    def _append_log(self, job: ReconstructionJob, text: str) -> None:
        job.log_path.parent.mkdir(parents=True, exist_ok=True)
        with job.log_path.open("a") as handle:
            handle.write(text)

    def _tool_dirs(self) -> list[Path]:
        dirs: list[Path] = []
        virtual_env = os.environ.get("VIRTUAL_ENV")
        if virtual_env:
            dirs.append(Path(virtual_env) / "bin")
        dirs.append(Path(sys.executable).parent)
        resolved = Path(sys.executable).resolve().parent
        if resolved not in dirs:
            dirs.append(resolved)
        return dirs

    def _tool_env(self) -> dict[str, str]:
        env = os.environ.copy()
        extra = os.pathsep.join(str(path) for path in self._tool_dirs())
        env["PATH"] = f"{extra}{os.pathsep}{env.get('PATH', '')}" if extra else env.get("PATH", "")
        # Nerfstudio checkpoints are produced locally by the training subprocess.
        # PyTorch 2.6+ defaults torch.load() to weights_only=True, which breaks
        # Nerfstudio's exporter unless trusted local checkpoints are allowed.
        env.setdefault("TORCH_FORCE_NO_WEIGHTS_ONLY_LOAD", "1")
        return env

    def _which_tool(self, name: str) -> str | None:
        for directory in self._tool_dirs():
            candidate = directory / name
            if candidate.is_file() and os.access(candidate, os.X_OK):
                return str(candidate)
        return shutil.which(name)

    def _require_tool(self, name: str) -> str:
        path = self._which_tool(name)
        if path is None:
            raise RuntimeError(f"missing reconstruction tool: {name}")
        return path

    def _set_job(
        self,
        job: ReconstructionJob,
        *,
        state: str | None = None,
        stage: str | None = None,
        error: str | None = None,
        ended_at: float | None = None,
    ) -> None:
        with self._lock:
            if state is not None:
                job.state = state
            if stage is not None:
                job.stage = stage
            if error is not None:
                job.error = error
            if ended_at is not None:
                job.ended_at = ended_at
            job.updated_at = time.time()


class ReconstructionStopped(RuntimeError):
    pass


def sample_evenly(frames: list[Path], count: int) -> list[Path]:
    if count >= len(frames):
        return frames
    if count <= 1:
        return [frames[0]]
    last = len(frames) - 1
    return [frames[round(i * last / (count - 1))] for i in range(count)]


def parse_frame_index(name: str, fallback: int) -> int:
    parts = name.split("_")
    if len(parts) >= 2 and parts[1].isdigit():
        return int(parts[1])
    stem = Path(name).stem
    digits = "".join(ch for ch in stem if ch.isdigit())
    return int(digits) if digits else fallback


def build_transforms_json(copied: list[tuple[int, Path]], poses: list[dict[str, object]]) -> dict[str, object] | None:
    if not copied:
        return None
    pose_by_frame = {int(p.get("frameIndex", -1)): p for p in poses}
    frames = []
    for source_index, path in copied:
        pose = pose_by_frame.get(source_index)
        if pose is None:
            continue
        frames.append({
            "file_path": f"images/{path.name}",
            "transform_matrix": pose_matrix(pose),
        })
    if not frames:
        return None

    width, height = image_size(copied[0][1])
    focal = 0.82 * max(width, height)
    return {
        "camera_model": "OPENCV",
        "fl_x": focal,
        "fl_y": focal,
        "cx": width / 2.0,
        "cy": height / 2.0,
        "w": width,
        "h": height,
        "frames": frames,
    }


def pose_matrix(pose: dict[str, object]) -> list[list[float]]:
    qw = float(pose.get("qw", 1.0))
    qx = float(pose.get("qx", 0.0))
    qy = float(pose.get("qy", 0.0))
    qz = float(pose.get("qz", 0.0))
    norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz) or 1.0
    qw, qx, qy, qz = qw / norm, qx / norm, qy / norm, qz / norm
    r00 = 1.0 - 2.0 * (qy * qy + qz * qz)
    r01 = 2.0 * (qx * qy - qz * qw)
    r02 = 2.0 * (qx * qz + qy * qw)
    r10 = 2.0 * (qx * qy + qz * qw)
    r11 = 1.0 - 2.0 * (qx * qx + qz * qz)
    r12 = 2.0 * (qy * qz - qx * qw)
    r20 = 2.0 * (qx * qz - qy * qw)
    r21 = 2.0 * (qy * qz + qx * qw)
    r22 = 1.0 - 2.0 * (qx * qx + qy * qy)
    return [
        [r00, r01, r02, float(pose.get("x", 0.0))],
        [r10, r11, r12, float(pose.get("y", 0.0))],
        [r20, r21, r22, float(pose.get("z", 0.0))],
        [0.0, 0.0, 0.0, 1.0],
    ]


def image_size(path: Path) -> tuple[int, int]:
    with Image.open(path) as image:
        return image.size


def find_latest_config(root: Path) -> Path | None:
    configs = sorted(root.rglob("config.yml"), key=lambda p: p.stat().st_mtime)
    return configs[-1] if configs else None


def find_splat_artifact(root: Path) -> Path | None:
    if root.is_file() and root.suffix.lower() in SPLAT_EXTENSIONS:
        return root
    if not root.is_dir():
        return None
    candidates = [p for p in root.rglob("*") if p.is_file() and p.suffix.lower() in SPLAT_EXTENSIONS]
    if not candidates:
        return None
    priority = {".splat": 0, ".spz": 1, ".ply": 2}
    return sorted(candidates, key=lambda p: (priority.get(p.suffix.lower(), 99), p.stat().st_size))[0]


def tail_text(path: Path, *, max_bytes: int = 4096) -> str:
    if not path.is_file():
        return ""
    size = path.stat().st_size
    with path.open("rb") as handle:
        if size > max_bytes:
            handle.seek(size - max_bytes)
        return handle.read().decode(errors="replace")
