"""
Live, service-hosted simulation session.

Runs a ``SwarmEnv`` under the analytic expert (or, later, a learned policy) in a
background thread so the desktop UI can watch many drones fly the sim in real
time: it keeps each drone's trajectory and latest synthetic forward-camera
frame, and resamples a drone's goal when it arrives so the swarm keeps moving.

This is the sim counterpart to the live runtime: the UI consumes the same shape
of data (per-drone poses + per-drone JPEG frames) whether it points at the sim
session or the real runtime.
"""

from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any

import torch

from .dynamics import QuadParams
from .env import EnvConfig, SwarmEnv
from .expert import ExpertController
from .render import CameraConfig, CameraRenderer


# Distinct-ish hues (HSL-ish) for up to a handful of drones; cycles beyond that.
_PALETTE = [
    "#7fd1ff", "#ffd35a", "#8be0a0", "#f0a39d", "#c9a3ff",
    "#ff9f5a", "#5ad8d8", "#e069c8", "#9ad84a", "#6a9bff",
]


@dataclass(slots=True)
class SimSessionConfig:
    num_drones: int = 4
    task: str = "goto"
    scene: str = "open_field"       # named scene plan rendered by the synthetic camera
    camera_noise: object = "medium" # OV2640-style sensor noise: off|low|medium|high or dict
    rate_hz: float = 15.0
    max_speed: bool = False         # run the sim as fast as the CPU allows (ignore rate_hz pacing)
    max_trajectory: int = 400
    render: bool = True
    render_every: int = 2          # render frames every N ticks (CPU bound)
    arrival_radius: float = 0.6     # resample a drone's goal once it arrives
    image_size: int = 128
    seed: int = 0


@dataclass(slots=True)
class _DroneTrack:
    poses: deque = field(default_factory=lambda: deque(maxlen=400))
    frame: bytes | None = None


class SimSession:
    def __init__(self, config: SimSessionConfig | None = None) -> None:
        self.config = config or SimSessionConfig()
        self.params = QuadParams()
        self._lock = threading.RLock()
        self._env: SwarmEnv | None = None
        self._expert: ExpertController | None = None
        self._renderer: CameraRenderer | None = None
        self._tracks: list[_DroneTrack] = []
        self._thread: threading.Thread | None = None
        self._running = False
        self._step = 0

    @property
    def running(self) -> bool:
        return self._running

    def start(self, config: SimSessionConfig | None = None) -> dict[str, Any]:
        with self._lock:
            if self._running:
                self.stop()
            if config is not None:
                self.config = config
            cfg = self.config
            self._env = SwarmEnv(
                EnvConfig(
                    num_envs=1,
                    num_drones=cfg.num_drones,
                    task=cfg.task,
                    max_steps=10_000_000,
                    seed=cfg.seed,
                ),
                params=self.params,
            )
            self._expert = ExpertController(self.params)
            self._renderer = (
                CameraRenderer(
                    CameraConfig(width=cfg.image_size, height=int(cfg.image_size * 0.75)),
                    scene=cfg.scene,
                    noise=cfg.camera_noise,
                )
                if cfg.render
                else None
            )
            self._tracks = [_DroneTrack(poses=deque(maxlen=cfg.max_trajectory)) for _ in range(cfg.num_drones)]
            self._env.reset()
            self._step = 0
            self._running = True
            self._thread = threading.Thread(target=self._loop, name="sim-session", daemon=True)
            self._thread.start()
        return self.status()

    def stop(self) -> dict[str, Any]:
        with self._lock:
            self._running = False
            thread = self._thread
            self._thread = None
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2.0)
        return self.status()

    def _loop(self) -> None:
        interval = 1.0 / max(1.0, self.config.rate_hz)
        while True:
            with self._lock:
                if not self._running or self._env is None or self._expert is None:
                    return
                env = self._env
                expert = self._expert
            started = time.monotonic()

            obs = env._observe()
            command = expert.command(obs)
            obs, _reward, _done = env.step(command)

            # Resample arrived drones' goals so the swarm keeps flying.
            dist = obs.goal_rel.norm(dim=1)
            arrived = dist < self.config.arrival_radius
            if bool(arrived.any()):
                new_goals = env.task.sample_goals(env.k, env.device)
                env.goals[arrived] = new_goals[arrived]

            self._record(env)
            self._step += 1

            if self.config.max_speed:
                continue
            elapsed = time.monotonic() - started
            time.sleep(max(0.0, interval - elapsed))

    def _record(self, env: SwarmEnv) -> None:
        pos = env.state.pos.detach().cpu()
        quat = env.state.quat.detach().cpu()
        goals = env.goals.detach().cpu()
        frames = None
        if self._renderer is not None and self._step % max(1, self.config.render_every) == 0:
            frames = self._renderer.render(
                pos.numpy(), quat.numpy(), goals.numpy(), t=self._step * self.params.dt
            )
        with self._lock:
            for i, track in enumerate(self._tracks):
                track.poses.append(
                    {
                        "x": float(pos[i, 0]), "y": float(pos[i, 1]), "z": float(pos[i, 2]),
                        "qw": float(quat[i, 0]), "qx": float(quat[i, 1]),
                        "qy": float(quat[i, 2]), "qz": float(quat[i, 3]),
                    }
                )
                if frames is not None:
                    track.frame = frames[i]

    def status(self) -> dict[str, Any]:
        with self._lock:
            env = self._env
            drones = []
            if env is not None:
                pos = env.state.pos.detach().cpu()
                goals = env.goals.detach().cpu()
                dist = (env.goals - env.state.pos).norm(dim=1).detach().cpu()
                for i in range(self.config.num_drones):
                    drones.append(
                        {
                            "droneId": f"sim-{i}",
                            "color": _PALETTE[i % len(_PALETTE)],
                            "position": [float(v) for v in pos[i]],
                            "goal": [float(v) for v in goals[i]],
                            "distance": float(dist[i]),
                            "hasFrame": self._tracks[i].frame is not None if i < len(self._tracks) else False,
                        }
                    )
            return {
                "running": self._running,
                "task": self.config.task,
                "scene": self.config.scene,
                "cameraNoise": self.config.camera_noise,
                "numDrones": self.config.num_drones,
                "rateHz": self.config.rate_hz,
                "step": self._step,
                "render": self.config.render,
                "drones": drones,
            }

    def trajectories(self) -> dict[str, Any]:
        with self._lock:
            env = self._env
            out = []
            for i, track in enumerate(self._tracks):
                goal = None
                if env is not None:
                    goal = [float(v) for v in env.goals.detach().cpu()[i]]
                out.append(
                    {
                        "droneId": f"sim-{i}",
                        "color": _PALETTE[i % len(_PALETTE)],
                        "goal": goal,
                        "poses": list(track.poses),
                    }
                )
            return {"running": self._running, "drones": out}

    def frame(self, index: int) -> bytes | None:
        with self._lock:
            if 0 <= index < len(self._tracks):
                return self._tracks[index].frame
            return None

    def sim_time(self) -> float:
        """Elapsed simulated time (s) — drives moving scene objects."""
        with self._lock:
            return self._step * self.params.dt

    def set_max_speed(self, enabled: bool) -> None:
        """Toggle realtime pacing vs. run-as-fast-as-possible, live."""
        with self._lock:
            self.config.max_speed = bool(enabled)

    def latest_pose(self, index: int) -> dict[str, Any] | None:
        with self._lock:
            if 0 <= index < len(self._tracks) and self._tracks[index].poses:
                return dict(self._tracks[index].poses[-1])
            return None
