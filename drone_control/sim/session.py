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

import numpy as np
import torch

from .cloth import ClothFlag
from .dynamics import QuadParams
from .env import EnvConfig, SwarmEnv
from .expert import ExpertController
from .flow import AIR_DENSITY, FlowField, aero_accel
from .particles import ParticleField
from .render import CameraConfig, CameraRenderer
from .rigid import RigidWorld
from .scenes import Scene, build_scene


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
    # Airflow coupling: aerodynamic frontal area (m^2) + drag coeff used to turn
    # the local wind into a force on each (very light) drone. F = 0.5*rho*Cd*A*|v|v.
    drag_area: float = 0.006
    drag_cd: float = 1.1
    wind_enabled: bool = True


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
        # Airflow + soft bodies.
        self._scene: Scene | None = None
        self._flow: FlowField | None = None
        self._flags: list[ClothFlag] = []
        self._flag_render: list[tuple] = []  # (grid[ny,nx,3], color)
        self._rigid: RigidWorld | None = None
        self._rigid_render: list[tuple] = []  # (corners, color, label) face quads
        self._particles: ParticleField | None = None
        self._particle_render: tuple | None = None  # (pos, rgba, size)
        self._c_aero = 0.0
        self._wind_ambient = (0.0, 0.0, 0.0)

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

            # Airflow field + cloth soft bodies from the scene plan.
            self._scene = build_scene(cfg.scene)
            self._flow = FlowField(self._scene.flows, seed=cfg.seed) if cfg.wind_enabled else None
            self._flags = [
                ClothFlag(spec.mast_top, spec.direction, width=spec.width, height=spec.height,
                          color=spec.color, label=spec.label)
                for spec in self._scene.flags
            ]
            self._rigid = RigidWorld(self._scene.rigids, seed=cfg.seed) if self._scene.rigids else None
            self._rigid_render = []
            self._particles = ParticleField(self._scene.particles, seed=cfg.seed) if self._scene.particles else None
            self._particle_render = None
            self._c_aero = 0.5 * AIR_DENSITY * cfg.drag_cd * cfg.drag_area
            self._wind_ambient = (0.0, 0.0, 0.0)

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
            t = self._step * self.params.dt
            ext_accel = self._airflow_accel(env, t)
            obs, _reward, _done = env.step(command, ext_accel=ext_accel)
            # Advance soft bodies (cloth), rigid bodies, and particles against the
            # wind at the post-step instant.
            self._advance_bodies(env, t + self.params.dt)

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

    # ----------------------------------------------------------------- airflow

    def _object_kinematics(self, t: float):
        """(pos[M,3], vel[M,3]) of moving scene objects at time ``t`` (or None)."""
        specs = self._scene.dynamics if self._scene is not None else []
        if not specs:
            return None
        h = 0.05
        pos = np.array([s.position_at(t) for s in specs], dtype=float)
        nxt = np.array([s.position_at(t + h) for s in specs], dtype=float)
        prv = np.array([s.position_at(max(0.0, t - h)) for s in specs], dtype=float)
        vel = (nxt - prv) / (2 * h)
        return pos, vel

    def _rotor_sources(self, env: SwarmEnv):
        pos = env.state.pos.detach().cpu().numpy()
        if env.last_command is not None and env.last_command.numel():
            thr = np.clip(env.last_command[:, 2].detach().cpu().numpy(), 0.0, 1.0)
        else:
            thr = np.full(pos.shape[0], 0.5)
        return pos, thr

    def _airflow_accel(self, env: SwarmEnv, t: float) -> torch.Tensor | None:
        """World-frame [K,3] aero acceleration on the drones from the flow field."""
        if self._flow is None:
            return None
        pos = env.state.pos.detach().cpu().numpy()
        vel = env.state.vel.detach().cpu().numpy()
        wind = self._flow.sample(
            pos, t, objects=self._object_kinematics(t), rotors=self._rotor_sources(env)
        )
        self._wind_ambient = tuple(float(v) for v in self._flow.ambient(t, z=2.0))
        acc = aero_accel(wind, vel, self.params.mass, self._c_aero)
        return torch.from_numpy(acc.astype(np.float32))

    def _advance_bodies(self, env: SwarmEnv, t: float) -> None:
        """Step cloth, rigid bodies, and particles against the shared wind field."""
        if self._flow is None and not (self._flags or self._rigid or self._particles):
            return
        objects = self._object_kinematics(t)
        rotors = self._rotor_sources(env)

        def wind_at(points):
            if self._flow is None:
                return np.zeros_like(points)
            return self._flow.sample(points, t, objects=objects, rotors=rotors)

        flag_render = []
        for flag in self._flags:
            flag.step(self.params.dt, wind_at(flag.pos))
            flag_render.append((flag.grid(), flag.color))

        rigid_render: list = []
        if self._rigid is not None:
            self._rigid.step(self.params.dt, wind_at)
            rigid_render = self._rigid.face_quads()

        particle_render = None
        if self._particles is not None:
            self._particles.step(self.params.dt, wind_at)
            particle_render = self._particles.points()

        with self._lock:
            self._flag_render = flag_render
            self._rigid_render = rigid_render
            self._particle_render = particle_render

    def _record(self, env: SwarmEnv) -> None:
        pos = env.state.pos.detach().cpu()
        quat = env.state.quat.detach().cpu()
        goals = env.goals.detach().cpu()
        frames = None
        if self._renderer is not None and self._step % max(1, self.config.render_every) == 0:
            with self._lock:
                flag_render = list(self._flag_render)
                rigid_render = list(self._rigid_render)
                particle_render = self._particle_render
            frames = self._renderer.render(
                pos.numpy(), quat.numpy(), goals.numpy(), t=self._step * self.params.dt,
                flags=flag_render, wind=self._wind_ambient,
                rigids=rigid_render, particles=particle_render,
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
            wind = self._wind_ambient
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
                "wind": {
                    "ambient": [round(v, 3) for v in wind],
                    "speed": round(float(np.linalg.norm(wind)), 3),
                    "sources": len(self._scene.flows) if self._scene is not None else 0,
                    "flags": len(self._flags),
                    "rigidBodies": (self._rigid.centers().shape[0] if self._rigid is not None else 0),
                    "particles": (len(self._particle_render[0]) if self._particle_render is not None else 0),
                },
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
