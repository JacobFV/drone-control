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

from ..cameras import CameraModel, get_camera
from .dynamics import QuadParams
from .env import EnvConfig, SwarmEnv
from .expert import ExpertController
from .cloth_instancer import expand_cloth_instances
from .flow import AIR_DENSITY, FlowField, aero_accel
from .particles import ParticleField
from .physics import PyBulletWorld
from .render import CameraConfig, CameraRenderer
from .scenes import Scene, build_scene
from .smoke import VolumetricField


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
    camera_model: str = "ov2640"    # OV sensor: sets render resolution + lens FOV + realistic fps
    rate_hz: float = 30.0           # physics/sim step rate (Hz); camera fps comes from the model
    max_speed: bool = False         # run the sim as fast as the CPU allows (ignore rate_hz pacing)
    max_trajectory: int = 400
    render: bool = True
    arrival_radius: float = 0.6     # resample a drone's goal once it arrives
    image_size: int | None = None   # override the sensor's streamed width (None = use the model)
    seed: int = 0
    # Airflow coupling: aerodynamic frontal area (m^2) + drag coeff used to turn
    # the local wind into a force on each (very light) drone. F = 0.5*rho*Cd*A*|v|v.
    drag_area: float = 0.006
    drag_cd: float = 1.1
    wind_enabled: bool = True
    # Drone collision proxy radius (m) for PyBullet contact resolution.
    drone_radius: float = 0.13
    collision_restitution: float = 0.25


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
        self._omni_renderer: CameraRenderer | None = None
        self._camera: CameraModel = get_camera(self.config.camera_model)
        self._tracks: list[_DroneTrack] = []
        self._thread: threading.Thread | None = None
        self._running = False
        self._paused = False           # frozen but alive: loop keeps the thread, stops stepping
        self._step = 0
        self._next_frame_t = 0.0       # sim-time of the next camera frame (fps cadence)
        self._frame_rng = np.random.default_rng(0)  # link-jitter RNG
        # Per-drone control: latest stick command (E99 byte form) + recent history,
        # and the set of drones individually e-stopped (held in place) by the operator.
        self._commands: np.ndarray | None = None     # [K,4] uint8 roll/pitch/throttle/yaw
        self._cmd_history: list[deque] = []          # per-drone deque of {t,roll,pitch,throttle,yaw}
        self._cmd_next_t = 0.0                        # sim-time of next history sample
        self._frozen: set[int] = set()                # drone indices held by e-stop
        self._frozen_pose: dict[int, tuple] = {}      # i -> (pos, quat) snapshot to hold
        # Airflow + PyBullet physics world (rigid + deformable cloth) + atmospherics.
        self._scene: Scene | None = None
        self._flow: FlowField | None = None
        self._physics: PyBulletWorld | None = None
        self._rigid_render: list[tuple] = []  # (corners, color, label) face quads
        self._cloth_meshes: list[tuple] = []  # (verts, faces, color, label)
        self._particles: ParticleField | None = None
        self._particle_render: tuple | None = None  # (pos, rgba, size)
        self._smoke: VolumetricField | None = None
        self._smoke_render: dict | None = None      # puffs() arrays for the renderer
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
            # The drone carries a specific OV sensor: render at its streamed
            # resolution + lens FOV so perception sees exactly what the ESP32
            # bridge would deliver (not an idealised square thumbnail).
            cam = get_camera(cfg.camera_model)
            cam_w = int(cfg.image_size) if cfg.image_size else cam.width
            cam_h = int(round(cam_w / cam.aspect))
            self._camera = cam
            self._omni_renderer = None   # rebuilt lazily for the active scene
            self._renderer = (
                CameraRenderer(
                    CameraConfig(width=cam_w, height=cam_h, fov_deg=cam.hfov_deg),
                    scene=cfg.scene,
                    noise=cfg.camera_noise,
                )
                if cfg.render
                else None
            )
            self._tracks = [_DroneTrack(poses=deque(maxlen=cfg.max_trajectory)) for _ in range(cfg.num_drones)]

            # Airflow field + PyBullet physics world + atmospherics from the plan.
            scene = self._scene = build_scene(cfg.scene)
            self._flow = FlowField(scene.flows, seed=cfg.seed) if cfg.wind_enabled else None

            # Cloth panels authored directly + lite-style flags are unified into
            # ClothSpec; the static scene boxes become collision geometry so the
            # drones can crash into walls/shelves/furniture.
            static_boxes = [(b.center, b.size) for b in scene.boxes]
            # Standalone cloths + one simulated master per instance group.
            cloths = list(scene.cloths) + [g.master for g in scene.cloth_groups]
            self._physics = PyBulletWorld(scene.rigids, cloths, dt=self.params.dt,
                                          static_boxes=static_boxes, seed=cfg.seed)
            self._physics.set_drones(cfg.num_drones, radius=cfg.drone_radius)
            self._rigid_render = []
            self._cloth_meshes = []

            self._particles = ParticleField(scene.particles, seed=cfg.seed) if scene.particles else None
            self._particle_render = None
            self._smoke = (
                VolumetricField(scene.smokes, scene.fires, seed=cfg.seed)
                if (scene.smokes or scene.fires) else None
            )
            self._smoke_render = None
            self._c_aero = 0.5 * AIR_DENSITY * cfg.drag_cd * cfg.drag_area
            self._wind_ambient = (0.0, 0.0, 0.0)

            self._env.reset()
            self._step = 0
            self._next_frame_t = 0.0
            self._paused = False
            self._commands = np.full((cfg.num_drones, 4), 128, dtype=np.uint8)
            self._cmd_history = [deque(maxlen=120) for _ in range(cfg.num_drones)]
            self._cmd_next_t = 0.0
            self._frozen = set()
            self._frozen_pose = {}
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
        # Release the PyBullet client (if any) once the loop has stopped.
        physics = self._physics
        self._physics = None
        if physics is not None:
            try:
                physics.close()
            except Exception:
                pass
        return self.status()

    def _loop(self) -> None:
        interval = 1.0 / max(1.0, self.config.rate_hz)
        while True:
            with self._lock:
                if not self._running or self._env is None or self._expert is None:
                    return
                env = self._env
                expert = self._expert
                paused = self._paused
            if paused:
                # Hold the whole sim: keep the thread + state alive, just don't step.
                time.sleep(0.05)
                continue
            started = time.monotonic()

            obs = env._observe()
            command = expert.command(obs)
            t = self._step * self.params.dt
            self._capture_commands(command, t)
            ext_accel = self._airflow_accel(env, t)
            obs, _reward, _done = env.step(command, ext_accel=ext_accel)
            self._apply_freeze(env)
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

    # ------------------------------------------------------- per-drone control

    @staticmethod
    def _to_bytes(command: "np.ndarray") -> np.ndarray:
        """Expert command (roll/pitch/yaw in [-1,1], throttle in [0,1]) -> E99
        stick bytes (0-255, neutral 128), matching the real DroneAction packet."""
        c = np.asarray(command, dtype=float)
        roll = np.clip(np.round(128 + c[:, 0] * 127), 0, 255)
        pitch = np.clip(np.round(128 + c[:, 1] * 127), 0, 255)
        throttle = np.clip(np.round(c[:, 2] * 255), 0, 255)
        yaw = np.clip(np.round(128 + c[:, 3] * 127), 0, 255)
        return np.stack([roll, pitch, throttle, yaw], axis=1).astype(np.uint8)

    def _capture_commands(self, command, t: float) -> None:
        """Record the latest per-drone stick command (always) and a throttled
        history sample (~4 Hz of sim time) for the Drones panel."""
        bytes_cmd = self._to_bytes(command.detach().cpu().numpy())
        with self._lock:
            self._commands = bytes_cmd
            sample = t >= self._cmd_next_t
            if sample:
                self._cmd_next_t = t + 0.25
            for i in range(bytes_cmd.shape[0]):
                if i in self._frozen:
                    continue
                if sample and i < len(self._cmd_history):
                    r, p, th, y = (int(v) for v in bytes_cmd[i])
                    self._cmd_history[i].append(
                        {"t": round(t, 2), "roll": r, "pitch": p, "throttle": th, "yaw": y}
                    )

    def _apply_freeze(self, env: SwarmEnv) -> None:
        """Hold e-stopped drones exactly in place (zero velocity) post-step."""
        if not self._frozen:
            return
        with self._lock:
            frozen = list(self._frozen)
            poses = dict(self._frozen_pose)
        for i in frozen:
            saved = poses.get(i)
            if saved is None or i >= env.state.pos.shape[0]:
                continue
            pos, quat = saved
            env.state.pos[i] = torch.as_tensor(pos, dtype=env.state.pos.dtype, device=env.state.pos.device)
            env.state.quat[i] = torch.as_tensor(quat, dtype=env.state.quat.dtype, device=env.state.quat.device)
            env.state.vel[i] = 0.0
            env.state.omega[i] = 0.0

    def pause(self) -> dict[str, Any]:
        with self._lock:
            self._paused = True
        return self.status()

    def resume(self) -> dict[str, Any]:
        with self._lock:
            self._paused = False
        return self.status()

    def estop(self, index: int) -> None:
        """E-stop a single drone: hold it in place until released."""
        with self._lock:
            env = self._env
            if env is None or not (0 <= index < env.state.pos.shape[0]):
                return
            pos = env.state.pos[index].detach().cpu().numpy().copy()
            quat = env.state.quat[index].detach().cpu().numpy().copy()
            self._frozen.add(index)
            self._frozen_pose[index] = (pos, quat)
            if index < len(self._cmd_history):
                self._cmd_history[index].append({"t": round(self._step * self.params.dt, 2), "event": "e-stop"})

    def release(self, index: int) -> None:
        """Release a single drone from e-stop (resume its flight)."""
        with self._lock:
            self._frozen.discard(index)
            self._frozen_pose.pop(index, None)
            if index < len(self._cmd_history):
                self._cmd_history[index].append({"t": round(self._step * self.params.dt, 2), "event": "resume"})

    def drone_commands(self, index: int) -> list[dict[str, Any]]:
        with self._lock:
            if 0 <= index < len(self._cmd_history):
                return list(self._cmd_history[index])
            return []

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
        """Step the PyBullet world, atmospherics, and resolve drone collisions
        against the shared wind field at the post-step instant."""
        objects = self._object_kinematics(t)
        rotors = self._rotor_sources(env)

        def wind_at(points):
            if self._flow is None:
                return np.zeros_like(np.atleast_2d(points))
            return self._flow.sample(points, t, objects=objects, rotors=rotors)

        self._wind_ambient = (
            tuple(float(v) for v in self._flow.ambient(t, z=2.0)) if self._flow else (0.0, 0.0, 0.0)
        )

        rigid_render: list = []
        cloth_meshes: list = []
        if self._physics is not None:
            self._physics.update_drones(env.state.pos.detach().cpu().numpy())
            self._physics.step(self.params.dt, wind_at)
            rigid_render = self._physics.rigid_faces()
            cloth_meshes = self._expand_cloth(self._physics.cloth_meshes(), t, wind_at)
            self._resolve_collisions(env)

        particle_render = None
        if self._particles is not None:
            self._particles.step(self.params.dt, wind_at)
            particle_render = self._particles.points()

        smoke_render = None
        if self._smoke is not None:
            self._smoke.step(self.params.dt, wind_at)
            smoke_render = self._smoke.puffs()

        with self._lock:
            self._rigid_render = rigid_render
            self._cloth_meshes = cloth_meshes
            self._particle_render = particle_render
            self._smoke_render = smoke_render

    def _expand_cloth(self, meshes: list, t: float, wind_at) -> list:
        """Split simulated cloth into standalone panels (rendered directly) and
        instance-group masters (expanded into many swaying instances)."""
        if self._scene is None or not self._scene.cloth_groups:
            return meshes
        masters: dict = {}
        render: list = []
        for verts, faces, color, label in meshes:
            if label.startswith("__master__"):
                masters[label] = (np.asarray(verts, dtype=float), faces)
            else:
                render.append((verts, faces, color, label))
        for group in self._scene.cloth_groups:
            entry = masters.get(group.master.label)
            if entry is None:
                continue
            mv, faces = entry
            render.extend(
                expand_cloth_instances(mv, group.master.anchor, faces, group.placements(), t, wind_at)
            )
        return render

    def _resolve_collisions(self, env: SwarmEnv) -> None:
        """Push drones out of walls/bodies and kill the inward velocity component
        (with a little restitution) — drones physically crash into the world."""
        contacts = self._physics.drone_collisions(margin=0.02)
        if not contacts:
            return
        pos = env.state.pos
        vel = env.state.vel
        restitution = self.config.collision_restitution
        for k, normal, depth in contacts:
            if k >= pos.shape[0]:
                continue
            n = torch.tensor(normal, dtype=pos.dtype, device=pos.device)
            # Depenetrate along the outward normal.
            pos[k] = pos[k] + n * float(depth)
            # Remove the velocity component heading into the obstacle.
            vn = float(torch.dot(vel[k], n))
            if vn < 0.0:
                vel[k] = vel[k] - (1.0 + restitution) * vn * n

    def _record(self, env: SwarmEnv) -> None:
        pos = env.state.pos.detach().cpu()
        quat = env.state.quat.detach().cpu()
        goals = env.goals.detach().cpu()
        frames = None
        # Camera frames emit at the sensor's realistic ESP32-bridged fps (with
        # bursty link jitter) — NOT every physics tick. Poses still record every
        # step so trajectories stay smooth; only the image stream is throttled to
        # what the real link delivers.
        t = self._step * self.params.dt
        if self._renderer is not None and t >= self._next_frame_t:
            fps = max(1.0, self._camera.fps)
            jit = 1.0 + self._camera.jitter * float(self._frame_rng.uniform(-1.0, 1.0))
            self._next_frame_t = t + (1.0 / fps) * max(0.2, jit)
            with self._lock:
                rigid_render = list(self._rigid_render)
                cloth_meshes = list(self._cloth_meshes)
                particle_render = self._particle_render
                smoke_render = self._smoke_render
            frames = self._renderer.render(
                pos.numpy(), quat.numpy(), goals.numpy(), t=t,
                wind=self._wind_ambient, rigids=rigid_render, particles=particle_render,
                meshes=cloth_meshes, smoke=smoke_render,
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
                cmds = self._commands
                for i in range(self.config.num_drones):
                    command = None
                    if cmds is not None and i < cmds.shape[0]:
                        r, p, th, y = (int(v) for v in cmds[i])
                        command = {"roll": r, "pitch": p, "throttle": th, "yaw": y}
                    drones.append(
                        {
                            "droneId": f"sim-{i}",
                            "color": _PALETTE[i % len(_PALETTE)],
                            "position": [float(v) for v in pos[i]],
                            "goal": [float(v) for v in goals[i]],
                            "distance": float(dist[i]),
                            "hasFrame": self._tracks[i].frame is not None if i < len(self._tracks) else False,
                            "command": command,
                            "frozen": i in self._frozen,
                        }
                    )
            wind = self._wind_ambient
            return {
                "running": self._running,
                "paused": self._paused,
                "task": self.config.task,
                "scene": self.config.scene,
                "cameraNoise": self.config.camera_noise,
                "camera": {
                    "id": self._camera.id, "name": self._camera.name,
                    "width": self._renderer.config.width if self._renderer else self._camera.width,
                    "height": self._renderer.config.height if self._renderer else self._camera.height,
                    "fps": self._camera.fps, "hfovDeg": self._camera.hfov_deg,
                    "sensor": self._camera.sensor,
                },
                "numDrones": self.config.num_drones,
                "rateHz": self.config.rate_hz,
                "step": self._step,
                "render": self.config.render,
                "drones": drones,
                "wind": {
                    "ambient": [round(v, 3) for v in wind],
                    "speed": round(float(np.linalg.norm(wind)), 3),
                    "sources": len(self._scene.flows) if self._scene is not None else 0,
                    "cloth": len(self._cloth_meshes),
                    "rigidBodies": len(self._rigid_render) // 6,
                    "particles": (len(self._particle_render[0]) if self._particle_render is not None else 0),
                    "smoke": (len(self._smoke_render["pos"]) if self._smoke_render is not None else 0),
                },
            }

    def omniscient_frame(self, view: dict[str, list[float]] | None = None) -> bytes | None:
        """Render the whole sim world from a slowly-orbiting god's-eye camera —
        the omniscient ground-truth view (all drones, goals, scene, bodies)."""
        with self._lock:
            env = self._env
            if env is None or self._scene is None:
                return None
            pos = env.state.pos.detach().cpu().numpy()
            quat = env.state.quat.detach().cpu().numpy()
            goals = env.goals.detach().cpu().numpy()
            rigid_render = list(self._rigid_render)
            cloth_meshes = list(self._cloth_meshes)
            particle_render = self._particle_render
            smoke_render = self._smoke_render
            wind = self._wind_ambient
            t = self._step * self.params.dt
            if self._omni_renderer is None:
                self._omni_renderer = CameraRenderer(
                    CameraConfig(width=480, height=360, fov_deg=72.0),
                    scene=self._scene, noise=None,
                )
            renderer = self._omni_renderer
        colors = [
            tuple(int(_PALETTE[i % len(_PALETTE)].lstrip("#")[k:k + 2], 16) for k in (0, 2, 4))
            for i in range(pos.shape[0])
        ]
        centroid = pos.mean(axis=0) if pos.shape[0] else np.zeros(3)
        spread = float(np.linalg.norm(pos - centroid, axis=1).max()) if pos.shape[0] > 1 else 0.0
        # Pull back far enough to frame the scene geometry (walls/shelves), not
        # just the swarm — render's grid_range bounds the world at ~14 m.
        world_extent = float(getattr(self._omni_renderer.config, "grid_range", 14.0))
        radius = max(world_extent * 1.4, spread * 1.8 + 8.0)
        if view and "eye" in view and "target" in view:
            cam_pos = np.asarray(view["eye"], dtype=float)
            target = np.asarray(view["target"], dtype=float)
        else:
            ang = 0.1 * t
            cam_pos = centroid + np.array([radius * np.cos(ang), radius * np.sin(ang), radius * 0.55])
            target = centroid + np.array([0.0, 0.0, 1.0])
        return renderer.render_omniscient(
            cam_pos, target, pos, quat, goals, colors=colors, t=t, wind=wind,
            rigids=rigid_render, particles=particle_render, meshes=cloth_meshes, smoke=smoke_render,
        )

    def camera_intrinsics(self) -> dict[str, float]:
        """Pinhole intrinsics matching the active renderer (lens FOV + resolution).

        This is camera *calibration* — the one sim-specific fact perception is
        allowed — so it rides along in ``camera_pose`` and the depth front-end
        uses the true focal length for the chosen OV lens instead of a guess.
        """
        cam = self._camera
        w = self._renderer.config.width if self._renderer else cam.width
        h = self._renderer.config.height if self._renderer else cam.height
        fx = (w / 2.0) / float(np.tan(np.deg2rad(cam.hfov_deg) / 2.0))
        return {"fx": fx, "fy": fx, "cx": w / 2.0, "cy": h / 2.0, "width": w, "height": h}

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
