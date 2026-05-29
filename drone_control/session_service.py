"""
SessionService — owns the single active flight session.

A flight session is shared by all drones in one environment (sim or real). The
service starts the environment, then runs two background loops:

* a **perception loop** that pulls each drone's latest camera frame, runs
  screen-space segmentation, and fuses world-space objects via the drone pose;
* a **recorder loop** that (when recording) writes per-drone camera frames,
  pose tracks, and control signals to disk.

On stop, recorded streams + inferences (camera frames, pose tracks, control,
gaussian splat, screen/world segmentation) are imported into the store as
records under the session, matching the user's conceptual model:

    Environment → FlightSession (shared by drones) → records (experience + inferences)
"""

from __future__ import annotations

import hashlib
import json
import threading
import time
from pathlib import Path
from typing import Any

import numpy as np

from drone_control.environment.real_env import RealEnvironment
from drone_control.environment.sim_env import SimEnvironment
from drone_control.perception import live_splat
from drone_control.perception.depth import DepthEstimator, write_ply
from drone_control.perception.segmentation import ScreenDetection, Segmenter, rotmat_to_quat_xyzw
from drone_control.runtime.manager import RuntimeManager
from drone_control.sim.session import SimSessionConfig
from drone_control.store import ControlStationStore


def _fingerprint(data: bytes) -> str:
    head = data[:96]
    return f"{len(data)}:{hashlib.blake2b(head, digest_size=8).hexdigest()}"


class SessionService:
    def __init__(
        self,
        store: ControlStationStore,
        runtime: RuntimeManager,
        *,
        work_root: Path,
        export_root: Path,
        perception_hz: float = 5.0,
        recorder_hz: float = 12.0,
    ) -> None:
        self.store = store
        self.runtime = runtime
        self.work_root = work_root
        self.export_root = export_root
        self.perception_hz = perception_hz
        self.recorder_hz = recorder_hz

        self.segmenter = Segmenter()
        self.depth = DepthEstimator()
        self._depth_every = 2  # run depth every Nth perception tick (it is heavier)

        self._lock = threading.RLock()
        self._env: SimEnvironment | RealEnvironment | None = None
        self._splat: Any | None = None     # session-owned live splat (sim)
        self._seeded = False               # seed the splat from the depth cloud once
        self._session_id: str | None = None
        self._environment_id: str | None = None
        self._recording = False
        self._started_at = 0.0
        self._session_dir: Path | None = None
        self._frame_counts: dict[str, int] = {}
        self._frame_bytes: dict[str, int] = {}
        self._last_fp: dict[str, str] = {}

        self._stop_event = threading.Event()
        self._perception_thread: threading.Thread | None = None
        self._recorder_thread: threading.Thread | None = None

    # -------------------------------------------------------------- lifecycle

    def start(self, kind: str, name: str, options: dict[str, Any] | None = None) -> dict[str, Any]:
        options = options or {}
        with self._lock:
            if self._env is not None:
                raise RuntimeError("a session is already active; stop it first")

            kind = kind.lower().strip()
            if kind == "sim":
                env: SimEnvironment | RealEnvironment = SimEnvironment(
                    SimSessionConfig(
                        num_drones=int(options.get("numDrones") or 4),
                        task=str(options.get("task") or "goto"),
                        scene=str(options.get("scene") or "open_field"),
                        rate_hz=float(options.get("rateHz") or 15.0),
                        max_speed=bool(options.get("maxSpeed", False)),
                        render=bool(options.get("render", True)),
                    )
                )
                environment_id = str(options.get("environmentId") or "env-sim-default")
                if not self.store.environment_exists(environment_id):
                    self.store.create_environment("Swarm simulator", "sim", environment_id=environment_id)
            elif kind == "real":
                env = RealEnvironment(self.runtime, world_model=bool(options.get("worldModel", True)))
                environment_id = str(options.get("environmentId") or "env-real-default")
                if not self.store.environment_exists(environment_id):
                    self.store.create_environment("Live (real drones)", "real", environment_id=environment_id)
            else:
                raise ValueError("kind must be 'sim' or 'real'")

            env.start()
            drones = env.drone_ids()
            session = self.store.create_session(
                environment_id,
                name or f"{kind} session {time.strftime('%H:%M:%S')}",
                drones,
                metadata={"kind": kind, "options": options},
            )

            self.segmenter.reset()
            self.depth.reset()
            self._seeded = False
            # The sim owns its own splat engine (it has exact poses); the real
            # runtime builds its splat through its own ingestion path.
            self._splat = None
            if isinstance(env, SimEnvironment) and live_splat.available():
                try:
                    self._splat = live_splat.LiveSplatEngine()
                    self._splat.start()
                except Exception:
                    self._splat = None
            self._env = env
            self._environment_id = environment_id
            self._session_id = session["id"]
            self._started_at = time.monotonic()
            self._recording = bool(options.get("record", True))
            self._frame_counts = {}
            self._frame_bytes = {}
            self._last_fp = {}
            self._session_dir = self.work_root / session["id"]
            if self._recording:
                self._session_dir.mkdir(parents=True, exist_ok=True)

            self._stop_event = threading.Event()
            self._perception_thread = threading.Thread(
                target=self._perception_loop, name="session-perception", daemon=True
            )
            self._recorder_thread = threading.Thread(
                target=self._recorder_loop, name="session-recorder", daemon=True
            )
            self._perception_thread.start()
            self._recorder_thread.start()

        return self.status()

    def stop(self) -> dict[str, Any]:
        with self._lock:
            if self._env is None or self._session_id is None:
                return {"active": False}
            self._stop_event.set()
            env = self._env
            session_id = self._session_id
            recording = self._recording
            session_dir = self._session_dir
            duration = time.monotonic() - self._started_at

        # Join loops outside the lock.
        for thread in (self._perception_thread, self._recorder_thread):
            if thread is not None:
                thread.join(timeout=3.0)

        try:
            env.stop()
        except Exception:
            pass

        metrics: dict[str, Any] = {"durationSeconds": round(duration, 2)}
        if recording and session_dir is not None:
            metrics.update(self._finalize_records(session_id, env, session_dir))

        if self._splat is not None:
            try:
                self._splat.stop()
            except Exception:
                pass

        self.store.update_session(
            session_id,
            state="stopped",
            ended_at=time.strftime("%Y-%m-%d %H:%M:%S"),
            duration=_format_duration(duration),
            metrics=metrics,
        )

        with self._lock:
            final = {
                "active": False,
                "sessionId": session_id,
                "metrics": metrics,
            }
            self._env = None
            self._session_id = None
            self._environment_id = None
            self._recording = False
            self._session_dir = None
            self._splat = None
            self._perception_thread = None
            self._recorder_thread = None
        return final

    # ---------------------------------------------------------------- queries

    @property
    def active(self) -> bool:
        return self._env is not None

    def frame(self, drone_id: str) -> bytes | None:
        env = self._env
        return env.latest_frame(drone_id) if env is not None else None

    def depth_frame(self, drone_id: str) -> bytes | None:
        return self.depth.latest_depth_jpeg(drone_id)

    def point_cloud(self, max_points: int = 2500) -> dict[str, Any]:
        return {"points": self.depth.cloud_snapshot(max_points)}

    def splat_snapshot(self) -> bytes | None:
        """Export the active splat to .ply bytes (sim engine or runtime world)."""
        env = self._env
        if env is None:
            return None
        path = self.export_root / "world_model" / "live.ply"
        path.parent.mkdir(parents=True, exist_ok=True)
        try:
            if self._splat is not None:
                self._splat.export_ply(path)
            elif isinstance(env, RealEnvironment) and env.world_model_status().get("running"):
                self.runtime.export_world_model(path)
            else:
                return None
            return path.read_bytes()
        except Exception:
            return None

    def set_speed(self, mode: str) -> dict[str, Any]:
        env = self._env
        if env is None:
            raise RuntimeError("no active session")
        env.set_speed("max" if mode == "max" else "realtime")
        return self.status()

    def status(self) -> dict[str, Any]:
        env = self._env
        if env is None:
            return {"active": False, "segmentation": self.segmenter.status()}
        drones = env.drone_ids()
        return {
            "active": True,
            "sessionId": self._session_id,
            "environmentId": self._environment_id,
            "kind": env.kind,
            "recording": self._recording,
            "speed": "max" if env.status().get("speed") == "max" else "realtime",
            "elapsedSeconds": round(time.monotonic() - self._started_at, 2),
            "drones": drones,
            "frameCounts": dict(self._frame_counts),
            "trajectories": env.trajectories(),
            "worldModel": self._splat.snapshot() if self._splat is not None else env.world_model_status(),
            "segmentation": {
                "status": self.segmenter.status(),
                "screen": self.segmenter.screen_summary(),
                "world": self.segmenter.world_objects(),
            },
            "depth": self.depth.status(),
            "env": env.status(),
        }

    # ------------------------------------------------------------------ loops

    def _perception_loop(self) -> None:
        interval = 1.0 / max(0.5, self.perception_hz)
        tick = 0
        while not self._stop_event.is_set():
            started = time.monotonic()
            env = self._env
            if env is not None:
                run_depth = self.depth.available() and tick % self._depth_every == 0
                try:
                    if isinstance(env, SimEnvironment):
                        self._perceive_sim(env, run_depth)
                    else:
                        self._perceive_real(env, run_depth)
                except Exception:
                    pass
            tick += 1
            elapsed = time.monotonic() - started
            self._stop_event.wait(max(0.0, interval - elapsed))

    def _perceive_real(self, env: RealEnvironment, run_depth: bool) -> None:
        # Real cameras: segment with the model, depth with the z-forward
        # convention, and seed the runtime's splat once from the cloud.
        for drone_id in env.drone_ids():
            jpeg = env.latest_frame(drone_id)
            if not jpeg:
                continue
            pose = env.latest_pose(drone_id)
            if run_depth:
                self.depth.process(drone_id, jpeg, pose)
            dets = self.segmenter.segment_frame(drone_id, jpeg)
            self.segmenter.project_to_world(
                drone_id, dets, pose, depth_map=self.depth.latest_depth_map(drone_id)
            )
        if run_depth and not self._seeded and env.world_model_status().get("running"):
            xyz, rgb = self.depth.cloud_arrays()
            if xyz.shape[0]:
                self.runtime.seed_world_points(xyz, rgb)
                self._seeded = True

    def _perceive_sim(self, env: SimEnvironment, run_depth: bool) -> None:
        # The sim has exact poses + object positions: use the correct camera
        # basis for depth/cloud, derive world objects from ground truth, and
        # build a splat by ingesting frames with exact extrinsics.
        positions = env.positions()
        w, h = env.image_size()
        focal = (w / 2.0) / np.tan(np.deg2rad(env.fov_deg) / 2.0)
        cx, cy = w / 2.0, h / 2.0
        for drone_id in env.drone_ids():
            jpeg = env.latest_frame(drone_id)
            pose = env.latest_pose(drone_id)
            if not jpeg or pose is None:
                continue
            cam_rot = env.camera_rot(drone_id)
            if cam_rot is None:
                continue
            center = np.array([pose["x"], pose["y"], pose["z"]], dtype=float)
            if run_depth:
                self.depth.process(drone_id, jpeg, pose, cam_rot=cam_rot)

            # Ground-truth detections: the other drones, projected into this view.
            dets: list[ScreenDetection] = []
            worlds: list[list[float]] = []
            for other, p in positions.items():
                if other == drone_id:
                    continue
                rel = np.array(p, dtype=float) - center
                zc = float(rel @ cam_rot[:, 2])
                if zc <= 0.3:
                    continue
                u = cx + focal * float(rel @ cam_rot[:, 0]) / zc
                v = cy + focal * float(rel @ cam_rot[:, 1]) / zc
                if not (0 <= u < w and 0 <= v < h):
                    continue
                size = max(6.0, 240.0 / zc)
                dets.append(
                    ScreenDetection("drone", 0.99, [u - size / 2, v - size / 2, size, size], [u, v], [], w, h)
                )
                worlds.append(list(p))
            self.segmenter.ingest_truth(drone_id, dets, worlds)

            # Splat keyframe with exact camera extrinsics.
            if self._splat is not None:
                quat = rotmat_to_quat_xyzw(cam_rot)
                self._splat.ingest(
                    drone_id, jpeg,
                    {"x": float(center[0]), "y": float(center[1]), "z": float(center[2]), "rotation_xyzw": quat},
                )
        # Seed the sim splat once from the metric depth cloud for a dense init.
        if run_depth and self._splat is not None and not self._seeded:
            xyz, rgb = self.depth.cloud_arrays()
            if xyz.shape[0]:
                try:
                    self._splat.seed_from_points(xyz, rgb)
                    self._seeded = True
                except Exception:
                    pass

    def _recorder_loop(self) -> None:
        if not self._recording or self._session_dir is None:
            return
        interval = 1.0 / max(1.0, self.recorder_hz)
        control_path = self._session_dir / "control.jsonl"
        while not self._stop_event.is_set():
            started = time.monotonic()
            env = self._env
            if env is not None:
                snapshots = {}
                if isinstance(env, RealEnvironment):
                    snapshots = {s["droneId"]: s for s in env.runtime.snapshots().get("drones", [])}
                for drone_id in env.drone_ids():
                    jpeg = env.latest_frame(drone_id)
                    pose = env.latest_pose(drone_id)
                    self._record_drone_tick(drone_id, jpeg, pose, snapshots.get(drone_id), control_path)
            elapsed = time.monotonic() - started
            self._stop_event.wait(max(0.0, interval - elapsed))

    def _record_drone_tick(
        self,
        drone_id: str,
        jpeg: bytes | None,
        pose: dict[str, Any] | None,
        snapshot: dict[str, Any] | None,
        control_path: Path,
    ) -> None:
        assert self._session_dir is not None
        now = time.time()
        if jpeg:
            fp = _fingerprint(jpeg)
            if self._last_fp.get(drone_id) != fp:
                self._last_fp[drone_id] = fp
                drone_dir = self._session_dir / _safe(drone_id)
                drone_dir.mkdir(parents=True, exist_ok=True)
                index = self._frame_counts.get(drone_id, 0)
                (drone_dir / f"frame_{index:06d}.jpg").write_bytes(jpeg)
                self._frame_counts[drone_id] = index + 1
                self._frame_bytes[drone_id] = self._frame_bytes.get(drone_id, 0) + len(jpeg)
                if pose is not None:
                    with (drone_dir / "pose.jsonl").open("a") as handle:
                        handle.write(json.dumps({"frameIndex": index, "timestamp": now, **pose}) + "\n")
        if snapshot is not None and snapshot.get("lastAction") is not None:
            with control_path.open("a") as handle:
                handle.write(
                    json.dumps({"droneId": drone_id, "timestamp": now, "action": snapshot["lastAction"]}) + "\n"
                )

    # ------------------------------------------------------------- finalizing

    def _finalize_records(
        self, session_id: str, env: SimEnvironment | RealEnvironment, session_dir: Path
    ) -> dict[str, Any]:
        metrics: dict[str, Any] = {"frames": 0, "bytes": 0, "byDrone": {}}

        # Per-drone camera frames + pose tracks.
        for drone_id in env.drone_ids():
            drone_dir = session_dir / _safe(drone_id)
            if not drone_dir.is_dir():
                continue
            frames = sorted(drone_dir.glob("*.jpg"))
            if frames:
                self.store.import_record(
                    session_id, "camera", "frames", f"Camera frames — {drone_id}",
                    "image/jpeg-sequence", drone_dir, drone_id=drone_id,
                )
                metrics["frames"] += len(frames)
                metrics["byDrone"][drone_id] = {
                    "frames": len(frames), "bytes": self._frame_bytes.get(drone_id, 0)
                }
                metrics["bytes"] += self._frame_bytes.get(drone_id, 0)
            pose_path = drone_dir / "pose.jsonl"
            if pose_path.is_file() and pose_path.stat().st_size > 0:
                self.store.import_record(
                    session_id, "pose", "pose-track", f"Pose track — {drone_id}",
                    "application/jsonl", pose_path, drone_id=drone_id,
                )

        # Control signals (shared file).
        control_path = session_dir / "control.jsonl"
        if control_path.is_file() and control_path.stat().st_size > 0:
            self.store.import_record(
                session_id, "control", "control", "Control signals", "application/jsonl", control_path,
            )

        # Gaussian splat snapshot (sim: session-owned engine; real: runtime).
        try:
            ply = self.export_root / "world_model" / f"{session_id}.ply"
            ply.parent.mkdir(parents=True, exist_ok=True)
            exported = None
            if self._splat is not None:
                exported = str(self._splat.export_ply(ply))
            elif env.world_model_status().get("running"):
                exported = self.runtime.export_world_model(ply)
            if exported:
                self.store.import_record(
                    session_id, "splat", "gaussian-splat", "Live world splat",
                    "model/vnd.gaussian-splat", Path(exported),
                )
        except Exception:
            pass

        # Segmentation inferences.
        screen = self.segmenter.screen_summary()
        if screen:
            seg_screen = session_dir / "segmentation_screen.json"
            seg_screen.write_text(json.dumps(screen))
            self.store.import_record(
                session_id, "seg-screen", "seg-screen", "Screen-space segmentation",
                "application/json", seg_screen,
            )
        world = self.segmenter.world_objects()
        if world:
            seg_world = session_dir / "segmentation_world.json"
            seg_world.write_text(json.dumps(world))
            self.store.import_record(
                session_id, "seg-world", "seg-world", "World-space segmentation",
                "application/json", seg_world,
            )
        metrics["worldObjects"] = len(world)

        # Estimated-depth point cloud.
        xyz, rgb = self.depth.cloud_arrays()
        if xyz.shape[0]:
            cloud_path = session_dir / "pointcloud.ply"
            write_ply(cloud_path, xyz, rgb)
            self.store.import_record(
                session_id, "pointcloud", "point-cloud", "Depth point cloud",
                "model/ply", cloud_path,
            )
            metrics["points"] = int(xyz.shape[0])
        return metrics


def _safe(name: str) -> str:
    return "".join(c if c.isalnum() or c in "-_" else "_" for c in name)


def _format_duration(seconds: float) -> str:
    total = max(0, int(round(seconds)))
    hours, remainder = divmod(total, 3600)
    minutes, secs = divmod(remainder, 60)
    return f"{hours:02d}:{minutes:02d}:{secs:02d}"
