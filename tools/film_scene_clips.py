#!/usr/bin/env python3
"""Render a library of per-scene clips straight from the sim (no UI needed).

For each named scene this runs a live ``SimSession`` and captures two streams:

  * an *omniscient* cinematic orbit (god's-eye ground-truth view), and
  * a drone forward-camera POV (the OV2640 stream the perception stack sees).

Frames are written as a numbered JPEG sequence and encoded to mp4 with ffmpeg.
The sim is paced in *sim time* (sampled at a fixed output fps) so playback is
smooth regardless of how fast the renderer actually runs.

Usage:
    python tools/film_scene_clips.py --out film/assets/clips/scenes \
        --scenes house_on_fire retail_store city warehouse --seconds 9
"""
from __future__ import annotations

import argparse
import math
import shutil
import subprocess
import time
from pathlib import Path

import numpy as np

from drone_control.sim.render import CameraConfig, CameraRenderer
from drone_control.sim.session import SimSession, SimSessionConfig

ALL_SCENES = [
    "open_field", "warehouse", "office", "city", "park",
    "construction", "atrium", "clothing_store", "retail_store", "house_on_fire",
]


def encode(frame_dir: Path, out: Path, fps: int) -> None:
    out.parent.mkdir(parents=True, exist_ok=True)
    cmd = [
        "ffmpeg", "-y", "-framerate", str(fps), "-i", str(frame_dir / "f%05d.jpg"),
        "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "18",
        "-vf", "scale=trunc(iw/2)*2:trunc(ih/2)*2", "-movflags", "+faststart", str(out),
    ]
    subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def render_scene(scene: str, out_dir: Path, *, seconds: float, fps: int,
                 drones: int, omni_w: int, omni_h: int) -> None:
    print(f"[{scene}] starting sim ({drones} drones)…", flush=True)
    cfg = SimSessionConfig(
        num_drones=drones, task="goto", scene=scene,
        camera_noise="medium", camera_model="ov2640", max_speed=True, seed=7,
    )
    sess = SimSession()
    sess.start(cfg)
    # Dedicated high-res omniscient renderer (the session's default is 480x360).
    sess._omni_renderer = CameraRenderer(  # noqa: SLF001 - intentional override
        CameraConfig(width=omni_w, height=omni_h, fov_deg=72.0),
        scene=sess._scene, noise=None,  # noqa: SLF001
    )

    omni_dir = out_dir / "_frames" / f"{scene}_omni"
    pov_dir = out_dir / "_frames" / f"{scene}_pov"
    for d in (omni_dir, pov_dir):
        if d.exists():
            shutil.rmtree(d)
        d.mkdir(parents=True)

    n = int(seconds * fps)
    dt = 1.0 / fps
    # Let the world settle so smoke/fire/cloth populate before we record.
    time.sleep(1.5)
    next_t = sess.sim_time() + 0.5
    pov_idx = 0
    for i in range(n):
        # Wait until sim time reaches this sample point (smooth sim-time pacing).
        deadline = time.time() + 2.0
        while sess.sim_time() < next_t and time.time() < deadline:
            time.sleep(0.002)
        next_t += dt

        # Cinematic orbit: slow 360 over the clip, gentle height bob.
        ang = 2.0 * math.pi * (i / max(n, 1)) * 0.85 + 0.6
        st = sess.status()
        pts = np.array([d["position"] for d in st["drones"]], dtype=float)
        centroid = pts.mean(axis=0) if len(pts) else np.zeros(3)
        spread = float(np.linalg.norm(pts - centroid, axis=1).max()) if len(pts) > 1 else 0.0
        radius = max(16.0, spread * 1.7 + 9.0)
        height = radius * (0.45 + 0.08 * math.sin(2.0 * math.pi * i / max(n, 1)))
        eye = centroid + np.array([radius * math.cos(ang), radius * math.sin(ang), height])
        target = centroid + np.array([0.0, 0.0, 1.0])
        omni = sess.omniscient_frame({"eye": eye.tolist(), "target": target.tolist()})
        if omni:
            (omni_dir / f"f{i:05d}.jpg").write_bytes(omni)

        pov = sess.frame(0)
        if pov:
            (pov_dir / f"f{pov_idx:05d}.jpg").write_bytes(pov)
            pov_idx += 1

    sess.stop()
    print(f"[{scene}] captured {n} omni / {pov_idx} pov frames; encoding…", flush=True)
    encode(omni_dir, out_dir / f"scene_{scene}_omni.mp4", fps)
    if pov_idx > fps:
        encode(pov_dir, out_dir / f"scene_{scene}_pov.mp4", fps)
    shutil.rmtree(omni_dir, ignore_errors=True)
    shutil.rmtree(pov_dir, ignore_errors=True)
    print(f"[{scene}] done -> {out_dir}/scene_{scene}_*.mp4", flush=True)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=Path("film/assets/clips/scenes"))
    ap.add_argument("--scenes", nargs="*", default=ALL_SCENES)
    ap.add_argument("--seconds", type=float, default=9.0)
    ap.add_argument("--fps", type=int, default=24)
    ap.add_argument("--drones", type=int, default=4)
    ap.add_argument("--omni-w", type=int, default=1280)
    ap.add_argument("--omni-h", type=int, default=720)
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    for scene in args.scenes:
        render_scene(scene, args.out, seconds=args.seconds, fps=args.fps,
                     drones=args.drones, omni_w=args.omni_w, omni_h=args.omni_h)


if __name__ == "__main__":
    main()
