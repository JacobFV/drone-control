"""
Depth/SLAM evaluation CLI.

Renders a parallax-rich camera fly-through of a sim scene, runs the depth
front-ends against the raycast ground-truth oracle, prints a metrics table, and
dumps RGB | GT | estimate | error panels for visual inspection.

    python -m tools.depth_eval.run --scene warehouse --frames 30
    python -m tools.depth_eval.run --scene warehouse --mono   # also score monocular

Metrics (over pixels where GT is valid):
  absRel  mean |est-gt|/gt          (lower; <0.1 excellent)
  delta1  fraction with max(e/g,g/e)<1.25  (higher; >0.9 excellent)
  corr    Pearson corr of est vs gt depth  (higher; sign matters)
  rmse    metres
``--align`` reports the affine-aligned scores (the charitable view for the
scale-ambiguous monocular prior); the multi-view front-end is metric and is
scored without alignment.
"""

from __future__ import annotations

import argparse
import io

import numpy as np
from PIL import Image

from drone_control.perception.slam import MultiViewSLAM

from .metrics import colorize, error_map, metrics
from .sequence import generate


def _agg(rows, key):
    vals = [r[key] for r in rows if np.isfinite(r[key])]
    return float(np.mean(vals)) if vals else float("nan")


def _print_table(name, rows, align):
    print(f"  {name:24s} cov={_agg(rows,'coverage'):.2f}  "
          f"absRel={_agg(rows,'absRel'):.3f}  delta1={_agg(rows,'delta1'):.2f}  "
          f"corr={_agg(rows,'corr'):+.2f}  rmse={_agg(rows,'rmse'):.2f}"
          + ("  [affine-aligned]" if align else ""))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", default="warehouse")
    ap.add_argument("--frames", type=int, default=30)
    ap.add_argument("--image-size", type=int, default=128)
    ap.add_argument("--noise", default="medium")
    ap.add_argument("--far", type=float, default=22.0)
    ap.add_argument("--warmup", type=int, default=6, help="ignore frames until the window fills")
    ap.add_argument("--mono", action="store_true", help="also score the monocular baseline")
    ap.add_argument("--out", default="/tmp/depth_eval.png")
    args = ap.parse_args()

    print(f"scene={args.scene} frames={args.frames} size={args.image_size} noise={args.noise}")
    frames = generate(args.scene, n=args.frames, image_size=args.image_size,
                      noise=args.noise, seed=1)

    slam = MultiViewSLAM(far=args.far, near=0.5)
    mvs_rows, store = [], {}
    for f in frames:
        slam.process("sim-0", f.jpeg, f.pose)
        dm = slam.latest_depth_map("sim-0")
        ready = len(slam._windows.get("sim-0", type("X", (), {"grays": []})()).grays) >= args.warmup
        if dm is not None and ready:
            mvs_rows.append(metrics(dm, f.gt_depth))
            store[f.index] = (dm, f)

    print("\nResults:")
    _print_table("multi-view SLAM (metric)", mvs_rows, align=False)

    if args.mono:
        from drone_control.perception.depth import DepthEstimator
        de = DepthEstimator()
        raw, ali = [], []
        for f in frames[args.warmup:]:
            de.process("sim-0", f.jpeg, f.pose)
            dm = de.latest_depth_map("sim-0")
            if dm is not None:
                raw.append(metrics(dm, f.gt_depth, align=False))
                ali.append(metrics(dm, f.gt_depth, align=True))
        _print_table("monocular (raw)", raw, align=False)
        _print_table("monocular (aligned)", ali, align=True)

    # Visual panels.
    picks = sorted(store)[:: max(1, len(store) // 3)][:3]
    panels = []
    h = int(args.image_size * 0.75)
    sep = np.full((h, 3, 3), 255, np.uint8)
    for ri in picks:
        dm, f = store[ri]
        rgb = np.asarray(Image.open(io.BytesIO(f.jpeg)).convert("RGB"))
        gt = f.gt_depth
        panels.append(np.concatenate(
            [rgb, sep, colorize(gt, 0.5, args.far * 0.8), sep,
             colorize(dm, 0.5, args.far * 0.8), sep, error_map(dm, gt, 1.0)], axis=1))
        panels.append(np.full((3, panels[-1].shape[1], 3), 255, np.uint8))
    if panels:
        canvas = np.concatenate(panels, axis=0)
        Image.fromarray(canvas).resize((canvas.shape[1] * 4, canvas.shape[0] * 4),
                                       Image.NEAREST).save(args.out)
        print(f"\nRGB | GT | MVS | error  ->  {args.out}  (rows: frames {picks})")


if __name__ == "__main__":
    main()
