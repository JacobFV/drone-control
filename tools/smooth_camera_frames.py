#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path

try:
    from PIL import Image, ImageChops, ImageDraw, ImageFilter, ImageStat
except ImportError as exc:
    raise SystemExit(
        "Pillow is required for frame smoothing. Install it with: python3 -m pip install --user pillow"
    ) from exc


@dataclass
class FrameMetrics:
    name: str
    raw_temporal_mae: float | None
    smooth_temporal_mae: float | None
    raw_speckle_mae: float
    smooth_speckle_mae: float
    raw_to_smooth_mae: float
    replaced_pixel_pct: float


def main() -> int:
    parser = argparse.ArgumentParser(description="Temporally smooth decoded drone camera JPEG frames.")
    parser.add_argument("input_dir", type=Path, help="Directory containing decoded .jpg frames.")
    parser.add_argument("--out-dir", type=Path, help="Output directory. Defaults to <input_dir>_smooth.")
    parser.add_argument("--pattern", default="*.jpg")
    parser.add_argument("--outlier-threshold", type=int, default=45, help="Current pixel must differ from both neighbors by this much.")
    parser.add_argument("--stable-threshold", type=int, default=28, help="Neighbor pixels must be this close to be considered stable.")
    parser.add_argument("--ema", type=float, default=0.12, help="Light blend weight from previous smoothed frame, 0 disables it.")
    parser.add_argument("--quality", type=int, default=92)
    parser.add_argument("--compare-index", type=int, default=13, help="Frame index for raw/smoothed contact sheet.")
    args = parser.parse_args()

    paths = sorted(args.input_dir.glob(args.pattern))
    if not paths:
        print(f"no frames found in {args.input_dir} matching {args.pattern}", file=sys.stderr)
        return 1

    out_dir = args.out_dir or args.input_dir.with_name(args.input_dir.name + "_smooth")
    out_dir.mkdir(parents=True, exist_ok=True)

    raw_frames = [load_rgb(path) for path in paths]
    smoothed_frames: list[Image.Image] = []
    metrics: list[FrameMetrics] = []

    previous_smooth: Image.Image | None = None
    for index, current in enumerate(raw_frames):
        previous_raw = raw_frames[index - 1] if index > 0 else current
        next_raw = raw_frames[index + 1] if index + 1 < len(raw_frames) else current

        corrected, replaced_pixel_pct = temporal_outlier_filter(
            previous_raw,
            current,
            next_raw,
            args.outlier_threshold,
            args.stable_threshold,
        )
        if previous_smooth is not None and args.ema > 0:
            corrected = Image.blend(corrected, previous_smooth, max(0.0, min(1.0, args.ema)))

        out_path = out_dir / paths[index].name
        corrected.save(out_path, quality=args.quality, optimize=True)
        smoothed_frames.append(corrected)

        raw_temporal_mae = image_mae(raw_frames[index - 1], current) if index > 0 else None
        smooth_temporal_mae = image_mae(smoothed_frames[index - 1], corrected) if index > 0 else None
        metrics.append(
            FrameMetrics(
                name=paths[index].name,
                raw_temporal_mae=raw_temporal_mae,
                smooth_temporal_mae=smooth_temporal_mae,
                raw_speckle_mae=speckle_mae(current),
                smooth_speckle_mae=speckle_mae(corrected),
                raw_to_smooth_mae=image_mae(current, corrected),
                replaced_pixel_pct=replaced_pixel_pct,
            )
        )
        previous_smooth = corrected

    summary = summarize(metrics)
    (out_dir / "metrics.json").write_text(json.dumps(summary, indent=2) + "\n")
    (out_dir / "metrics.txt").write_text(format_summary(summary))

    compare_index = min(max(args.compare_index, 0), len(paths) - 1)
    write_comparison_sheet(
        raw_frames[compare_index],
        smoothed_frames[compare_index],
        out_dir / f"comparison_{compare_index:05d}.jpg",
        paths[compare_index].name,
    )

    print(format_summary(summary).strip())
    print(f"output={out_dir}")
    print(f"comparison={out_dir / f'comparison_{compare_index:05d}.jpg'}")
    return 0


def load_rgb(path: Path) -> Image.Image:
    with Image.open(path) as image:
        return image.convert("RGB")


def temporal_outlier_filter(
    previous: Image.Image,
    current: Image.Image,
    next_frame: Image.Image,
    outlier_threshold: int,
    stable_threshold: int,
) -> tuple[Image.Image, float]:
    current_previous = threshold_high(max_channel_diff(current, previous), outlier_threshold)
    current_next = threshold_high(max_channel_diff(current, next_frame), outlier_threshold)
    previous_next = threshold_low(max_channel_diff(previous, next_frame), stable_threshold)
    mask = ImageChops.multiply(ImageChops.multiply(current_previous, current_next), previous_next)
    replacement = Image.blend(previous, next_frame, 0.5)
    filtered = Image.composite(replacement, current, mask)
    replaced = mask.histogram()[255]
    pixels = current.width * current.height
    return filtered, (replaced / pixels) * 100.0


def max_channel_diff(left: Image.Image, right: Image.Image) -> Image.Image:
    red, green, blue = ImageChops.difference(left, right).split()
    return ImageChops.lighter(ImageChops.lighter(red, green), blue)


def threshold_high(image: Image.Image, threshold: int) -> Image.Image:
    return image.point(lambda value: 255 if value >= threshold else 0, mode="L")


def threshold_low(image: Image.Image, threshold: int) -> Image.Image:
    return image.point(lambda value: 255 if value <= threshold else 0, mode="L")


def image_mae(left: Image.Image, right: Image.Image) -> float:
    if left.size != right.size:
        raise ValueError("image sizes differ")
    stat = ImageStat.Stat(ImageChops.difference(left, right))
    return sum(stat.mean) / len(stat.mean)


def speckle_mae(image: Image.Image) -> float:
    median = image.filter(ImageFilter.MedianFilter(size=3))
    return image_mae(image, median)


def summarize(metrics: list[FrameMetrics]) -> dict[str, object]:
    raw_temporal = [item.raw_temporal_mae for item in metrics if item.raw_temporal_mae is not None]
    smooth_temporal = [item.smooth_temporal_mae for item in metrics if item.smooth_temporal_mae is not None]
    raw_speckle = [item.raw_speckle_mae for item in metrics]
    smooth_speckle = [item.smooth_speckle_mae for item in metrics]
    changed = [item.raw_to_smooth_mae for item in metrics]
    replaced = [item.replaced_pixel_pct for item in metrics]

    raw_temporal_mean = mean(raw_temporal)
    smooth_temporal_mean = mean(smooth_temporal)
    raw_speckle_mean = mean(raw_speckle)
    smooth_speckle_mean = mean(smooth_speckle)

    return {
        "frames": len(metrics),
        "raw_temporal_mae": raw_temporal_mean,
        "smooth_temporal_mae": smooth_temporal_mean,
        "temporal_mae_delta_pct": percent_delta(raw_temporal_mean, smooth_temporal_mean),
        "raw_speckle_mae": raw_speckle_mean,
        "smooth_speckle_mae": smooth_speckle_mean,
        "speckle_mae_delta_pct": percent_delta(raw_speckle_mean, smooth_speckle_mean),
        "raw_to_smooth_mae": mean(changed),
        "replaced_pixel_pct": mean(replaced),
        "worst_raw_speckle_frames": [
            item.name for item in sorted(metrics, key=lambda value: value.raw_speckle_mae, reverse=True)[:8]
        ],
    }


def mean(values: list[float]) -> float:
    return sum(values) / len(values) if values else 0.0


def percent_delta(before: float, after: float) -> float:
    if math.isclose(before, 0.0):
        return 0.0
    return ((after - before) / before) * 100.0


def format_summary(summary: dict[str, object]) -> str:
    return (
        f"frames={summary['frames']}\n"
        f"temporal_mae: raw={summary['raw_temporal_mae']:.3f} "
        f"smooth={summary['smooth_temporal_mae']:.3f} "
        f"delta={summary['temporal_mae_delta_pct']:.1f}%\n"
        f"speckle_mae:  raw={summary['raw_speckle_mae']:.3f} "
        f"smooth={summary['smooth_speckle_mae']:.3f} "
        f"delta={summary['speckle_mae_delta_pct']:.1f}%\n"
        f"raw_to_smooth_mae={summary['raw_to_smooth_mae']:.3f}\n"
        f"replaced_pixel_pct={summary['replaced_pixel_pct']:.3f}%\n"
        f"worst_raw_speckle_frames={', '.join(summary['worst_raw_speckle_frames'])}\n"
    )


def write_comparison_sheet(raw: Image.Image, smooth: Image.Image, path: Path, name: str) -> None:
    label_h = 26
    sheet = Image.new("RGB", (raw.width * 2, raw.height + label_h), "white")
    sheet.paste(raw, (0, label_h))
    sheet.paste(smooth, (raw.width, label_h))
    draw = ImageDraw.Draw(sheet)
    draw.text((8, 7), f"raw {name}", fill=(0, 0, 0))
    draw.text((raw.width + 8, 7), "smoothed", fill=(0, 0, 0))

    diff = ImageChops.difference(raw, smooth)
    if diff.getbbox():
        diff_path = path.with_name(path.stem + "_diff.jpg")
        diff.save(diff_path, quality=92, optimize=True)
    sheet.save(path, quality=92, optimize=True)


if __name__ == "__main__":
    raise SystemExit(main())
