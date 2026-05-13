#!/usr/bin/env python3
from __future__ import annotations

import argparse
import itertools
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drone_control.live_video import DirectoryFrameSource, mjpeg_chunks


def main() -> int:
    parser = argparse.ArgumentParser(description="Smoke-test a live_video frame directory as MJPEG chunks.")
    parser.add_argument("frame_dir", nargs="?", type=Path, help="Directory containing .jpg frames.")
    parser.add_argument("--fps", type=float, default=10.0)
    parser.add_argument("--chunks", type=int, default=5, help="Number of multipart chunks to print metadata for.")
    args = parser.parse_args()

    frame_dir = args.frame_dir or find_existing_frame_dir()
    if frame_dir is None:
        print("no existing frame directory found; pass one explicitly", file=sys.stderr)
        return 2

    source = DirectoryFrameSource(frame_dir, fps=args.fps)
    for index, chunk in enumerate(itertools.islice(mjpeg_chunks(source), args.chunks)):
        print(f"chunk[{index}] bytes={len(chunk)} preview={chunk[:40]!r}")
    return 0


def find_existing_frame_dir() -> Path | None:
    for root in (Path("camera_captures"), Path(".")):
        if not root.exists():
            continue
        for path in sorted(root.glob("**")):
            if path.is_dir() and next(path.glob("*.jpg"), None) is not None:
                return path
    return None


if __name__ == "__main__":
    raise SystemExit(main())
