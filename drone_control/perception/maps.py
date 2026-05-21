from __future__ import annotations

from pathlib import Path
from typing import Any

from .state import MapSummary


MAP_RECORD_TYPES = {"gaussian-splat", "pose-track", "map", "point-cloud"}


def map_summary_from_record(record: dict[str, Any], path: Path | None = None) -> MapSummary:
    record_type = str(record.get("type") or "")
    label = str(record.get("label") or record_type or "map")
    keyframes = 0
    points = 0
    state = "artifact" if record_type in MAP_RECORD_TYPES else "none"
    if path is not None and path.exists():
        if path.is_dir():
            keyframes = len(list(path.glob("*.jpg")))
            points = len(list(path.glob("*.ply")))
        elif path.suffix == ".jsonl":
            keyframes = sum(1 for _ in path.open("r", encoding="utf-8", errors="ignore"))
        elif path.suffix in {".ply", ".splat", ".spz"}:
            points = 1
    return MapSummary(
        state=state,
        record_id=str(record.get("id") or "") or None,
        keyframes=keyframes,
        points=points,
        label=label,
    )


def latest_map_summary(records: list[dict[str, Any]], resolve_path: object | None = None) -> MapSummary:
    for record in reversed(records):
        if record.get("type") not in MAP_RECORD_TYPES:
            continue
        path = None
        if callable(resolve_path):
            path = resolve_path(str(record.get("id") or ""))
        return map_summary_from_record(record, path)
    return MapSummary()
