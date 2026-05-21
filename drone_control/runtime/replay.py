from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from drone_control.actions import DroneAction, action_from_dict
from drone_control.perception.state import FrameMetadata, MapSummary, PoseEstimate
from drone_control.runtime.events import DroneObservation


@dataclass(frozen=True, slots=True)
class ReplayTrace:
    observations: list[DroneObservation]
    actions: list[DroneAction]
    mission: dict[str, Any]


def load_replay_trace(path: str | Path) -> ReplayTrace:
    data = json.loads(Path(path).read_text())
    observations = [_observation_from_dict(item) for item in data.get("observations", [])]
    actions = [action_from_dict(item) for item in data.get("actions", [])]
    mission = dict(data.get("mission") or {})
    return ReplayTrace(observations=observations, actions=actions, mission=mission)


def _observation_from_dict(item: dict[str, Any]) -> DroneObservation:
    frame_data = item.get("latestFrame")
    pose_data = item.get("pose")
    map_data = item.get("mapSummary")
    return DroneObservation(
        timestamp=float(item.get("timestamp", 0.0)),
        drone_id=str(item.get("droneId", "")),
        link_state=str(item.get("linkState", "unknown")),
        latest_frame=FrameMetadata(
            index=frame_data.get("index"),
            timestamp=frame_data.get("timestamp"),
            width=frame_data.get("width"),
            height=frame_data.get("height"),
            source=str(frame_data.get("source", "")),
        )
        if isinstance(frame_data, dict)
        else None,
        pose=PoseEstimate(
            timestamp=pose_data.get("timestamp"),
            frame_index=pose_data.get("frame_index", pose_data.get("frameIndex")),
            translation=tuple(pose_data["translation"]) if isinstance(pose_data.get("translation"), list) else None,
            rotation_xyzw=tuple(pose_data["rotation_xyzw"])
            if isinstance(pose_data.get("rotation_xyzw"), list)
            else tuple(pose_data["rotation"])
            if isinstance(pose_data.get("rotation"), list)
            else None,
            quality=str(pose_data.get("quality", "unavailable")),
            confidence=float(pose_data.get("confidence", 0.0)),
        )
        if isinstance(pose_data, dict)
        else None,
        map_summary=MapSummary(
            state=str(map_data.get("state", "none")),
            record_id=map_data.get("recordId") or map_data.get("record_id"),
            keyframes=int(map_data.get("keyframes", 0)),
            points=int(map_data.get("points", 0)),
            label=str(map_data.get("label", "")),
        )
        if isinstance(map_data, dict)
        else None,
        battery=item.get("battery"),
        confidence=float(item.get("confidence", 0.0)),
    )
