from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable

from .tasks import Assignment, ConstraintUpdate, Mission, MissionProgress


@dataclass(slots=True)
class VLMCoordinator:
    model_step: Callable[[dict[str, Any]], dict[str, Any]] | None = None

    @property
    def available(self) -> bool:
        return self.model_step is not None

    def step(self, mission: Mission, summaries: list[dict[str, Any]]) -> MissionProgress:
        if self.model_step is None:
            return MissionProgress(mission.id, "unavailable", notes=["vlm_unavailable"])
        try:
            return parse_vlm_output(mission, summaries, self.model_step({"mission": mission.as_dict(), "drones": summaries}))
        except (KeyError, TypeError, ValueError) as exc:
            return MissionProgress(mission.id, "faulted", notes=[f"invalid_vlm_output:{exc}"])


def parse_vlm_output(mission: Mission, summaries: list[dict[str, Any]], result: dict[str, Any]) -> MissionProgress:
    if not isinstance(result, dict):
        raise TypeError("model output must be an object")
    state = result.get("state", "running")
    if state not in {"running", "blocked", "completed", "faulted"}:
        raise ValueError("state must be running, blocked, completed, or faulted")
    raw_assignments = result.get("assignments", [])
    if not isinstance(raw_assignments, list):
        raise TypeError("assignments must be a list")
    known = {str(item.get("droneId")) for item in summaries if isinstance(item, dict) and item.get("droneId")}
    assignments: list[Assignment] = []
    for item in raw_assignments:
        if not isinstance(item, dict):
            raise TypeError("assignment must be an object")
        drone_id = str(item["droneId"])
        if drone_id not in known:
            raise ValueError(f"assignment references unknown drone {drone_id}")
        role = item.get("role", "support")
        task = item.get("task", "hold")
        if not isinstance(role, str) or not role:
            raise TypeError("role must be a non-empty string")
        if not isinstance(task, str) or not task:
            raise TypeError("task must be a non-empty string")
        constraints = item.get("constraints") if isinstance(item.get("constraints"), dict) else {}
        max_throttle = constraints.get("maxThrottle")
        if max_throttle is not None:
            if not isinstance(max_throttle, int) or max_throttle < 0 or max_throttle > 255:
                raise ValueError("maxThrottle must be an integer from 0 to 255")
        require_heartbeat = constraints.get("requireHeartbeat")
        if require_heartbeat is not None and not isinstance(require_heartbeat, bool):
            raise TypeError("requireHeartbeat must be boolean")
        min_confidence = constraints.get("minConfidence", 0.0)
        if not isinstance(min_confidence, (int, float)) or min_confidence < 0.0 or min_confidence > 1.0:
            raise ValueError("minConfidence must be between 0 and 1")
        assignments.append(
            Assignment(
                drone_id=drone_id,
                role=role,
                task=task,
                constraints=ConstraintUpdate(
                    drone_id=drone_id,
                    max_throttle=max_throttle,
                    require_heartbeat=require_heartbeat,
                    min_confidence=float(min_confidence),
                ),
            )
        )
    raw_notes = result.get("notes", [])
    if not isinstance(raw_notes, list) or not all(isinstance(item, str) for item in raw_notes):
        raise TypeError("notes must be a list of strings")
    return MissionProgress(mission.id, state, assignments=assignments, notes=raw_notes)
