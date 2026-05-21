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
            result = self.model_step({"mission": mission.as_dict(), "drones": summaries})
            raw_assignments = result.get("assignments", [])
            if not isinstance(raw_assignments, list):
                return MissionProgress(mission.id, "faulted", notes=["bad_assignments"])
            assignments: list[Assignment] = []
            for item in raw_assignments:
                if not isinstance(item, dict):
                    return MissionProgress(mission.id, "faulted", notes=["bad_assignment"])
                drone_id = str(item["droneId"])
                constraints = item.get("constraints") if isinstance(item.get("constraints"), dict) else {}
                assignments.append(
                    Assignment(
                        drone_id=drone_id,
                        role=str(item.get("role", "support")),
                        task=str(item.get("task", "hold")),
                        constraints=ConstraintUpdate(
                            drone_id=drone_id,
                            max_throttle=constraints.get("maxThrottle"),
                            require_heartbeat=constraints.get("requireHeartbeat"),
                            min_confidence=float(constraints.get("minConfidence", 0.0)),
                        ),
                    )
                )
            return MissionProgress(mission.id, str(result.get("state", "running")), assignments=assignments)
        except (KeyError, TypeError, ValueError) as exc:
            return MissionProgress(mission.id, "faulted", notes=[f"invalid_vlm_output:{exc}"])
