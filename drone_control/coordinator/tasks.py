from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any


@dataclass(frozen=True, slots=True)
class Mission:
    id: str
    objective: str
    context: dict[str, Any] = field(default_factory=dict)

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True, slots=True)
class Role:
    name: str
    description: str = ""

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True, slots=True)
class ConstraintUpdate:
    drone_id: str
    max_throttle: int | None = None
    require_heartbeat: bool | None = None
    min_confidence: float = 0.0

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True, slots=True)
class Assignment:
    drone_id: str
    role: str
    task: str
    constraints: ConstraintUpdate | None = None

    def as_dict(self) -> dict[str, Any]:
        return {
            "droneId": self.drone_id,
            "role": self.role,
            "task": self.task,
            "constraints": self.constraints.as_dict() if self.constraints else None,
        }


@dataclass(frozen=True, slots=True)
class MissionProgress:
    mission_id: str
    state: str
    assignments: list[Assignment] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)

    def as_dict(self) -> dict[str, Any]:
        return {
            "missionId": self.mission_id,
            "state": self.state,
            "assignments": [item.as_dict() for item in self.assignments],
            "notes": self.notes,
        }

