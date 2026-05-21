from __future__ import annotations

import time
from dataclasses import dataclass

from drone_control.runtime.drone_runtime import DroneRuntimeSnapshot

from .tasks import Assignment, ConstraintUpdate, Mission, MissionProgress


@dataclass(slots=True)
class CoordinatorScheduler:
    mission: Mission | None = None
    tick_hz: float = 1.0
    last_progress: MissionProgress | None = None
    _last_tick_at: float = 0.0

    def start(self, mission: Mission) -> MissionProgress:
        self.mission = mission
        self.last_progress = MissionProgress(mission.id, "running", notes=["mission_started"])
        self._last_tick_at = 0.0
        return self.last_progress

    def stop(self) -> MissionProgress:
        mission_id = self.mission.id if self.mission else ""
        self.mission = None
        self.last_progress = MissionProgress(mission_id, "stopped", notes=["mission_stopped"])
        return self.last_progress

    def step(self, snapshots: list[DroneRuntimeSnapshot]) -> MissionProgress:
        if self.mission is None:
            self.last_progress = MissionProgress("", "idle")
            return self.last_progress
        now = time.monotonic()
        if self._last_tick_at and now - self._last_tick_at < 1.0 / max(0.1, self.tick_hz):
            return self.last_progress or MissionProgress(self.mission.id, "running")
        self._last_tick_at = now

        assignments: list[Assignment] = []
        notes: list[str] = []
        available = [snapshot for snapshot in snapshots if snapshot.link_state in {"connected", "dry_run"}]
        if not snapshots:
            self.last_progress = MissionProgress(self.mission.id, "blocked", notes=["missing_drone"])
            return self.last_progress
        if not available:
            self.last_progress = MissionProgress(self.mission.id, "blocked", notes=["all_links_unavailable"])
            return self.last_progress

        for index, snapshot in enumerate(snapshots):
            obs = snapshot.observation
            confidence = obs.confidence
            if snapshot.link_state not in {"connected", "dry_run"}:
                notes.append(f"{snapshot.drone_id}:link_{snapshot.link_state}")
            if confidence < 0.2:
                notes.append(f"{snapshot.drone_id}:low_confidence")
            role = "lead" if snapshot is available[0] else "support"
            task = "survey" if snapshot is available[0] else "hold_safe_spacing"
            max_throttle = 120 if confidence < 0.2 or snapshot.link_state not in {"connected", "dry_run"} else 160
            assignments.append(
                Assignment(
                    drone_id=snapshot.drone_id,
                    role=role,
                    task=task,
                    constraints=ConstraintUpdate(drone_id=snapshot.drone_id, max_throttle=max_throttle, min_confidence=0.2),
                )
            )
        self.last_progress = MissionProgress(self.mission.id, "running", assignments=assignments, notes=notes)
        return self.last_progress
