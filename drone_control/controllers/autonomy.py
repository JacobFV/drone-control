from __future__ import annotations

from dataclasses import dataclass

from drone_control.actions import DroneAction
from drone_control.runtime.events import DroneObservation

from .base import ActionRequest, SafetyConstraints


@dataclass(slots=True)
class BoundedAutonomyController:
    """
    Built-in local autonomy policy for dry-run and provider-failure operation.

    This is intentionally conservative. It consumes the same observations and
    assignment metadata as a VLA, but it only emits small bounded primitives.
    Production model autonomy should use VLAController; this controller keeps
    the full runtime loop executable and testable without credentials or model
    weights.
    """

    name: str = "autonomy"

    def step(
        self,
        observation: DroneObservation,
        history: list[DroneObservation],
        constraints: SafetyConstraints,
    ) -> ActionRequest:
        task = str(constraints.metadata.get("assignmentTask") or "hold")
        if observation.link_state not in {"connected", "dry_run"}:
            return ActionRequest.stop("autonomy_link_unavailable")
        if observation.confidence < float(constraints.metadata.get("minConfidence", 0.0)):
            return ActionRequest.stop("autonomy_low_confidence")
        if task in {"survey", "inspect", "search"}:
            return ActionRequest(DroneAction(pitch=132, throttle=min(132, constraints.max_throttle)), reason=f"autonomy_{task}")
        if task in {"hold_safe_spacing", "hold", "support"}:
            return ActionRequest(DroneAction.neutral(), reason=f"autonomy_{task}")
        if task in {"land", "return"}:
            return ActionRequest(DroneAction(throttle=0, land=True), reason=f"autonomy_{task}")
        return ActionRequest(DroneAction.neutral(), reason="autonomy_default")
