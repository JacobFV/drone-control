from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable

from drone_control.actions import action_from_dict
from drone_control.runtime.events import DroneObservation

from .base import ActionRequest, SafetyConstraints


@dataclass(slots=True)
class VLAController:
    """
    Strict adapter boundary for future single-drone VLA control.

    The callable must return a dict with an ``action`` object. This adapter does
    schema-level validation only; the shared safety wrapper still clamps output
    before packets are built.
    """

    model_step: Callable[[dict[str, Any]], dict[str, Any]] | None = None
    mission_context: dict[str, Any] | None = None
    name: str = "vla"

    @property
    def available(self) -> bool:
        return self.model_step is not None

    def step(
        self,
        observation: DroneObservation,
        history: list[DroneObservation],
        constraints: SafetyConstraints,
    ) -> ActionRequest:
        if self.model_step is None:
            return ActionRequest.stop("vla_unavailable")
        payload = {
            "observation": observation.as_dict(),
            "history": [item.as_dict() for item in history[-20:]],
            "constraints": constraints.as_dict(),
            "mission": self.mission_context or {},
        }
        try:
            result = self.model_step(payload)
            action_data = result.get("action")
            if not isinstance(action_data, dict):
                return ActionRequest.stop("vla_bad_output")
            confidence = float(result.get("confidence", 0.0))
            if confidence < 0.0 or confidence > 1.0:
                return ActionRequest.stop("vla_bad_confidence")
            return ActionRequest(action_from_dict(action_data), reason=str(result.get("reason", "vla")), confidence=confidence)
        except (TypeError, ValueError, KeyError) as exc:
            return ActionRequest.stop(f"vla_invalid_output:{exc}")
