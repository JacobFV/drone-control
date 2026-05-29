from __future__ import annotations

import base64
import threading
import time
from concurrent.futures import Future, TimeoutError as FutureTimeoutError
from dataclasses import dataclass, field
from typing import Any, Callable

from drone_control.actions import DroneAction
from drone_control.perception.frame_registry import LiveFrameRegistry
from drone_control.runtime.events import DroneObservation

from .base import ActionRequest, SafetyConstraints
from .vla import parse_vla_output

# A batched model takes a list of per-drone request payloads and returns a list
# of result objects (each ideally carrying its own ``droneId``). The hub indexes
# results by droneId, falling back to positional alignment.
BatchModelStep = Callable[[list[dict[str, Any]]], list[dict[str, Any]]]


class BatchedVLAHub:
    """
    Coalescing barrier that turns N independent per-drone control ticks into one
    batched model call per window.

    Each :class:`BatchedVLAController` submits its observation and blocks on a
    future. A background worker collects everything that arrives inside a short
    window and issues a single ``model_step_batch`` call, then fulfils each
    drone's future. The window flushes when either every *registered* drone has
    submitted (the common all-armed case → near-zero added latency) or
    ``max_wait_seconds`` elapses since the window opened (so disarmed/idle drones
    that never submit cannot stall the batch).

    On any model failure the pending futures resolve to ``None``; the controller
    then emits ``motor_stop`` and the shared :class:`SafetyController` still
    clamps the result downstream.
    """

    def __init__(self, model_step_batch: BatchModelStep, *, max_wait_seconds: float = 0.025) -> None:
        self._model = model_step_batch
        self._max_wait = max(0.001, float(max_wait_seconds))
        self._cond = threading.Condition()
        self._pending: dict[str, tuple[dict[str, Any], Future]] = {}
        self._registered: set[str] = set()
        self._window_start: float | None = None
        self._running = True
        self.last_batch_size = 0
        self.batches = 0
        self._worker = threading.Thread(target=self._run, name="batched-vla-hub", daemon=True)
        self._worker.start()

    def register(self, drone_id: str) -> None:
        with self._cond:
            self._registered.add(drone_id)

    def unregister(self, drone_id: str) -> None:
        with self._cond:
            self._registered.discard(drone_id)
            pending = self._pending.pop(drone_id, None)
        if pending is not None and not pending[1].done():
            pending[1].set_result(None)

    def submit(self, drone_id: str, payload: dict[str, Any]) -> Future:
        future: Future = Future()
        with self._cond:
            stale = self._pending.get(drone_id)
            if stale is not None and not stale[1].done():
                stale[1].set_result(None)
            self._pending[drone_id] = (payload, future)
            if self._window_start is None:
                self._window_start = time.monotonic()
            self._cond.notify()
        return future

    def close(self) -> None:
        with self._cond:
            self._running = False
            pending = list(self._pending.values())
            self._pending.clear()
            self._cond.notify_all()
        for _payload, future in pending:
            if not future.done():
                future.set_result(None)

    def _run(self) -> None:
        while True:
            with self._cond:
                while self._running and not self._pending:
                    self._cond.wait()
                if not self._running:
                    return
                while True:
                    target = len(self._registered) or 1
                    if len(self._pending) >= target:
                        break
                    elapsed = time.monotonic() - (self._window_start or time.monotonic())
                    remaining = self._max_wait - elapsed
                    if remaining <= 0:
                        break
                    self._cond.wait(timeout=remaining)
                    if not self._running:
                        return
                batch = self._pending
                self._pending = {}
                self._window_start = None
            self._flush(batch)

    def _flush(self, batch: dict[str, tuple[dict[str, Any], Future]]) -> None:
        drone_ids = list(batch)
        payloads = [batch[d][0] for d in drone_ids]
        self.last_batch_size = len(payloads)
        self.batches += 1
        results_by_id: dict[str, dict[str, Any]] = {}
        try:
            results = self._model(payloads)
            results_by_id = _index_results(results, drone_ids)
        except Exception:
            results_by_id = {}
        for index, drone_id in enumerate(drone_ids):
            future = batch[drone_id][1]
            if future.done():
                continue
            future.set_result(results_by_id.get(drone_id))


def _index_results(results: Any, drone_ids: list[str]) -> dict[str, dict[str, Any]]:
    if not isinstance(results, list):
        raise TypeError("batched model output must be a list")
    by_id: dict[str, dict[str, Any]] = {}
    for index, item in enumerate(results):
        if not isinstance(item, dict):
            raise TypeError("each batched result must be an object")
        drone_id = item.get("droneId")
        if not isinstance(drone_id, str) or drone_id not in set(drone_ids):
            # Positional fallback when the model does not echo droneId.
            if index < len(drone_ids):
                drone_id = drone_ids[index]
            else:
                continue
        by_id[drone_id] = item
    return by_id


@dataclass(slots=True)
class BatchedVLAController:
    """Per-drone proxy that routes one control tick through the shared hub."""

    hub: BatchedVLAHub
    drone_id: str
    registry: LiveFrameRegistry | None = None
    guidance_bus: Any = None
    wait_seconds: float = 0.05
    frame_max_age_seconds: float = 1.0
    mission_context: dict[str, Any] | None = None
    name: str = "batched_vla"
    recent_actions: list[DroneAction] = field(default_factory=list)
    _registered: bool = field(default=False, repr=False)

    def __post_init__(self) -> None:
        self.hub.register(self.drone_id)
        self._registered = True

    def step(
        self,
        observation: DroneObservation,
        history: list[DroneObservation],
        constraints: SafetyConstraints,
    ) -> ActionRequest:
        payload = self._build_payload(observation, history, constraints)
        future = self.hub.submit(self.drone_id, payload)
        try:
            result = future.result(timeout=self.wait_seconds)
        except FutureTimeoutError:
            return ActionRequest.stop("batched_vla_timeout")
        except Exception as exc:  # defensive: hub-side failure surfaced as fault
            return ActionRequest(DroneAction.motor_stop(), reason="batched_vla_error", fault=str(exc))
        if result is None:
            return ActionRequest.stop("batched_vla_no_result")
        try:
            action, confidence, reason = parse_vla_output(result)
            return ActionRequest(action, reason=reason, confidence=confidence)
        except (TypeError, ValueError, KeyError) as exc:
            return ActionRequest(DroneAction.motor_stop(), reason="batched_vla_invalid_output", fault=str(exc))

    def _build_payload(
        self,
        observation: DroneObservation,
        history: list[DroneObservation],
        constraints: SafetyConstraints,
    ) -> dict[str, Any]:
        frame_b64: str | None = None
        frame_width: int | None = None
        frame_height: int | None = None
        if self.registry is not None:
            frame = self.registry.latest(self.drone_id, max_age_seconds=self.frame_max_age_seconds)
            if frame is not None:
                frame_b64 = base64.b64encode(frame.jpeg).decode("ascii")
                frame_width = frame.width
                frame_height = frame.height

        # Fold low-frequency VLM guidance into the per-tick conditioning: a target
        # (or trajectory waypoint) becomes goalRel, plus a style vector and an
        # optional policy id that groups the batch.
        goal_rel: list[float] | None = None
        style: list[float] = []
        policy_id: str | None = None
        if self.guidance_bus is not None:
            pos = _observation_position(observation)
            target, style, policy_id = self.guidance_bus.resolve(self.drone_id, pos)
            if target is not None:
                goal_rel = [target[0] - pos[0], target[1] - pos[1], target[2] - pos[2]]

        return {
            "droneId": self.drone_id,
            "observation": observation.as_dict(),
            "history": [item.as_dict() for item in history[-8:]],
            "recentActions": [_action_schema(item) for item in self.recent_actions[-20:]],
            "constraints": constraints.as_dict(),
            "mission": self.mission_context or {},
            "frameJpegB64": frame_b64,
            "frameWidth": frame_width,
            "frameHeight": frame_height,
            "goalRel": goal_rel,
            "style": style,
            "policyId": policy_id,
        }

    def set_recent_actions(self, actions: list[DroneAction]) -> None:
        self.recent_actions = list(actions[-20:])

    def record_action(self, action: DroneAction) -> None:
        self.recent_actions.append(action.sanitized())
        del self.recent_actions[:-20]

    def close(self) -> None:
        # The hub is shared and owned by the manager; only drop this drone.
        if self._registered:
            self.hub.unregister(self.drone_id)
            self._registered = False


def _observation_position(observation: DroneObservation) -> tuple[float, float, float]:
    pose = observation.pose
    if pose is not None and getattr(pose, "translation", None):
        t = pose.translation
        return (float(t[0]), float(t[1]), float(t[2]))
    return (0.0, 0.0, 0.0)


def _action_schema(action: DroneAction) -> dict[str, Any]:
    action = action.sanitized()
    return {
        "roll": action.roll,
        "pitch": action.pitch,
        "throttle": action.throttle,
        "yaw": action.yaw,
        "takeoff": action.takeoff,
        "land": action.land,
        "emergencyStop": action.emergency_stop,
        "calibrate": action.calibrate,
        "headless": action.headless,
        "flip": action.flip,
    }
