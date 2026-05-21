from __future__ import annotations

import queue
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Any

from drone_control.actions import DroneAction
from drone_control.controllers.base import DroneController, SafetyConstraints
from drone_control.controllers.safety import SafetyController
from drone_control.perception.estimator import NullPoseEstimator, PoseEstimator
from drone_control.perception.frames import FrameMetadataSource, StaticFrameSource
from drone_control.perception.state import MapSummary
from drone_control.protocols import PacketProtocol
from drone_control.transport import DroneLink

from .events import ActionEvent, DroneObservation, LinkStatusEvent, ObservationEvent, RuntimeErrorEvent, RuntimeEvent


@dataclass(slots=True)
class DroneRuntimeSnapshot:
    drone_id: str
    running: bool
    controller: str
    link_type: str
    link_state: str
    sent: int
    errors: int
    dry_run: bool
    observation: DroneObservation
    safety: dict[str, object]
    constraints: dict[str, Any]
    last_action: DroneAction | None

    def as_dict(self) -> dict[str, Any]:
        return {
            "droneId": self.drone_id,
            "running": self.running,
            "controller": self.controller,
            "linkType": self.link_type,
            "linkState": self.link_state,
            "sent": self.sent,
            "errors": self.errors,
            "dryRun": self.dry_run,
            "observation": self.observation.as_dict(),
            "safety": self.safety,
            "constraints": self.constraints,
            "lastAction": _action_dict(self.last_action),
        }


class DroneRuntime:
    def __init__(
        self,
        *,
        drone_id: str,
        protocol: PacketProtocol,
        link: DroneLink | None,
        controller: DroneController,
        constraints: SafetyConstraints | None = None,
        frame_source: FrameMetadataSource | None = None,
        pose_estimator: PoseEstimator | None = None,
        map_summary: MapSummary | None = None,
        control_hz: float = 20.0,
        dry_run: bool = False,
        link_type: str = "unknown",
    ) -> None:
        self.drone_id = drone_id
        self.protocol = protocol
        self.link = link
        self.controller = SafetyController(controller, constraints)
        self.frame_source = frame_source or StaticFrameSource()
        self.pose_estimator = pose_estimator or NullPoseEstimator()
        self.map_summary = map_summary or MapSummary()
        self.control_hz = max(1.0, float(control_hz))
        self.link_type = link_type
        self.dry_run = dry_run
        self.sent = 0
        self.errors = 0
        self.last_error: str | None = None
        self.link_state = "dry_run" if dry_run else ("connected" if link is not None else "missing")
        self._running = False
        self._thread: threading.Thread | None = None
        self._lock = threading.RLock()
        self._events: queue.Queue[RuntimeEvent] = queue.Queue()
        self._history: deque[DroneObservation] = deque(maxlen=120)
        self._action_history: deque[DroneAction] = deque(maxlen=120)
        self._last_observation = DroneObservation.empty(drone_id, link_state=self.link_state)

    def start(self) -> None:
        with self._lock:
            if self._running:
                return
            self._running = True
            self._thread = threading.Thread(target=self._loop, name=f"runtime-{self.drone_id}", daemon=True)
            self._thread.start()
            self._emit(LinkStatusEvent(self.drone_id, self.link_state, sent=self.sent, errors=self.errors))

    def stop(self) -> None:
        with self._lock:
            self._running = False
            thread = self._thread
        if thread is not None:
            thread.join(timeout=1.0)
        self._send_stop_burst()
        if self.link is not None:
            self.link.close()
        self.link_state = "stopped"
        self._emit(LinkStatusEvent(self.drone_id, self.link_state, sent=self.sent, errors=self.errors))

    def step_once(self) -> DroneAction:
        observation = self._observe()
        self._sync_controller_context()
        request = self.controller.step(observation, list(self._history))
        action = request.action.sanitized()
        self._send_action(action)
        self._action_history.append(action)
        self._record_controller_action(action)
        self._emit(ObservationEvent(self.drone_id, observation))
        self._emit(ActionEvent(self.drone_id, action, controller=self.controller.inner.name))
        return action

    def set_controller(self, controller: DroneController) -> None:
        with self._lock:
            self.controller.set_controller(controller)

    def snapshot(self) -> DroneRuntimeSnapshot:
        with self._lock:
            return DroneRuntimeSnapshot(
                drone_id=self.drone_id,
                running=self._running,
                controller=self.controller.inner.name,
                link_type=self.link_type,
                link_state=self.link_state,
                sent=self.sent,
                errors=self.errors,
                dry_run=self.dry_run,
                observation=self._last_observation,
                safety=self.controller.state.as_dict(),
                constraints=self.controller.constraints.as_dict(),
                last_action=self._action_history[-1] if self._action_history else None,
            )

    def drain_events(self, limit: int = 100) -> list[RuntimeEvent]:
        events: list[RuntimeEvent] = []
        for _ in range(max(0, limit)):
            try:
                events.append(self._events.get_nowait())
            except queue.Empty:
                break
        return events

    def arm(self) -> None:
        self.controller.arm()

    def disarm(self) -> None:
        self.controller.disarm()

    def heartbeat(self) -> None:
        self.controller.heartbeat()

    def clear_fault(self) -> None:
        self.controller.clear_fault()

    def update_constraints(self, constraints: SafetyConstraints) -> None:
        with self._lock:
            constraints = SafetyConstraints(
                armed=self.controller.state.armed,
                max_throttle=constraints.max_throttle,
                require_heartbeat=constraints.require_heartbeat,
                heartbeat_timeout_seconds=constraints.heartbeat_timeout_seconds,
                throttle_slew_per_second=constraints.throttle_slew_per_second,
                command_hz=constraints.command_hz,
                metadata=constraints.metadata,
            )
            self.controller.constraints = constraints

    def _loop(self) -> None:
        interval = 1.0 / self.control_hz
        while True:
            with self._lock:
                running = self._running
            if not running:
                return
            started = time.monotonic()
            try:
                self.step_once()
            except Exception as exc:  # defensive runtime boundary
                self.errors += 1
                self.last_error = str(exc)
                self._emit(RuntimeErrorEvent(self.drone_id, str(exc)))
            elapsed = time.monotonic() - started
            time.sleep(max(0.0, interval - elapsed))

    def _observe(self) -> DroneObservation:
        frame = self.frame_source.latest()
        pose = self.pose_estimator.latest_pose()
        confidence = max(
            0.0,
            min(
                1.0,
                (0.35 if frame else 0.0)
                + (pose.confidence if pose else 0.0)
                + (0.15 if self.link_state in {"connected", "dry_run"} else 0.0),
            ),
        )
        observation = DroneObservation(
            timestamp=time.time(),
            drone_id=self.drone_id,
            link_state=self.link_state,
            latest_frame=frame,
            pose=pose,
            map_summary=self.map_summary,
            battery=None,
            confidence=confidence,
        )
        self._last_observation = observation
        self._history.append(observation)
        return observation

    def _send_action(self, action: DroneAction) -> None:
        if self.dry_run:
            self.sent += 1
            return
        if self.link is None:
            self.errors += 1
            self.link_state = "missing"
            raise RuntimeError("runtime has no link")
        try:
            self.link.send(self.protocol.build(action))
            self.sent += 1
            self.last_error = None
            self.link_state = "connected"
        except OSError as exc:
            self.errors += 1
            self.last_error = str(exc)
            self.link_state = "error"
            self._emit(LinkStatusEvent(self.drone_id, self.link_state, sent=self.sent, errors=self.errors))
            raise

    def _send_stop_burst(self) -> None:
        if self.dry_run or self.link is None:
            return
        packet = self.protocol.build(DroneAction.motor_stop())
        for _ in range(3):
            try:
                self.link.send(packet)
            except OSError:
                break
            time.sleep(0.02)

    def _emit(self, event: RuntimeEvent) -> None:
        self._events.put(event)

    def _sync_controller_context(self) -> None:
        setter = getattr(self.controller.inner, "set_recent_actions", None)
        if callable(setter):
            setter(list(self._action_history))

    def _record_controller_action(self, action: DroneAction) -> None:
        recorder = getattr(self.controller.inner, "record_action", None)
        if callable(recorder):
            recorder(action)


def _action_dict(action: DroneAction | None) -> dict[str, Any] | None:
    if action is None:
        return None
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
