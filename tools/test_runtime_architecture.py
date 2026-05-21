from __future__ import annotations

import time
import unittest

from drone_control.actions import DroneAction
from drone_control.config import DroneConfig
from drone_control.controllers.base import SafetyConstraints
from drone_control.controllers.manual import ManualController
from drone_control.controllers.safety import SafetyController
from drone_control.controllers.scripted import neutral_controller, takeoff_controller
from drone_control.perception.frames import StaticFrameSource
from drone_control.perception.state import FrameMetadata, PoseEstimate
from drone_control.protocols import make_protocol
from drone_control.runtime.drone_runtime import DroneRuntime
from drone_control.runtime.events import DroneObservation
from drone_control.runtime.manager import RuntimeManager, RuntimeManagerConfig


class FakeLink:
    def __init__(self) -> None:
        self.packets: list[bytes] = []
        self.closed = False

    def send(self, packet: bytes) -> None:
        self.packets.append(packet)

    def recv_once(self, size: int = 2048) -> tuple[bytes, tuple[str, int]] | None:
        return None

    def close(self) -> None:
        self.closed = True


class StaticPoseEstimator:
    def latest_pose(self) -> PoseEstimate | None:
        return PoseEstimate(frame_index=7, translation=(1.0, 2.0, 3.0), rotation_xyzw=(0.0, 0.0, 0.0, 1.0), confidence=0.5)


class RuntimeArchitectureTest(unittest.TestCase):
    def test_runtime_emits_observation_and_action_without_camera(self) -> None:
        link = FakeLink()
        runtime = DroneRuntime(
            drone_id="drone-test",
            protocol=make_protocol("wifi_8k_prefixed_short"),
            link=link,
            controller=neutral_controller(),
            constraints=SafetyConstraints(armed=True, require_heartbeat=False),
        )
        action = runtime.step_once()
        events = [event.as_dict() for event in runtime.drain_events()]
        self.assertEqual(action, DroneAction.neutral())
        self.assertEqual(len(link.packets), 1)
        self.assertTrue(any(event["type"] == "observation" for event in events))
        self.assertTrue(any(event["type"] == "action" for event in events))
        snapshot = runtime.snapshot().as_dict()
        self.assertEqual(snapshot["observation"]["latestFrame"], None)

    def test_runtime_snapshot_includes_frame_and_pose_confidence(self) -> None:
        runtime = DroneRuntime(
            drone_id="drone-vision",
            protocol=make_protocol("wifi_8k_prefixed_short"),
            link=None,
            controller=neutral_controller(),
            constraints=SafetyConstraints(armed=True, require_heartbeat=False),
            frame_source=StaticFrameSource(FrameMetadata(index=3, width=640, height=360, source="fixture")),
            pose_estimator=StaticPoseEstimator(),
            dry_run=True,
        )
        runtime.step_once()
        observation = runtime.snapshot().observation
        self.assertEqual(observation.latest_frame.index if observation.latest_frame else None, 3)
        self.assertEqual(observation.pose.frame_index if observation.pose else None, 7)
        self.assertGreater(observation.confidence, 0.8)

    def test_safety_clamps_controller_output_before_action(self) -> None:
        controller = takeoff_controller(ticks=1, throttle=255)
        safety = SafetyController(
            controller,
            SafetyConstraints(armed=True, max_throttle=140, require_heartbeat=False, throttle_slew_per_second=1000),
        )
        observation = DroneObservation.empty("drone-safe", link_state="dry_run")
        for _ in range(5):
            request = safety.step(observation, [])
            time.sleep(0.002)
        self.assertLessEqual(request.action.throttle, 140)

    def test_manager_runs_two_fake_drones_and_switches_manual(self) -> None:
        manager = RuntimeManager(config=RuntimeManagerConfig(dry_run=True, enable_io=False, control_hz=40))
        manager.configure_drones([
            DroneConfig(id="drone-a", link_type="udp"),
            DroneConfig(id="drone-b", link_type="esp_serial", ssid="WIFI_8K-test", serial_port="/dev/null"),
        ])
        manager.set_controller("drone-a", "manual")
        manager.arm("drone-a")
        manager.heartbeat("drone-a")
        manager.set_manual_axes("drone-a", {"throttle": 90})
        manager.start_all()
        try:
            time.sleep(0.08)
            status = manager.snapshots()
        finally:
            manager.stop_all()
        drones = {item["droneId"]: item for item in status["drones"]}
        self.assertEqual(set(drones), {"drone-a", "drone-b"})
        self.assertGreater(drones["drone-a"]["sent"], 0)
        self.assertGreater(drones["drone-b"]["sent"], 0)
        self.assertEqual(drones["drone-a"]["controller"], "manual")


if __name__ == "__main__":
    unittest.main()
