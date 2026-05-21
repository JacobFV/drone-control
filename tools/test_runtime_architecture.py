from __future__ import annotations

import time
import unittest
import json
import tempfile
import threading
from pathlib import Path
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from drone_control.actions import DroneAction
from drone_control.config import DroneConfig
from drone_control.controllers.local_vla import LocalVLAClient, LocalVLAConfig
from drone_control.controllers.vla import VLAController, parse_vla_output
from drone_control.coordinator.http_vlm import HttpVLMClient, HttpVLMConfig
from drone_control.coordinator.scheduler import CoordinatorScheduler
from drone_control.coordinator.tasks import Mission
from drone_control.coordinator.vlm import VLMCoordinator
from drone_control.controllers.base import SafetyConstraints
from drone_control.controllers.manual import ManualController
from drone_control.controllers.safety import SafetyController
from drone_control.controllers.scripted import neutral_controller, takeoff_controller
from drone_control.perception.frames import StaticFrameSource
from drone_control.perception.imu import FileImuSource, imu_samples_from_csv, imu_samples_from_jsonl
from drone_control.perception.pipeline import PerceptionPipeline
from drone_control.perception.state import FrameMetadata, ImuSample, MapSummary, PoseEstimate
from drone_control.protocols import make_protocol
from drone_control.runtime.drone_runtime import DroneRuntime
from drone_control.runtime.events import DroneObservation
from drone_control.runtime.manager import RuntimeManager, RuntimeManagerConfig
from drone_control.runtime.replay import load_replay_trace


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


class StaticImuSource:
    def latest(self) -> ImuSample | None:
        return ImuSample(timestamp=1.0, acceleration=(0.0, 0.0, 9.81), gyro=(0.0, 0.0, 0.0))


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
            imu_source=StaticImuSource(),
            map_summary=MapSummary(state="artifact", record_id="map-a", keyframes=3),
            dry_run=True,
        )
        runtime.step_once()
        observation = runtime.snapshot().observation
        self.assertEqual(observation.latest_frame.index if observation.latest_frame else None, 3)
        self.assertEqual(observation.pose.frame_index if observation.pose else None, 7)
        self.assertIsNotNone(observation.imu)
        self.assertEqual(observation.map_summary.record_id if observation.map_summary else None, "map-a")
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

    def test_mission_assignments_apply_safety_constraints(self) -> None:
        manager = RuntimeManager(config=RuntimeManagerConfig(dry_run=True, enable_io=False, control_hz=40))
        manager.configure_drones([DroneConfig(id="drone-a"), DroneConfig(id="drone-b")])
        manager.start_all()
        try:
            time.sleep(0.05)
            progress = CoordinatorScheduler().start(Mission("mission-a", "inspection"))
            scheduler = CoordinatorScheduler(tick_hz=100)
            scheduler.start(Mission("mission-a", "inspection"))
            progress = scheduler.step(manager.snapshot_objects())
            manager.apply_mission_progress(progress)
            drones = {item["droneId"]: item for item in manager.snapshots()["drones"]}
        finally:
            manager.stop_all()
        self.assertLessEqual(drones["drone-a"]["constraints"]["maxThrottle"], 160)
        self.assertIn("assignmentRole", drones["drone-a"]["constraints"]["metadata"])

    def test_coordinator_blocks_missing_and_flags_low_confidence(self) -> None:
        scheduler = CoordinatorScheduler(tick_hz=100)
        scheduler.start(Mission("mission-empty", "inspection"))
        self.assertEqual(scheduler.step([]).state, "blocked")
        runtime = DroneRuntime(
            drone_id="drone-low",
            protocol=make_protocol("wifi_8k_prefixed_short"),
            link=None,
            controller=neutral_controller(),
            constraints=SafetyConstraints(armed=True, require_heartbeat=False),
            dry_run=True,
        )
        runtime.step_once()
        scheduler.start(Mission("mission-low", "inspection"))
        progress = scheduler.step([runtime.snapshot()])
        self.assertTrue(any("low_confidence" in note for note in progress.notes))
        self.assertEqual(progress.assignments[0].constraints.max_throttle, 120)

    def test_vla_replay_and_bad_schema_faults(self) -> None:
        trace = load_replay_trace(Path("tools/fixtures/runtime_replay.json"))
        seen_payloads = []

        def model(payload):
            seen_payloads.append(payload)
            return {
                "action": {"roll": 128, "pitch": 132, "throttle": 130, "yaw": 128},
                "confidence": 0.9,
                "reason": "fixture_follow",
            }

        controller = VLAController(model_step=model, mission_context=trace.mission)
        request = controller.step(trace.observations[-1], trace.observations[:-1], SafetyConstraints(require_heartbeat=False))
        self.assertEqual(request.action.pitch, 132)
        self.assertEqual(seen_payloads[0]["mission"]["id"], "fixture-inspection")
        self.assertIn("recentActions", seen_payloads[0])
        with self.assertRaises(ValueError):
            parse_vla_output({"action": {"roll": 999}, "confidence": 0.5, "reason": "bad"})
        bad = VLAController(model_step=lambda _payload: {"action": {"roll": "bad"}, "confidence": 0.5, "reason": "bad"})
        fault = bad.step(trace.observations[-1], [], SafetyConstraints(require_heartbeat=False))
        self.assertEqual(fault.action, DroneAction.motor_stop())
        self.assertIsNotNone(fault.fault)

    def test_vlm_rejects_unknown_drone_assignments(self) -> None:
        coordinator = VLMCoordinator(
            model_step=lambda _payload: {
                "state": "running",
                "assignments": [{"droneId": "missing", "role": "lead", "task": "survey"}],
            }
        )
        progress = coordinator.step(Mission("mission-vlm", "inspection"), [{"droneId": "drone-a"}])
        self.assertEqual(progress.state, "faulted")
        self.assertTrue(progress.notes)

    def test_local_vla_client_executes_json_lines_process(self) -> None:
        command = [
            "python3",
            "-u",
            "-c",
            (
                "import json,sys\n"
                "for line in sys.stdin:\n"
                " p=json.loads(line)\n"
                " print(json.dumps({'action': {'roll': 128, 'pitch': 129, 'throttle': 130, 'yaw': 128}, "
                "'confidence': 0.7, 'reason': 'local'}), flush=True)\n"
            ),
        ]
        client = LocalVLAClient(LocalVLAConfig(command=command, timeout_seconds=1.0))
        try:
            result = client.step({"observation": {"droneId": "drone-a"}})
        finally:
            client.close()
        action, confidence, reason = parse_vla_output(result)
        self.assertEqual(action.throttle, 130)
        self.assertEqual(confidence, 0.7)
        self.assertEqual(reason, "local")

    def test_http_vlm_client_executes_post_contract(self) -> None:
        class Handler(BaseHTTPRequestHandler):
            def do_POST(self) -> None:
                length = int(self.headers.get("Content-Length", "0"))
                payload = json.loads(self.rfile.read(length).decode())
                self.server.payload = payload  # type: ignore[attr-defined]
                body = json.dumps({
                    "state": "running",
                    "assignments": [{
                        "droneId": "drone-a",
                        "role": "lead",
                        "task": "survey",
                        "constraints": {"maxThrottle": 144, "minConfidence": 0.2},
                    }],
                    "notes": ["ok"],
                }).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, fmt: str, *args: object) -> None:
                return

        server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            client = HttpVLMClient(HttpVLMConfig(endpoint=f"http://127.0.0.1:{server.server_port}/vlm"))
            result = client.step({"mission": {"id": "m"}, "drones": [{"droneId": "drone-a"}]})
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=1.0)
        self.assertEqual(result["assignments"][0]["constraints"]["maxThrottle"], 144)

    def test_imu_file_extractors_feed_perception_pipeline(self) -> None:
        with tempfile.TemporaryDirectory() as tempdir:
            root = Path(tempdir)
            jsonl = root / "imu.jsonl"
            csv_path = root / "imu.csv"
            jsonl.write_text('{"timestamp": 1.0, "ax": 0, "ay": 0, "az": 9.81, "gx": 0, "gy": 0, "gz": 0}\n')
            csv_path.write_text("timestamp,ax,ay,az,gx,gy,gz\n2.0,0.1,0,9.8,0,0.01,0\n")
            self.assertEqual(len(imu_samples_from_jsonl(jsonl)), 1)
            self.assertEqual(len(imu_samples_from_csv(csv_path)), 1)
            pipeline = PerceptionPipeline(
                frame_source=StaticFrameSource(FrameMetadata(index=1)),
                pose_estimator=StaticPoseEstimator(),
                imu_source=FileImuSource(csv_path),
                map_summary=MapSummary(state="artifact", record_id="map"),
            )
            status = pipeline.status()
            self.assertIsNotNone(status.imu)
            self.assertGreaterEqual(status.confidence, 0.9)


if __name__ == "__main__":
    unittest.main()
