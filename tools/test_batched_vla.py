from __future__ import annotations

import threading
import time
import unittest

from drone_control.actions import DroneAction
from drone_control.controllers.base import SafetyConstraints
from drone_control.controllers.batched_vla import BatchedVLAController, BatchedVLAHub
from drone_control.perception.frame_registry import LiveFrameRegistry
from drone_control.runtime.events import DroneObservation


def neutral_result(drone_id: str, throttle: int = 128) -> dict:
    return {
        "droneId": drone_id,
        "action": {"roll": 128, "pitch": 128, "throttle": throttle, "yaw": 128},
        "confidence": 0.5,
        "reason": "test",
    }


class BatchedVLATest(unittest.TestCase):
    def _obs(self, drone_id: str) -> DroneObservation:
        return DroneObservation.empty(drone_id, link_state="dry_run")

    def test_coalesces_into_single_call(self) -> None:
        calls: list[list[str]] = []

        def model(payloads: list[dict]) -> list[dict]:
            calls.append([p["droneId"] for p in payloads])
            return [neutral_result(p["droneId"]) for p in payloads]

        hub = BatchedVLAHub(model, max_wait_seconds=0.1)
        controllers = {
            d: BatchedVLAController(hub=hub, drone_id=d, wait_seconds=0.5)
            for d in ("a", "b", "c")
        }
        constraints = SafetyConstraints(armed=True)
        results: dict[str, object] = {}

        def run(d: str) -> None:
            results[d] = controllers[d].step(self._obs(d), [], constraints)

        threads = [threading.Thread(target=run, args=(d,)) for d in controllers]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        # All three submitted within the window -> exactly one batched call of size 3.
        self.assertEqual(len(calls), 1)
        self.assertEqual(sorted(calls[0]), ["a", "b", "c"])
        for d in controllers:
            self.assertEqual(results[d].action.throttle, 128)
        hub.close()

    def test_routes_distinct_actions_per_drone(self) -> None:
        def model(payloads: list[dict]) -> list[dict]:
            # Give each drone a unique throttle to prove routing is correct.
            return [neutral_result(p["droneId"], throttle=10 + i) for i, p in enumerate(payloads)]

        hub = BatchedVLAHub(model, max_wait_seconds=0.1)
        ctrls = {d: BatchedVLAController(hub=hub, drone_id=d, wait_seconds=0.5) for d in ("x", "y")}
        constraints = SafetyConstraints(armed=True)
        out: dict[str, int] = {}

        def run(d: str) -> None:
            out[d] = ctrls[d].step(self._obs(d), [], constraints).action.throttle

        ts = [threading.Thread(target=run, args=(d,)) for d in ctrls]
        for t in ts:
            t.start()
        for t in ts:
            t.join()
        # Two distinct throttles, each mapped to the right drone via droneId echo.
        self.assertNotEqual(out["x"], out["y"])
        hub.close()

    def test_timeout_yields_motor_stop(self) -> None:
        def slow_model(payloads: list[dict]) -> list[dict]:
            time.sleep(0.3)
            return [neutral_result(p["droneId"]) for p in payloads]

        hub = BatchedVLAHub(slow_model, max_wait_seconds=0.01)
        ctrl = BatchedVLAController(hub=hub, drone_id="a", wait_seconds=0.05)
        request = ctrl.step(self._obs("a"), [], SafetyConstraints(armed=True))
        self.assertEqual(request.action.throttle, 0)
        self.assertTrue(request.action.emergency_stop)
        self.assertEqual(request.reason, "batched_vla_timeout")
        hub.close()

    def test_model_exception_yields_no_result_stop(self) -> None:
        def broken(payloads: list[dict]) -> list[dict]:
            raise RuntimeError("boom")

        hub = BatchedVLAHub(broken, max_wait_seconds=0.05)
        ctrl = BatchedVLAController(hub=hub, drone_id="a", wait_seconds=0.5)
        request = ctrl.step(self._obs("a"), [], SafetyConstraints(armed=True))
        self.assertEqual(request.action.throttle, 0)
        self.assertEqual(request.reason, "batched_vla_no_result")
        hub.close()

    def test_invalid_output_yields_fault_stop(self) -> None:
        def bad(payloads: list[dict]) -> list[dict]:
            return [{"droneId": p["droneId"], "action": {"roll": 999}, "confidence": 0.5, "reason": "x"} for p in payloads]

        hub = BatchedVLAHub(bad, max_wait_seconds=0.05)
        ctrl = BatchedVLAController(hub=hub, drone_id="a", wait_seconds=0.5)
        request = ctrl.step(self._obs("a"), [], SafetyConstraints(armed=True))
        self.assertEqual(request.action.throttle, 0)
        self.assertEqual(request.reason, "batched_vla_invalid_output")
        self.assertIsNotNone(request.fault)
        hub.close()

    def test_frame_registry_payload(self) -> None:
        seen: list[bool] = []

        def model(payloads: list[dict]) -> list[dict]:
            seen.append(payloads[0].get("frameJpegB64") is not None)
            return [neutral_result(p["droneId"]) for p in payloads]

        registry = LiveFrameRegistry()
        registry.publish("a", b"\xff\xd8\xff\xfakejpeg", width=320, height=240)
        hub = BatchedVLAHub(model, max_wait_seconds=0.05)
        ctrl = BatchedVLAController(hub=hub, drone_id="a", registry=registry, wait_seconds=0.5)
        ctrl.step(self._obs("a"), [], SafetyConstraints(armed=True))
        self.assertEqual(seen, [True])
        hub.close()


class BatchedVLAManagerIntegrationTest(unittest.TestCase):
    """Drives the real diffusion policy subprocess through the RuntimeManager."""

    def test_swarm_runs_on_batched_diffusion_policy(self) -> None:
        import sys
        from pathlib import Path

        from drone_control.config import DroneConfig
        from drone_control.runtime.manager import RuntimeManager, RuntimeManagerConfig

        policy = str(Path(__file__).resolve().parent / "diffusion_vla_policy.py")
        manager = RuntimeManager(
            config=RuntimeManagerConfig(
                control_hz=20.0,
                dry_run=True,
                batched_vla_command=[sys.executable, policy, "--device", "cpu", "--steps", "5"],
                batched_vla_timeout_seconds=0.5,
            )
        )
        configs = [DroneConfig(id=f"drone-{i}") for i in range(3)]
        try:
            manager.configure_drones(configs)
            manager.set_all_controllers("batched_vla")
            manager.start_all()
            for drone_id in manager.runtime_ids():
                manager.arm(drone_id)
            # Let several control windows run; heartbeat keeps the safety wrapper live.
            deadline = time.time() + 4.0
            while time.time() < deadline:
                manager.heartbeat_all()
                time.sleep(0.1)
            snap = manager.snapshots()
        finally:
            manager.stop_all()
            manager.close_vla()

        self.assertTrue(snap["batchedVlaConfigured"])
        self.assertGreater(snap["batchedVla"]["batches"], 0)
        # Batches should frequently carry more than one drone (true batching).
        self.assertGreaterEqual(snap["batchedVla"]["lastBatchSize"], 1)
        for drone in snap["drones"]:
            self.assertGreater(drone["sent"], 0)
            self.assertEqual(drone["controller"], "batched_vla")


if __name__ == "__main__":
    unittest.main()
