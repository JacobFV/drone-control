from __future__ import annotations

import time
import unittest

from drone_control.controllers.base import SafetyConstraints
from drone_control.controllers.batched_vla import BatchedVLAController, BatchedVLAHub
from drone_control.coordinator.guidance import GuidanceBus, apply_tool_calls
from drone_control.perception.state import PoseEstimate
from drone_control.runtime.events import DroneObservation
from drone_control.runtime.manager import RuntimeManager


def obs_at(drone_id: str, pos) -> DroneObservation:
    return DroneObservation(
        timestamp=0.0,
        drone_id=drone_id,
        link_state="dry_run",
        pose=PoseEstimate(translation=tuple(pos), rotation_xyzw=(0, 0, 0, 1), confidence=0.9, quality="tracking"),
        confidence=0.9,
    )


class GuidanceBusTest(unittest.TestCase):
    def test_target_resolves(self) -> None:
        bus = GuidanceBus()
        bus.set_target("a", (1.0, 2.0, 3.0))
        target, style, policy = bus.resolve("a", (0.0, 0.0, 0.0))
        self.assertEqual(target, (1.0, 2.0, 3.0))
        self.assertEqual(style, [])
        self.assertIsNone(policy)

    def test_trajectory_advances(self) -> None:
        bus = GuidanceBus()
        bus.set_trajectory("a", [(0, 0, 1), (5, 0, 1), (10, 0, 1)])
        # at waypoint 0 -> advances to 1
        t, _, _ = bus.resolve("a", (0.0, 0.0, 1.0))
        self.assertEqual(t, (5, 0, 1))
        # near waypoint 1 -> advances to 2
        t, _, _ = bus.resolve("a", (5.0, 0.0, 1.0))
        self.assertEqual(t, (10, 0, 1))
        # last waypoint, no loop -> holds
        t, _, _ = bus.resolve("a", (10.0, 0.0, 1.0))
        self.assertEqual(t, (10, 0, 1))

    def test_style_and_policy(self) -> None:
        bus = GuidanceBus()
        bus.set_style("a", [0.5, -0.2])
        bus.select_policy("a", "aggressive")
        _, style, policy = bus.resolve("a", (0, 0, 0))
        self.assertEqual(style, [0.5, -0.2])
        self.assertEqual(policy, "aggressive")

    def test_apply_tool_calls(self) -> None:
        bus = GuidanceBus()
        results = apply_tool_calls(
            bus,
            [
                {"name": "set_target", "arguments": {"droneId": "a", "x": 1, "y": 2, "z": 3}},
                {"name": "set_trajectory", "arguments": {"droneId": "b", "waypoints": [[0, 0, 1], [1, 1, 1]], "loop": True}},
                {"name": "set_style", "arguments": {"droneId": "a", "style": [1.0]}},
                {"name": "select_policy", "arguments": {"droneId": "b", "policyId": "scout"}},
                {"name": "bogus", "arguments": {"droneId": "a"}},
            ],
        )
        self.assertTrue(results[0]["ok"])
        self.assertFalse(results[4]["ok"])
        snap = bus.snapshot()
        self.assertEqual(snap["a"]["target"], [1.0, 2.0, 3.0])
        self.assertTrue(snap["b"]["loop"])
        self.assertEqual(snap["b"]["policyId"], "scout")


class GuidanceConditioningTest(unittest.TestCase):
    def test_controller_payload_carries_guidance(self) -> None:
        captured: list[dict] = []

        def model(payloads):
            captured.extend(payloads)
            return [
                {"droneId": p["droneId"], "action": {"roll": 128, "pitch": 128, "throttle": 128, "yaw": 128}, "confidence": 0.5, "reason": "t"}
                for p in payloads
            ]

        bus = GuidanceBus()
        bus.set_target("a", (5.0, 0.0, 2.0))
        bus.set_style("a", [0.3, 0.4])
        bus.select_policy("a", "scout")
        hub = BatchedVLAHub(model, max_wait_seconds=0.05)
        ctrl = BatchedVLAController(hub=hub, drone_id="a", guidance_bus=bus, wait_seconds=0.5)
        ctrl.step(obs_at("a", (1.0, 0.0, 2.0)), [], SafetyConstraints(armed=True))
        hub.close()
        self.assertEqual(len(captured), 1)
        p = captured[0]
        self.assertEqual(p["goalRel"], [4.0, 0.0, 0.0])  # target - pos
        self.assertEqual(p["style"], [0.3, 0.4])
        self.assertEqual(p["policyId"], "scout")


class PolicyGroupingTest(unittest.TestCase):
    def test_batch_groups_by_policy(self) -> None:
        manager = RuntimeManager()
        groups: list[str] = []
        original = manager._run_policy_group

        def spy(policy_id, group):
            groups.append(policy_id)
            return original(policy_id, group)

        manager._run_policy_group = spy  # type: ignore[assignment]
        payloads = [
            {"droneId": "a", "policyId": None},
            {"droneId": "b", "policyId": None},
            {"droneId": "c", "policyId": "scout"},
        ]
        results = manager._batch_model(payloads)
        # Two groups: default (a,b) and scout (c). All drones present in results.
        self.assertEqual(sorted(groups), ["default", "scout"])
        self.assertEqual({r["droneId"] for r in results}, {"a", "b", "c"})


if __name__ == "__main__":
    unittest.main()
