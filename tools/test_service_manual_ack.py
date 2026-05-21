from __future__ import annotations

import tempfile
import time
import unittest
from pathlib import Path

from drone_control.actions import DroneAction
from drone_control.manual_control import ManualControlConfig, ManualControlState
from drone_control.service import ControlStationServer
from drone_control.store import ControlStationStore


class AlwaysSendingTransport:
    def __init__(self) -> None:
        self.sent = 0

    def send(self, action: object | None) -> bool:
        if action is None:
            return False
        self.sent += 1
        return True

    def close(self) -> None:
        pass

    def status(self) -> object:
        raise AssertionError("status is not used by this test")

    def config_dict(self) -> dict[str, object]:
        return {}


class ManualAckIntegrationTest(unittest.TestCase):
    def test_successful_transport_send_acks_manual_state_machine(self) -> None:
        with tempfile.TemporaryDirectory() as tempdir:
            root = Path(tempdir)
            store = ControlStationStore(root / "test.sqlite3", root / "blobs", Path.cwd())
            server = ControlStationServer(("127.0.0.1", 0), store)
            server.manual_loop_running = False
            server.manual_thread.join(timeout=1.0)
            server.manual_transport = AlwaysSendingTransport()  # type: ignore[assignment]
            server.manual.config = ManualControlConfig(ack_timeout_seconds=0.05, command_hz=100.0)
            try:
                server.manual.arm()
                action = server.manual.tick()
                self.assertIsNotNone(action)
                self.assertTrue(server.send_manual_action(action))
                time.sleep(0.08)
                self.assertIsNotNone(server.manual.tick())
                self.assertNotEqual(server.manual.state, ManualControlState.FAULTED)
            finally:
                server.server_close()
                store.close()


if __name__ == "__main__":
    unittest.main()
