from __future__ import annotations

import os
import pty
import select
import threading
import time
import unittest

from drone_control.config import load_config
from drone_control.transport import (
    ESP_FRAME_HEADER,
    ESP_FRAME_MAGIC,
    ESP_FRAME_VERSION,
    ESP_MSG_CONFIG,
    ESP_MSG_SEND,
    ESP_MSG_STATUS,
    EspSerialDroneLink,
    EspSerialTarget,
    _crc16_ccitt,
)


def build_frame(msg_type: int, seq: int, payload: bytes) -> bytes:
    header = ESP_FRAME_HEADER.pack(ESP_FRAME_MAGIC, ESP_FRAME_VERSION, msg_type, seq, len(payload))
    return header + payload + _crc16_ccitt(header + payload).to_bytes(2, "little")


def pop_frame(buffer: bytearray) -> tuple[int, int, bytes] | None:
    start = buffer.find(ESP_FRAME_MAGIC)
    if start < 0:
        del buffer[:-1]
        return None
    if start:
        del buffer[:start]
    if len(buffer) < ESP_FRAME_HEADER.size:
        return None
    _magic, version, msg_type, seq, payload_len = ESP_FRAME_HEADER.unpack(buffer[:ESP_FRAME_HEADER.size])
    if version != ESP_FRAME_VERSION:
        del buffer[:2]
        return None
    frame_len = ESP_FRAME_HEADER.size + payload_len + 2
    if len(buffer) < frame_len:
        return None
    frame = bytes(buffer[:frame_len])
    del buffer[:frame_len]
    if _crc16_ccitt(frame[:-2]) != int.from_bytes(frame[-2:], "little"):
        return None
    return msg_type, seq, frame[ESP_FRAME_HEADER.size:-2]


class EspSerialDroneLinkTest(unittest.TestCase):
    def test_bridge_resends_config_and_forwards_packets_after_ready(self) -> None:
        master_fd, slave_fd = pty.openpty()
        slave_name = os.ttyname(slave_fd)
        os.close(slave_fd)
        received: list[tuple[int, bytes]] = []
        done = threading.Event()

        def fake_esp() -> None:
            buffer = bytearray()
            config_count = 0
            os.write(master_fd, b"ESP-ROM boot noise\n")
            deadline = time.monotonic() + 4.0
            while time.monotonic() < deadline and not done.is_set():
                ready, _, _ = select.select([master_fd], [], [], 0.1)
                if ready:
                    try:
                        buffer.extend(os.read(master_fd, 4096))
                    except OSError:
                        time.sleep(0.02)
                        continue
                while True:
                    frame = pop_frame(buffer)
                    if frame is None:
                        break
                    msg_type, _seq, payload = frame
                    received.append((msg_type, payload))
                    if msg_type == ESP_MSG_CONFIG:
                        config_count += 1
                        if config_count >= 2:
                            os.write(master_fd, b"more-noise")
                            os.write(master_fd, build_frame(ESP_MSG_STATUS, 7, b"READY 192.168.1.50"))
                    if msg_type == ESP_MSG_SEND:
                        done.set()
                        return

        thread = threading.Thread(target=fake_esp, daemon=True)
        thread.start()
        try:
            link = EspSerialDroneLink(
                EspSerialTarget(
                    port=slave_name,
                    drone_ssid="WIFI_8K-test",
                    drone_ip="192.168.1.1",
                    drone_port=7099,
                    connect_timeout=3.0,
                ),
                timeout=0.05,
            )
            try:
                link.send(b"\x03\x66\x80\x80\x80\x80\x00\x00\x99")
            finally:
                link.close()
            self.assertTrue(done.wait(1.0))
        finally:
            done.set()
            thread.join(timeout=1.0)
            os.close(master_fd)

        config_payloads = [payload for msg_type, payload in received if msg_type == ESP_MSG_CONFIG]
        send_payloads = [payload for msg_type, payload in received if msg_type == ESP_MSG_SEND]
        self.assertGreaterEqual(len(config_payloads), 2)
        self.assertEqual(config_payloads[-1], b"WIFI_8K-test\x00\x00192.168.1.1\x007099")
        self.assertEqual(send_payloads, [b"\x03\x66\x80\x80\x80\x80\x00\x00\x99"])

    def test_mixed_link_config_loads(self) -> None:
        configs = load_config("config/drones.example.json")
        self.assertEqual([cfg.link_type for cfg in configs], ["esp_serial", "esp_serial", "udp"])
        self.assertEqual(configs[0].serial_port, "/dev/ttyUSB0")
        self.assertEqual(configs[2].iface, "wlan2")


if __name__ == "__main__":
    unittest.main()
