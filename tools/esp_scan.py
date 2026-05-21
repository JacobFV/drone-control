from __future__ import annotations

import argparse
import os
import select
import termios
import time
import tty

from drone_control.transport import (
    ESP_FRAME_HEADER,
    ESP_FRAME_MAGIC,
    ESP_FRAME_VERSION,
    ESP_MSG_ERROR,
    ESP_MSG_SCAN,
    ESP_MSG_STATUS,
    _crc16_ccitt,
)


def main() -> int:
    parser = argparse.ArgumentParser(description="Scan Wi-Fi networks through the ESP32 drone bridge.")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--timeout", type=float, default=12.0)
    args = parser.parse_args()

    fd = os.open(args.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        configure_port(fd, args.baud)
        write_frame(fd, ESP_MSG_SCAN, 0, b"")
        deadline = time.monotonic() + args.timeout
        buffer = bytearray()
        while time.monotonic() < deadline:
            frame = pop_frame(buffer)
            if frame is not None:
                msg_type, _seq, payload = frame
                text = payload.decode(errors="replace")
                if msg_type == ESP_MSG_STATUS:
                    print(text)
                    if text.startswith("SCAN_DONE"):
                        return 0
                elif msg_type == ESP_MSG_ERROR:
                    print(f"ERROR {text}")
                    return 1
                continue
            ready, _, _ = select.select([fd], [], [], max(0.0, min(0.25, deadline - time.monotonic())))
            if ready:
                try:
                    buffer.extend(os.read(fd, 4096))
                except BlockingIOError:
                    pass
        print("ERROR scan timeout")
        return 1
    finally:
        os.close(fd)


def configure_port(fd: int, baud: int) -> None:
    tty.setraw(fd)
    attrs = termios.tcgetattr(fd)
    speed = getattr(termios, f"B{baud}")
    attrs[4] = speed
    attrs[5] = speed
    attrs[2] |= termios.CLOCAL | termios.CREAD
    attrs[2] &= ~getattr(termios, "CRTSCTS", 0)
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIOFLUSH)


def write_frame(fd: int, msg_type: int, seq: int, payload: bytes) -> None:
    header = ESP_FRAME_HEADER.pack(ESP_FRAME_MAGIC, ESP_FRAME_VERSION, msg_type, seq, len(payload))
    frame = header + payload + _crc16_ccitt(header + payload).to_bytes(2, "little")
    os.write(fd, frame)


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


if __name__ == "__main__":
    raise SystemExit(main())
