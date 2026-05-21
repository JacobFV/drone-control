from __future__ import annotations

import os
import select
import socket
import struct
import termios
import time
import tty
from dataclasses import dataclass
from typing import Protocol


SO_BINDTODEVICE = getattr(socket, "SO_BINDTODEVICE", 25)
ESP_FRAME_MAGIC = b"DL"
ESP_FRAME_VERSION = 1
ESP_FRAME_HEADER = struct.Struct("<2sBBHH")
ESP_MAX_PAYLOAD = 2048

ESP_MSG_CONFIG = 0x01
ESP_MSG_SEND = 0x02
ESP_MSG_STATUS = 0x81
ESP_MSG_ACK = 0x82
ESP_MSG_ERROR = 0x83


class DroneLink(Protocol):
    def send(self, packet: bytes) -> None:
        ...

    def recv_once(self, size: int = 2048) -> tuple[bytes, tuple[str, int]] | None:
        ...

    def close(self) -> None:
        ...


@dataclass(slots=True)
class UdpTarget:
    ip: str
    port: int
    iface: str | None = None


@dataclass(slots=True)
class EspSerialTarget:
    port: str
    baud: int = 921600
    drone_ssid: str = ""
    drone_password: str = ""
    drone_ip: str = "192.168.1.1"
    drone_port: int = 7099
    connect_timeout: float = 12.0


class UdpDroneLink:
    def __init__(self, target: UdpTarget, *, bind_device: bool = True, timeout: float = 0.15) -> None:
        self.target = target
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(timeout)
        self.bound_device = False
        if bind_device and target.iface:
            self.bind_to_device(target.iface)

    def bind_to_device(self, iface: str) -> None:
        if os.geteuid() != 0:
            raise PermissionError("SO_BINDTODEVICE requires root; rerun with sudo or use --no-bind-device")
        self.sock.setsockopt(socket.SOL_SOCKET, SO_BINDTODEVICE, iface.encode() + b"\0")
        self.bound_device = True

    def send(self, packet: bytes) -> None:
        self.sock.sendto(packet, (self.target.ip, self.target.port))

    def recv_once(self, size: int = 2048) -> tuple[bytes, tuple[str, int]] | None:
        try:
            return self.sock.recvfrom(size)
        except TimeoutError:
            return None
        except socket.timeout:
            return None

    def close(self) -> None:
        self.sock.close()


class EspSerialDroneLink:
    """
    USB-serial bridge link for one ESP32 associated with one drone AP.

    The PC sends already-built drone control packets to the ESP32. The ESP32
    owns Wi-Fi association and UDP delivery to the drone, so higher-level
    control code can mix this link with direct UDP links in the same swarm.
    """

    def __init__(self, target: EspSerialTarget, *, timeout: float = 0.15) -> None:
        if not target.port:
            raise ValueError("ESP serial port is required")
        if not target.drone_ssid:
            raise ValueError("ESP drone SSID is required")
        self.target = target
        self.timeout = timeout
        self.seq = 0
        self._read_buffer = bytearray()
        self.fd = os.open(target.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            self._configure_port(target.baud)
            self._configure_bridge()
        except Exception:
            os.close(self.fd)
            raise

    def send(self, packet: bytes) -> None:
        self._write_frame(ESP_MSG_SEND, packet)

    def recv_once(self, size: int = 2048) -> tuple[bytes, tuple[str, int]] | None:
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            frame = self._read_frame(deadline)
            if frame is None:
                return None
            msg_type, _seq, payload = frame
            if msg_type == ESP_MSG_STATUS:
                return payload[:size], ("esp32", self.target.drone_port)
        return None

    def close(self) -> None:
        os.close(self.fd)

    def _configure_port(self, baud: int) -> None:
        attrs = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)
        attrs = termios.tcgetattr(self.fd)
        speed = getattr(termios, f"B{baud}", None)
        if speed is None:
            raise ValueError(f"unsupported serial baud rate: {baud}")
        attrs[4] = speed
        attrs[5] = speed
        attrs[2] |= termios.CLOCAL | termios.CREAD
        attrs[2] &= ~getattr(termios, "CRTSCTS", 0)
        attrs[3] &= ~(termios.ICANON | termios.ECHO | termios.ECHOE | termios.ISIG)
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        termios.tcflush(self.fd, termios.TCIOFLUSH)

    def _configure_bridge(self) -> None:
        payload = b"\0".join(
            [
                self.target.drone_ssid.encode(),
                self.target.drone_password.encode(),
                self.target.drone_ip.encode(),
                str(self.target.drone_port).encode(),
            ]
        )
        deadline = time.monotonic() + self.target.connect_timeout
        next_config = 0.0
        last_error = ""
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_config:
                self._write_frame(ESP_MSG_CONFIG, payload)
                next_config = now + 0.5
            frame = self._read_frame(min(deadline, time.monotonic() + 0.1))
            if frame is None:
                continue
            msg_type, _seq, body = frame
            text = body.decode(errors="replace")
            if msg_type == ESP_MSG_STATUS and text.startswith("READY"):
                return
            if msg_type == ESP_MSG_ERROR:
                last_error = text
        detail = f": {last_error}" if last_error else ""
        raise TimeoutError(f"ESP32 bridge did not become ready{detail}")

    def _write_frame(self, msg_type: int, payload: bytes) -> None:
        if len(payload) > ESP_MAX_PAYLOAD:
            raise ValueError(f"ESP frame payload too large: {len(payload)}")
        seq = self.seq & 0xFFFF
        self.seq = (self.seq + 1) & 0xFFFF
        header = ESP_FRAME_HEADER.pack(
            ESP_FRAME_MAGIC,
            ESP_FRAME_VERSION,
            msg_type & 0xFF,
            seq,
            len(payload),
        )
        crc = _crc16_ccitt(header + payload).to_bytes(2, "little")
        self._write_all(header + payload + crc)

    def _read_frame(self, deadline: float) -> tuple[int, int, bytes] | None:
        while time.monotonic() < deadline:
            frame = self._try_pop_frame()
            if frame is not None:
                return frame
            timeout = max(0.0, min(self.timeout, deadline - time.monotonic()))
            ready, _, _ = select.select([self.fd], [], [], timeout)
            if not ready:
                continue
            try:
                chunk = os.read(self.fd, 4096)
            except BlockingIOError:
                continue
            if chunk:
                self._read_buffer.extend(chunk)
        return None

    def _try_pop_frame(self) -> tuple[int, int, bytes] | None:
        while True:
            start = self._read_buffer.find(ESP_FRAME_MAGIC)
            if start < 0:
                del self._read_buffer[:-1]
                return None
            if start:
                del self._read_buffer[:start]
            if len(self._read_buffer) < ESP_FRAME_HEADER.size:
                return None

            magic, version, msg_type, seq, payload_len = ESP_FRAME_HEADER.unpack(
                self._read_buffer[:ESP_FRAME_HEADER.size]
            )
            if magic != ESP_FRAME_MAGIC or version != ESP_FRAME_VERSION or payload_len > ESP_MAX_PAYLOAD:
                del self._read_buffer[:2]
                continue

            frame_len = ESP_FRAME_HEADER.size + payload_len + 2
            if len(self._read_buffer) < frame_len:
                return None

            frame = bytes(self._read_buffer[:frame_len])
            del self._read_buffer[:frame_len]
            expected = int.from_bytes(frame[-2:], "little")
            actual = _crc16_ccitt(frame[:-2])
            if actual != expected:
                continue
            payload = frame[ESP_FRAME_HEADER.size:-2]
            return msg_type, seq, payload

    def _write_all(self, data: bytes) -> None:
        offset = 0
        while offset < len(data):
            try:
                offset += os.write(self.fd, data[offset:])
            except BlockingIOError:
                _, ready, _ = select.select([], [self.fd], [], self.timeout)
                if not ready:
                    raise TimeoutError("serial write timed out")


def make_drone_link(config: object) -> DroneLink:
    link_type = str(getattr(config, "link_type", "udp")).lower()
    if link_type in {"udp", "direct_udp", "pc_udp"}:
        return UdpDroneLink(
            UdpTarget(
                ip=str(getattr(config, "ip")),
                port=int(getattr(config, "port")),
                iface=getattr(config, "iface", None),
            ),
            bind_device=bool(getattr(config, "bind_device", True)),
        )
    if link_type in {"esp_serial", "esp32_serial", "serial"}:
        return EspSerialDroneLink(
            EspSerialTarget(
                port=str(getattr(config, "serial_port")),
                baud=int(getattr(config, "serial_baud", 921600)),
                drone_ssid=str(getattr(config, "ssid") or getattr(config, "esp_ssid", "")),
                drone_password=str(getattr(config, "password", "") or ""),
                drone_ip=str(getattr(config, "ip")),
                drone_port=int(getattr(config, "port")),
                connect_timeout=float(getattr(config, "esp_connect_timeout", 12.0)),
            )
        )
    raise ValueError(f"unknown drone link type: {link_type}")


def _crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc
