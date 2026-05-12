from __future__ import annotations

import os
import socket
from dataclasses import dataclass


SO_BINDTODEVICE = getattr(socket, "SO_BINDTODEVICE", 25)


@dataclass(slots=True)
class UdpTarget:
    ip: str
    port: int
    iface: str | None = None


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
