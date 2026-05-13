from __future__ import annotations

import os
from dataclasses import dataclass

from drone_control.actions import DroneAction
from drone_control.protocols import PacketProtocol, make_protocol
from drone_control.transport import UdpDroneLink, UdpTarget


def env_bool(name: str, default: bool = False) -> bool:
    raw = os.environ.get(name)
    if raw is None:
        return default
    return raw.strip().lower() in {"1", "true", "yes", "on"}


@dataclass(slots=True)
class ManualTransportStatus:
    enabled: bool
    connected: bool
    target: str
    sent: int
    errors: int
    last_error: str | None

    def as_dict(self) -> dict[str, object]:
        return {
            "enabled": self.enabled,
            "connected": self.connected,
            "target": self.target,
            "sent": self.sent,
            "errors": self.errors,
            "lastError": self.last_error,
        }


class ManualDroneTransport:
    def __init__(
        self,
        *,
        enabled: bool,
        iface: str,
        ip: str,
        port: int,
        protocol: str,
        bind_device: bool,
    ) -> None:
        self.enabled = enabled
        self.target = UdpTarget(ip=ip, port=port, iface=iface or None)
        self.protocol: PacketProtocol = make_protocol(protocol)
        self.bind_device = bind_device
        self.link: UdpDroneLink | None = None
        self.sent = 0
        self.errors = 0
        self.last_error: str | None = None

    @classmethod
    def from_env(cls) -> "ManualDroneTransport":
        return cls(
            enabled=env_bool("DRONE_SERVICE_ENABLE_IO", False),
            iface=os.environ.get("DRONE_IFACE", "wlP9s9"),
            ip=os.environ.get("DRONE_IP", "192.168.1.1"),
            port=int(os.environ.get("DRONE_PORT", "7099")),
            protocol=os.environ.get("DRONE_PROTOCOL", "wifi_8k_prefixed_short"),
            bind_device=env_bool("DRONE_BIND_DEVICE", False),
        )

    def send(self, action: DroneAction | None) -> bool:
        if action is None or not self.enabled:
            return False
        try:
            if self.link is None:
                self.link = UdpDroneLink(self.target, bind_device=self.bind_device)
            self.link.send(self.protocol.build(action))
            self.sent += 1
            self.last_error = None
            return True
        except OSError as exc:
            self.errors += 1
            self.last_error = str(exc)
            self.close()
            return False

    def close(self) -> None:
        if self.link is None:
            return
        self.link.close()
        self.link = None

    def status(self) -> ManualTransportStatus:
        target = f"{self.target.ip}:{self.target.port}"
        if self.target.iface:
            target = f"{self.target.iface} -> {target}"
        return ManualTransportStatus(
            enabled=self.enabled,
            connected=self.link is not None,
            target=target,
            sent=self.sent,
            errors=self.errors,
            last_error=self.last_error,
        )
