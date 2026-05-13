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

    def configure(
        self,
        *,
        enabled: bool | None = None,
        iface: str | None = None,
        ip: str | None = None,
        port: int | None = None,
        protocol: str | None = None,
        bind_device: bool | None = None,
    ) -> None:
        target_changed = False
        if enabled is not None:
            self.enabled = enabled
        next_iface = self.target.iface if iface is None else iface or None
        next_ip = self.target.ip if ip is None else ip
        next_port = self.target.port if port is None else port
        if next_iface != self.target.iface or next_ip != self.target.ip or next_port != self.target.port:
            self.target = UdpTarget(ip=next_ip, port=next_port, iface=next_iface)
            target_changed = True
        if protocol is not None and protocol != self.protocol.name:
            self.protocol = make_protocol(protocol)
            target_changed = True
        if bind_device is not None and bind_device != self.bind_device:
            self.bind_device = bind_device
            target_changed = True
        if target_changed:
            self.close()

    def config_dict(self) -> dict[str, object]:
        return {
            "enabled": self.enabled,
            "iface": self.target.iface or "",
            "ip": self.target.ip,
            "port": self.target.port,
            "protocol": self.protocol.name,
            "bindDevice": self.bind_device,
        }

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
