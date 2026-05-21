from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass(slots=True)
class DroneConfig:
    id: str
    iface: str = ""
    ssid: str | None = None
    ip: str = "192.168.169.1"
    port: int = 7099
    protocol: str = "wifi_8k_prefixed_short"
    bind_device: bool = True
    link_type: str = "udp"
    serial_port: str = ""
    serial_baud: int = 921600
    password: str = ""
    esp_connect_timeout: float = 12.0


def load_config(path: str | Path) -> list[DroneConfig]:
    data = json.loads(Path(path).read_text())
    drones = data.get("drones", data if isinstance(data, list) else [])
    result: list[DroneConfig] = []
    for index, item in enumerate(drones, start=1):
        result.append(
            DroneConfig(
                id=str(item.get("id", f"drone{index}")),
                iface=str(item.get("iface", "")),
                ssid=item.get("ssid"),
                ip=str(item.get("ip", "192.168.169.1")),
                port=int(item.get("port", 7099)),
                protocol=str(item.get("protocol", "wifi_8k_prefixed_short")),
                bind_device=bool(item.get("bind_device", True)),
                link_type=str(item.get("link_type", item.get("linkType", "udp"))),
                serial_port=str(item.get("serial_port", item.get("serialPort", ""))),
                serial_baud=int(item.get("serial_baud", item.get("serialBaud", 921600))),
                password=str(item.get("password", "")),
                esp_connect_timeout=float(item.get("esp_connect_timeout", item.get("espConnectTimeout", 12.0))),
            )
        )
    return result


def config_to_dict(configs: list[DroneConfig]) -> dict[str, Any]:
    return {
        "drones": [
            {
                "id": cfg.id,
                "iface": cfg.iface,
                "ssid": cfg.ssid,
                "ip": cfg.ip,
                "port": cfg.port,
                "protocol": cfg.protocol,
                "bind_device": cfg.bind_device,
                "link_type": cfg.link_type,
                "serial_port": cfg.serial_port,
                "serial_baud": cfg.serial_baud,
                "password": cfg.password,
                "esp_connect_timeout": cfg.esp_connect_timeout,
            }
            for cfg in configs
        ]
    }
