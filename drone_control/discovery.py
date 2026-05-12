from __future__ import annotations

import json
import re
import subprocess
from dataclasses import dataclass


DRONE_SSID_PATTERNS = (
    "E99",
    "WIFI_",
    "WIFI-",
    "WIFI8K",
    "WIFI_8K",
    "WIFI_UFO",
    "DRONE",
    "UAV",
    "FLOW",
    "GD89",
    "WTECH",
)


@dataclass(slots=True)
class WifiInterface:
    name: str
    state: str
    connection: str


@dataclass(slots=True)
class AccessPoint:
    ssid: str
    bssid: str
    channel: str
    frequency: str
    signal: int
    security: str
    likely_drone: bool


def run_command(args: list[str]) -> str:
    return subprocess.check_output(args, text=True, stderr=subprocess.STDOUT)


def wifi_interfaces() -> list[WifiInterface]:
    output = run_command(["nmcli", "-t", "-f", "DEVICE,TYPE,STATE,CONNECTION", "dev", "status"])
    interfaces: list[WifiInterface] = []
    for line in output.splitlines():
        parts = _split_nmcli(line)
        if len(parts) >= 4 and parts[1] == "wifi":
            interfaces.append(WifiInterface(parts[0], parts[2], parts[3]))
    return interfaces


def scan_access_points(iface: str | None = None, *, rescan: bool = True) -> list[AccessPoint]:
    args = ["nmcli", "-t", "-f", "SSID,BSSID,CHAN,FREQ,SIGNAL,SECURITY", "dev", "wifi", "list"]
    if iface:
        args += ["ifname", iface]
    args += ["--rescan", "yes" if rescan else "no"]
    output = run_command(args)
    aps: list[AccessPoint] = []
    for line in output.splitlines():
        parts = _split_nmcli(line)
        if len(parts) < 6:
            continue
        ssid = parts[0]
        signal = int(parts[4] or 0)
        aps.append(
            AccessPoint(
                ssid=ssid,
                bssid=parts[1],
                channel=parts[2],
                frequency=parts[3],
                signal=signal,
                security=parts[5],
                likely_drone=is_likely_drone_ssid(ssid),
            )
        )
    aps.sort(key=lambda ap: (not ap.likely_drone, -ap.signal, ap.ssid))
    return aps


def is_likely_drone_ssid(ssid: str) -> bool:
    upper = ssid.upper()
    return any(pattern in upper for pattern in DRONE_SSID_PATTERNS)


def aps_to_json(aps: list[AccessPoint]) -> str:
    return json.dumps([ap.__dict__ for ap in aps], indent=2)


def _split_nmcli(line: str) -> list[str]:
    placeholder = "\0"
    text = re.sub(r"\\:", placeholder, line)
    return [part.replace(placeholder, ":") for part in text.split(":")]

