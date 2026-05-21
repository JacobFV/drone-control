from __future__ import annotations

import json
import os
import platform
import re
import subprocess
import tempfile
from dataclasses import asdict, dataclass
from pathlib import Path


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
    platform: str = ""
    kind: str = "wifi"


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
    system = platform.system().lower()
    if system == "darwin":
        return _macos_wifi_interfaces()
    if system == "windows":
        return _windows_wifi_interfaces()
    return _linux_wifi_interfaces()


def scan_access_points(iface: str | None = None, *, rescan: bool = True) -> list[AccessPoint]:
    system = platform.system().lower()
    if system == "darwin":
        return _macos_scan_access_points(iface)
    if system == "windows":
        return _windows_scan_access_points(iface)
    return _linux_scan_access_points(iface, rescan=rescan)


def current_wifi_connection(iface: str) -> str:
    system = platform.system().lower()
    if system == "darwin":
        try:
            output = run_command(["networksetup", "-getairportnetwork", iface]).strip()
        except (OSError, subprocess.CalledProcessError):
            return ""
        marker = "Current Wi-Fi Network:"
        return output.split(marker, 1)[1].strip() if marker in output else ""
    if system == "windows":
        try:
            output = run_command(["netsh", "wlan", "show", "interfaces"])
        except (OSError, subprocess.CalledProcessError):
            return ""
        current_name = ""
        ssid = ""
        for line in output.splitlines():
            key, _, value = line.partition(":")
            key = key.strip().lower()
            value = value.strip()
            if key == "name":
                current_name = value
            elif key == "ssid" and current_name == iface and not key.startswith("bssid"):
                ssid = value
        return ssid
    try:
        output = run_command(["nmcli", "-t", "-f", "DEVICE,CONNECTION", "dev", "status"])
    except (OSError, subprocess.CalledProcessError):
        return ""
    for line in output.splitlines():
        device, _, connection = line.partition(":")
        if device == iface:
            return connection
    return ""


def connect_wifi(iface: str, ssid: str, password: str | None = None) -> dict[str, object]:
    system = platform.system().lower()
    if system == "darwin":
        args = ["networksetup", "-setairportnetwork", iface, ssid]
        if password:
            args.append(password)
        return run_network_command(args)
    if system == "windows":
        return _windows_connect_wifi(iface, ssid, password)
    args = ["nmcli", "dev", "wifi", "connect", ssid, "ifname", iface]
    if password:
        args += ["password", password]
    return run_network_command(args)


def reconnect_wifi(iface: str, ssid: str, password: str | None = None) -> dict[str, object]:
    result = connect_wifi(iface, ssid, password)
    if result["ok"] or platform.system().lower() != "linux":
        return result
    return run_network_command(["nmcli", "con", "up", ssid, "ifname", iface])


def run_network_command(args: list[str], *, timeout: float = 35.0) -> dict[str, object]:
    try:
        completed = subprocess.run(
            args,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=timeout,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {"ok": False, "returnCode": -1, "output": str(exc), "command": _redacted_command(args)}
    return {
        "ok": completed.returncode == 0,
        "returnCode": completed.returncode,
        "output": completed.stdout.strip(),
        "command": _redacted_command(args),
    }


def platform_network_summary() -> dict[str, object]:
    interfaces = wifi_interfaces()
    default_iface = interfaces[0].name if interfaces else default_wifi_interface()
    return {
        "platform": platform.system() or "Unknown",
        "defaultInterface": default_iface,
        "interfaces": [asdict(interface) for interface in interfaces],
        "singleWifiLikely": len([item for item in interfaces if item.kind == "wifi"]) <= 1,
        "notes": _platform_notes(len(interfaces)),
    }


def default_wifi_interface() -> str:
    system = platform.system().lower()
    if system == "darwin":
        return "en0"
    if system == "windows":
        interfaces = _windows_wifi_interfaces()
        return interfaces[0].name if interfaces else "Wi-Fi"
    interfaces = _linux_wifi_interfaces()
    return interfaces[0].name if interfaces else "wlan0"


def _linux_wifi_interfaces() -> list[WifiInterface]:
    output = run_command(["nmcli", "-t", "-f", "DEVICE,TYPE,STATE,CONNECTION", "dev", "status"])
    interfaces: list[WifiInterface] = []
    for line in output.splitlines():
        parts = _split_nmcli(line)
        if len(parts) >= 4 and parts[1] == "wifi":
            interfaces.append(WifiInterface(parts[0], parts[2], parts[3], platform="Linux"))
    return interfaces


def _linux_scan_access_points(iface: str | None = None, *, rescan: bool = True) -> list[AccessPoint]:
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


def _macos_wifi_interfaces() -> list[WifiInterface]:
    try:
        output = run_command(["networksetup", "-listallhardwareports"])
    except (OSError, subprocess.CalledProcessError):
        return []
    interfaces: list[WifiInterface] = []
    blocks = output.split("\n\n")
    for block in blocks:
        if "Hardware Port: Wi-Fi" not in block and "Hardware Port: AirPort" not in block:
            continue
        name = ""
        for line in block.splitlines():
            if line.startswith("Device:"):
                name = line.split(":", 1)[1].strip()
                break
        if not name:
            continue
        connection = current_wifi_connection(name)
        state = "connected" if connection else "disconnected"
        interfaces.append(WifiInterface(name, state, connection, platform="Darwin"))
    return interfaces


def _macos_scan_access_points(iface: str | None = None) -> list[AccessPoint]:
    airport = _airport_path()
    if airport is None:
        return []
    try:
        output = run_command([str(airport), "-s"])
    except (OSError, subprocess.CalledProcessError):
        return []
    aps: list[AccessPoint] = []
    for line in output.splitlines()[1:]:
        match = re.match(r"^\s*(?P<ssid>.+?)\s+(?P<bssid>[0-9a-fA-F:]{17})\s+(?P<rssi>-?\d+)\s+(?P<chan>\S+)\s+\S+\s+\S+\s*(?P<security>.*)$", line)
        if not match:
            continue
        ssid = match.group("ssid").strip()
        rssi = int(match.group("rssi"))
        signal = max(0, min(100, 2 * (rssi + 100)))
        channel = match.group("chan")
        aps.append(
            AccessPoint(
                ssid=ssid,
                bssid=match.group("bssid"),
                channel=channel,
                frequency="",
                signal=signal,
                security=match.group("security").strip() or "None",
                likely_drone=is_likely_drone_ssid(ssid),
            )
        )
    aps.sort(key=lambda ap: (not ap.likely_drone, -ap.signal, ap.ssid))
    return aps


def _windows_wifi_interfaces() -> list[WifiInterface]:
    try:
        output = run_command(["netsh", "wlan", "show", "interfaces"])
    except (OSError, subprocess.CalledProcessError):
        return []
    interfaces: list[WifiInterface] = []
    current: dict[str, str] = {}
    for line in output.splitlines():
        if not line.strip():
            if current:
                interfaces.append(_windows_interface_from_block(current))
                current = {}
            continue
        key, _, value = line.partition(":")
        if value:
            current[key.strip().lower()] = value.strip()
    if current:
        interfaces.append(_windows_interface_from_block(current))
    return [item for item in interfaces if item.name]


def _windows_interface_from_block(block: dict[str, str]) -> WifiInterface:
    name = block.get("name", "")
    state = block.get("state", "")
    connection = block.get("ssid", "") if state.lower() == "connected" else ""
    return WifiInterface(name, state, connection, platform="Windows")


def _windows_scan_access_points(iface: str | None = None) -> list[AccessPoint]:
    args = ["netsh", "wlan", "show", "networks", "mode=bssid"]
    if iface:
        args.append(f"interface={iface}")
    try:
        output = run_command(args)
    except (OSError, subprocess.CalledProcessError):
        return []
    aps: list[AccessPoint] = []
    current: dict[str, str] = {}
    bssid = ""
    for line in output.splitlines():
        stripped = line.strip()
        if stripped.startswith("SSID ") and ":" in stripped:
            if current.get("ssid"):
                aps.append(_windows_ap_from_block(current, bssid))
            current = {"ssid": stripped.split(":", 1)[1].strip()}
            bssid = ""
        elif stripped.lower().startswith("authentication"):
            current["security"] = stripped.split(":", 1)[1].strip()
        elif stripped.lower().startswith("bssid"):
            bssid = stripped.split(":", 1)[1].strip()
        elif stripped.lower().startswith("signal"):
            current["signal"] = stripped.split(":", 1)[1].strip().rstrip("%")
        elif stripped.lower().startswith("channel"):
            current["channel"] = stripped.split(":", 1)[1].strip()
    if current.get("ssid"):
        aps.append(_windows_ap_from_block(current, bssid))
    aps.sort(key=lambda ap: (not ap.likely_drone, -ap.signal, ap.ssid))
    return aps


def _windows_ap_from_block(block: dict[str, str], bssid: str) -> AccessPoint:
    ssid = block.get("ssid", "")
    return AccessPoint(
        ssid=ssid,
        bssid=bssid,
        channel=block.get("channel", ""),
        frequency="",
        signal=int(block.get("signal") or 0),
        security=block.get("security", ""),
        likely_drone=is_likely_drone_ssid(ssid),
    )


def _windows_connect_wifi(iface: str, ssid: str, password: str | None) -> dict[str, object]:
    profile = _windows_profile(ssid, password)
    with tempfile.NamedTemporaryFile("w", suffix=".xml", delete=False) as handle:
        handle.write(profile)
        profile_path = handle.name
    try:
        add = run_network_command(["netsh", "wlan", "add", "profile", f"filename={profile_path}", f"interface={iface}"])
        if not add["ok"]:
            return add
        return run_network_command(["netsh", "wlan", "connect", f"name={ssid}", f"ssid={ssid}", f"interface={iface}"])
    finally:
        try:
            os.unlink(profile_path)
        except OSError:
            pass


def _windows_profile(ssid: str, password: str | None) -> str:
    escaped = (
        ssid.replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
        .replace('"', "&quot;")
        .replace("'", "&apos;")
    )
    if password:
        key = (
            password.replace("&", "&amp;")
            .replace("<", "&lt;")
            .replace(">", "&gt;")
            .replace('"', "&quot;")
            .replace("'", "&apos;")
        )
        return f"""<?xml version="1.0"?>
<WLANProfile xmlns="http://www.microsoft.com/networking/WLAN/profile/v1">
  <name>{escaped}</name>
  <SSIDConfig><SSID><name>{escaped}</name></SSID></SSIDConfig>
  <connectionType>ESS</connectionType>
  <connectionMode>manual</connectionMode>
  <MSM><security><authEncryption><authentication>WPA2PSK</authentication><encryption>AES</encryption><useOneX>false</useOneX></authEncryption><sharedKey><keyType>passPhrase</keyType><protected>false</protected><keyMaterial>{key}</keyMaterial></sharedKey></security></MSM>
</WLANProfile>
"""
    return f"""<?xml version="1.0"?>
<WLANProfile xmlns="http://www.microsoft.com/networking/WLAN/profile/v1">
  <name>{escaped}</name>
  <SSIDConfig><SSID><name>{escaped}</name></SSID></SSIDConfig>
  <connectionType>ESS</connectionType>
  <connectionMode>manual</connectionMode>
  <MSM><security><authEncryption><authentication>open</authentication><encryption>none</encryption><useOneX>false</useOneX></authEncryption></security></MSM>
</WLANProfile>
"""


def _airport_path() -> Path | None:
    candidates = [
        Path("/System/Library/PrivateFrameworks/Apple80211.framework/Versions/Current/Resources/airport"),
        Path("/System/Library/PrivateFrameworks/Apple80211.framework/Resources/airport"),
    ]
    return next((path for path in candidates if path.exists()), None)


def is_likely_drone_ssid(ssid: str) -> bool:
    upper = ssid.upper()
    return any(pattern in upper for pattern in DRONE_SSID_PATTERNS)


def aps_to_json(aps: list[AccessPoint]) -> str:
    return json.dumps([asdict(ap) for ap in aps], indent=2)


def _split_nmcli(line: str) -> list[str]:
    placeholder = "\0"
    text = re.sub(r"\\:", placeholder, line)
    return [part.replace(placeholder, ":") for part in text.split(":")]


def _redacted_command(args: list[str]) -> list[str]:
    redacted: list[str] = []
    hide_next = False
    for arg in args:
        if hide_next:
            redacted.append("<redacted>")
            hide_next = False
            continue
        redacted.append(arg)
        if arg.lower() in {"password", "key"}:
            hide_next = True
    return redacted


def _platform_notes(interface_count: int) -> str:
    if interface_count <= 1:
        return "One usable PC Wi-Fi interface was detected. Direct UDP can use one drone AP at a time; add ESP32 bridges or another Wi-Fi adapter for simultaneous drones."
    return "Multiple PC Wi-Fi interfaces were detected. Assign one direct UDP interface or one ESP32 bridge per drone AP for simultaneous live control."
