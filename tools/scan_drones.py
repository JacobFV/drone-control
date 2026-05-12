#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drone_control.discovery import scan_access_points, wifi_interfaces


def main() -> int:
    parser = argparse.ArgumentParser(description="Scan for likely AP-mode drone Wi-Fi networks.")
    parser.add_argument("--iface", help="Wi-Fi interface to scan, for example wlan1 or wlP9s9.")
    parser.add_argument("--json", action="store_true", help="Emit JSON.")
    args = parser.parse_args()

    interfaces = wifi_interfaces()
    aps = scan_access_points(args.iface)
    if args.json:
        print(json.dumps({"interfaces": [item.__dict__ for item in interfaces], "aps": [ap.__dict__ for ap in aps]}, indent=2))
        return 0

    print("Wi-Fi interfaces:")
    for iface in interfaces:
        print(f"  {iface.name:12} {iface.state:16} {iface.connection}")
    print()
    print("Access points:")
    for ap in aps:
        marker = "*" if ap.likely_drone else " "
        security = ap.security or "open"
        print(f"{marker} {ap.ssid:24} signal={ap.signal:3d} chan={ap.channel:>3} bssid={ap.bssid} security={security}")
    print()
    print("* = likely drone AP")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
