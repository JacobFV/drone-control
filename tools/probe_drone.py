#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drone_control.actions import DroneAction
from drone_control.protocols import make_protocol
from drone_control.transport import UdpDroneLink, UdpTarget


DEFAULT_IPS = ["192.168.1.1", "192.168.169.1", "192.168.4.1", "192.168.4.153", "192.168.100.1"]
DEFAULT_PORTS = [7099, 8800, 8801, 8090, 50000, 8899]
DEFAULT_PROTOCOLS = ["wifi_8k_prefixed_short", "wifi_uav_envelope", "wifi_cam_extended", "wifi_cam_short"]


def main() -> int:
    parser = argparse.ArgumentParser(description="Send neutral probe packets to candidate drone UDP endpoints.")
    parser.add_argument("--iface", required=True, help="Interface connected to the drone AP.")
    parser.add_argument("--ip", action="append", dest="ips", help="Candidate drone IP. Can be repeated.")
    parser.add_argument("--port", action="append", dest="ports", type=int, help="Candidate UDP port. Can be repeated.")
    parser.add_argument("--protocol", action="append", dest="protocols", help="Protocol to try. Can be repeated.")
    parser.add_argument("--no-bind-device", action="store_true", help="Do not use SO_BINDTODEVICE.")
    parser.add_argument("--active", action="store_true", help="Also send calibrate once. Default only sends neutral packets.")
    parser.add_argument("--listen", action="store_true", help="Listen briefly for UDP replies after sends.")
    parser.add_argument("--seconds", type=float, default=0.35, help="Seconds to send each candidate.")
    args = parser.parse_args()

    ips = _dedupe(args.ips or DEFAULT_IPS)
    ports = [int(port) for port in _dedupe(args.ports or DEFAULT_PORTS)]
    protocols = _dedupe(args.protocols or DEFAULT_PROTOCOLS)

    neutral = DroneAction.neutral()
    active = DroneAction(calibrate=True) if args.active else neutral

    print("Probing with neutral packets. No takeoff/land command will be sent.")
    for ip in ips:
        for port in ports:
            for protocol_name in protocols:
                protocol = make_protocol(protocol_name)
                target = UdpTarget(ip=ip, port=port, iface=args.iface)
                try:
                    link = UdpDroneLink(target, bind_device=not args.no_bind_device)
                except PermissionError as exc:
                    print(f"permission: {exc}")
                    return 77
                deadline = time.monotonic() + args.seconds
                sends = 0
                try:
                    keepalive = getattr(protocol, "keepalive", None)
                    next_keepalive = time.monotonic()
                    while time.monotonic() < deadline:
                        action = active if sends == 0 else neutral
                        link.send(protocol.build(action))
                        sends += 1
                        if callable(keepalive) and time.monotonic() >= next_keepalive:
                            link.send(keepalive())
                            next_keepalive = time.monotonic() + 1.0
                        time.sleep(0.05)
                    replies = []
                    if args.listen:
                        listen_deadline = time.monotonic() + 0.2
                        while time.monotonic() < listen_deadline:
                            reply = link.recv_once(4096)
                            if reply is None:
                                break
                            payload, source = reply
                            replies.append((source, payload[:32].hex(" "), len(payload)))
                    suffix = ""
                    if replies:
                        suffix = " replies=" + "; ".join(
                            f"{source[0]}:{source[1]} len={length} head={head}"
                            for source, head, length in replies[:3]
                        )
                    print(f"sent {sends:2d} packets iface={args.iface} ip={ip:15} port={port:5d} protocol={protocol_name}{suffix}")
                except OSError as exc:
                    print(f"error iface={args.iface} ip={ip} port={port} protocol={protocol_name}: {exc}")
                finally:
                    link.close()
    return 0


def _dedupe(values: list) -> list:
    seen = set()
    result = []
    for value in values:
        if value in seen:
            continue
        seen.add(value)
        result.append(value)
    return result


if __name__ == "__main__":
    raise SystemExit(main())
