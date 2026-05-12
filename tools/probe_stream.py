#!/usr/bin/env python3
from __future__ import annotations

import argparse
import socket
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drone_control.transport import SO_BINDTODEVICE


PROBES = {
    "wifi_cam_start": (b"\x42\x76", [8080]),
    "wifi_cam_stop": (b"\x42\x77", [8080]),
    "wifi_uav_start": (b"\xef\x00\x04\x00", [8800, 8801, 1234]),
    "wifi_uav_request_a": (
        bytes.fromhex(
            "ef02580002020001000000000500000014006614808080800002000000000000000000000299"
            "00000000000000000000000000000000000000000000000000000000000000000000000000000000324b142d0000"
        ),
        [8800],
    ),
}


def main() -> int:
    parser = argparse.ArgumentParser(description="Send safe stream-start probes and listen for UDP replies.")
    parser.add_argument("--iface", required=True)
    parser.add_argument("--ip", action="append", dest="ips", default=[])
    parser.add_argument("--probe", action="append", dest="probes", choices=sorted(PROBES), help="Probe name. Can repeat.")
    parser.add_argument("--seconds", type=float, default=1.5)
    parser.add_argument("--no-bind-device", action="store_true")
    args = parser.parse_args()

    ips = args.ips or ["192.168.1.1", "192.168.169.1", "192.168.4.153", "192.168.100.1"]
    probes = args.probes or ["wifi_cam_start", "wifi_uav_start"]

    for ip in _dedupe(ips):
        for probe_name in probes:
            payload, ports = PROBES[probe_name]
            for port in ports:
                run_probe(args.iface, ip, port, probe_name, payload, args.seconds, bind_device=not args.no_bind_device)
    return 0


def run_probe(iface: str, ip: str, port: int, name: str, payload: bytes, seconds: float, *, bind_device: bool) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(0.2)
    sock.bind(("", 0))
    if bind_device:
        sock.setsockopt(socket.SOL_SOCKET, SO_BINDTODEVICE, iface.encode() + b"\0")
    local = sock.getsockname()
    replies: list[tuple[tuple[str, int], bytes]] = []
    try:
        sock.sendto(payload, (ip, port))
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            try:
                data, source = sock.recvfrom(4096)
            except socket.timeout:
                continue
            replies.append((source, data))
            if len(replies) >= 5:
                break
    except OSError as exc:
        print(f"{name:18} {ip:15}:{port:<5} local={local[1]:<5} error={exc}")
        return
    finally:
        sock.close()

    if replies:
        summary = "; ".join(
            f"{src[0]}:{src[1]} len={len(data)} head={data[:32].hex(' ')}"
            for src, data in replies
        )
        print(f"{name:18} {ip:15}:{port:<5} local={local[1]:<5} replies {summary}")
    else:
        print(f"{name:18} {ip:15}:{port:<5} local={local[1]:<5} no replies")


def _dedupe(values: list[str]) -> list[str]:
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
