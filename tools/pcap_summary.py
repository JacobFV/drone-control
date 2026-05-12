#!/usr/bin/env python3
from __future__ import annotations

import argparse
import collections
import socket
import struct
from pathlib import Path


SNAP_IPV4 = b"\xaa\xaa\x03\x00\x00\x00\x08\x00"


def main() -> int:
    parser = argparse.ArgumentParser(description="Summarize UDP flows and WIFI_8K control packets from a monitor-mode pcap.")
    parser.add_argument("pcap", type=Path)
    parser.add_argument("--limit", type=int, default=20, help="Rows to print per section.")
    args = parser.parse_args()

    flows: collections.Counter[tuple[str, int, str, int]] = collections.Counter()
    flow_bytes: collections.Counter[tuple[str, int, str, int]] = collections.Counter()
    flow_lengths: dict[tuple[str, int, str, int], collections.Counter[int]] = collections.defaultdict(collections.Counter)
    flow_first: dict[tuple[str, int, str, int], float] = {}
    flow_last: dict[tuple[str, int, str, int], float] = {}
    payloads: collections.Counter[tuple[str, int, str, int, bytes]] = collections.Counter()
    wifi8k: collections.Counter[tuple[str, int, str, int, bytes]] = collections.Counter()

    packets = 0
    udp_packets = 0
    first_ts: float | None = None
    for ts, frame in read_pcap_records(args.pcap):
        if first_ts is None:
            first_ts = ts
        rel_ts = ts - first_ts
        packets += 1
        parsed = parse_udp_from_radiotap(frame)
        if parsed is None:
            continue
        udp_packets += 1
        src, sport, dst, dport, payload = parsed
        flow = (src, sport, dst, dport)
        flows[flow] += 1
        flow_bytes[flow] += len(payload)
        flow_lengths[flow][len(payload)] += 1
        flow_first.setdefault(flow, rel_ts)
        flow_last[flow] = rel_ts
        if len(payload) <= 64:
            payloads[(src, sport, dst, dport, payload)] += 1
        if is_wifi8k_control(payload):
            wifi8k[(src, sport, dst, dport, payload)] += 1

    print(f"pcap={args.pcap}")
    print(f"frames={packets} udp_ipv4={udp_packets}")
    print()
    print("UDP flows:")
    for flow, count in flows.most_common(args.limit):
        src, sport, dst, dport = flow
        lens = ", ".join(f"{length}:{n}" for length, n in flow_lengths[flow].most_common(5))
        print(
            f"{count:6d} packets {flow_bytes[flow]:8d} bytes  "
            f"{src}:{sport} -> {dst}:{dport}  "
            f"first={flow_first[flow]:7.3f}s last={flow_last[flow]:7.3f}s  lens={lens}"
        )

    print()
    print("Likely camera streams:")
    camera_rows = find_camera_streams(flows, flow_bytes)
    if not camera_rows:
        print("none")
    for video_flow, aux_flow in camera_rows[:args.limit]:
        src, sport, dst, dport = video_flow
        aux_text = "aux=not found"
        if aux_flow:
            aux_src, aux_sport, aux_dst, aux_dport = aux_flow
            aux_text = f"aux={aux_src}:{aux_sport} -> {aux_dst}:{aux_dport}"
        print(
            f"{flows[video_flow]:6d} packets {flow_bytes[video_flow]:8d} bytes  "
            f"video={src}:{sport} -> {dst}:{dport}  "
            f"first={flow_first[video_flow]:7.3f}s last={flow_last[video_flow]:7.3f}s  {aux_text}"
        )

    print()
    print("Likely WIFI_8K controls:")
    if not wifi8k:
        print("none")
    for item, count in wifi8k.most_common(args.limit):
        src, sport, dst, dport, payload = item
        roll, pitch, throttle, yaw, flags, checksum = payload[2:8]
        expected = xor(payload[2:7])
        status = "ok" if checksum == expected else f"bad expected={expected:02x}"
        print(
            f"{count:6d} {src}:{sport} -> {dst}:{dport}  "
            f"R={roll:3d} P={pitch:3d} T={throttle:3d} Y={yaw:3d} "
            f"flags=0x{flags:02x} checksum={status}  {payload.hex(' ')}"
        )

    print()
    print("Short UDP payloads:")
    for item, count in payloads.most_common(args.limit):
        src, sport, dst, dport, payload = item
        print(f"{count:6d} {src}:{sport} -> {dst}:{dport} len={len(payload):2d} {payload.hex(' ')}")
    return 0


def read_pcap(path: Path):
    for _ts, frame in read_pcap_records(path):
        yield frame


def read_pcap_records(path: Path):
    with path.open("rb") as handle:
        header = handle.read(24)
        if len(header) != 24:
            raise ValueError("pcap header is truncated")
        magic = header[:4]
        if magic in {b"\xd4\xc3\xb2\xa1", b"\x4d\x3c\xb2\xa1"}:
            endian = "<"
        elif magic in {b"\xa1\xb2\xc3\xd4", b"\xa1\xb2\x3c\x4d"}:
            endian = ">"
        else:
            raise ValueError(f"unsupported pcap magic {magic.hex()}")

        record_struct = struct.Struct(endian + "IIII")
        while True:
            record_header = handle.read(record_struct.size)
            if not record_header:
                return
            if len(record_header) != record_struct.size:
                raise ValueError("pcap record header is truncated")
            _ts_sec, _ts_usec, incl_len, _orig_len = record_struct.unpack(record_header)
            frame = handle.read(incl_len)
            if len(frame) != incl_len:
                raise ValueError("pcap record body is truncated")
            yield _ts_sec + (_ts_usec / 1_000_000), frame


def parse_udp_from_radiotap(frame: bytes) -> tuple[str, int, str, int, bytes] | None:
    if len(frame) < 8:
        return None
    radiotap_len = int.from_bytes(frame[2:4], "little")
    start = max(0, min(radiotap_len, len(frame)))
    snap_at = frame.find(SNAP_IPV4, start)
    if snap_at < 0:
        return None
    packet = frame[snap_at + len(SNAP_IPV4):]
    if len(packet) < 28:
        return None
    version = packet[0] >> 4
    ihl = (packet[0] & 0x0F) * 4
    if version != 4 or ihl < 20 or len(packet) < ihl + 8:
        return None
    protocol = packet[9]
    if protocol != 17:
        return None
    src = socket.inet_ntoa(packet[12:16])
    dst = socket.inet_ntoa(packet[16:20])
    sport, dport, udp_len, _checksum = struct.unpack("!HHHH", packet[ihl:ihl + 8])
    if udp_len < 8 or len(packet) < ihl + udp_len:
        return None
    payload = packet[ihl + 8:ihl + udp_len]
    return src, sport, dst, dport, payload


def is_wifi8k_control(payload: bytes) -> bool:
    return len(payload) == 9 and payload[0] == 0x03 and payload[1] == 0x66 and payload[-1] == 0x99


def find_camera_streams(
    flows: collections.Counter[tuple[str, int, str, int]],
    flow_bytes: collections.Counter[tuple[str, int, str, int]],
) -> list[tuple[tuple[str, int, str, int], tuple[str, int, str, int] | None]]:
    rows: list[tuple[tuple[str, int, str, int], tuple[str, int, str, int] | None]] = []
    ignored_ports = {53, 5353, 7099, 8800, 3478}
    for flow, count in flows.items():
        src, sport, dst, dport = flow
        if flow_bytes[flow] < 500_000 or count < 100:
            continue
        if sport in ignored_ports or dport in ignored_ports:
            continue
        aux_flow = (dst, dport + 1, src, sport + 1)
        rows.append((flow, aux_flow if aux_flow in flows else None))
    rows.sort(key=lambda item: flow_bytes[item[0]], reverse=True)
    return rows


def xor(values: bytes) -> int:
    result = 0
    for value in values:
        result ^= value
    return result & 0xFF


if __name__ == "__main__":
    raise SystemExit(main())
