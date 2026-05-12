#!/usr/bin/env python3
from __future__ import annotations

import argparse
import collections
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from pcap_summary import parse_udp_from_radiotap, read_pcap_records


Flow = tuple[str, int, str, int]


def main() -> int:
    parser = argparse.ArgumentParser(description="Extract WIFI_8K/Taixin camera frame chunks from a monitor pcap.")
    parser.add_argument("pcap", type=Path)
    parser.add_argument("--flow", help="Video flow as src_ip:src_port,dst_ip:dst_port. Defaults to largest camera-like UDP flow.")
    parser.add_argument("--out-dir", type=Path, default=Path("camera_captures/pcap_frames"))
    parser.add_argument("--keep", type=int, default=6, help="Open frame ids to keep before flushing older frames.")
    args = parser.parse_args()

    flow = parse_flow(args.flow) if args.flow else find_video_flow(args.pcap)
    if flow is None:
        print("no camera-like UDP flow found", file=sys.stderr)
        return 1

    args.out_dir.mkdir(parents=True, exist_ok=True)
    magic = detect_magic(args.pcap, flow)
    if magic is None:
        print(f"no Taixin-like video magic detected for {format_flow(flow)}", file=sys.stderr)
        return 1

    frames: collections.OrderedDict[bytes, dict[int, bytes]] = collections.OrderedDict()
    packets = 0
    frame_count = 0
    raw_path = args.out_dir / "stream_payloads.bin"
    with raw_path.open("wb") as raw:
        for _ts, payload in iter_flow_payloads(args.pcap, flow):
            if len(payload) < 24 or payload[8:12] != magic:
                continue
            packets += 1
            raw.write(payload)
            frame_key = payload[4:8]
            offset = int.from_bytes(payload[14:16], "big")
            header_len = 156 if offset == 0 and len(payload) >= 156 else 24
            frames.setdefault(frame_key, {}).setdefault(offset, payload[header_len:])
            while len(frames) > args.keep:
                old_key, chunks = frames.popitem(last=False)
                frame_count += write_frame(args.out_dir, frame_count, old_key, chunks)

    for frame_key, chunks in frames.items():
        frame_count += write_frame(args.out_dir, frame_count, frame_key, chunks)

    print(f"flow={format_flow(flow)}")
    print(f"magic={magic.hex(' ')} packets={packets} frames={frame_count}")
    print(f"output={args.out_dir}")
    return 0 if packets else 1


def find_video_flow(pcap: Path) -> Flow | None:
    flows: collections.Counter[Flow] = collections.Counter()
    byte_counts: collections.Counter[Flow] = collections.Counter()
    for _ts, frame in read_pcap_records(pcap):
        parsed = parse_udp_from_radiotap(frame)
        if not parsed:
            continue
        src, sport, dst, dport, payload = parsed
        flow = (src, sport, dst, dport)
        flows[flow] += 1
        byte_counts[flow] += len(payload)

    ignored_ports = {53, 5353, 7099, 8800, 3478}
    candidates = [
        flow for flow, count in flows.items()
        if count >= 100
        and byte_counts[flow] >= 500_000
        and flow[1] not in ignored_ports
        and flow[3] not in ignored_ports
    ]
    if not candidates:
        return None
    return max(candidates, key=lambda item: byte_counts[item])


def detect_magic(pcap: Path, flow: Flow) -> bytes | None:
    magics: collections.Counter[bytes] = collections.Counter()
    for _ts, payload in iter_flow_payloads(pcap, flow):
        if len(payload) >= 24:
            magics[payload[8:12]] += 1
    if not magics:
        return None
    magic, count = magics.most_common(1)[0]
    return magic if count >= 10 else None


def iter_flow_payloads(pcap: Path, flow: Flow):
    for ts, frame in read_pcap_records(pcap):
        parsed = parse_udp_from_radiotap(frame)
        if not parsed:
            continue
        src, sport, dst, dport, payload = parsed
        if (src, sport, dst, dport) == flow:
            yield ts, payload


def write_frame(out_dir: Path, index: int, frame_key: bytes, chunks: dict[int, bytes]) -> int:
    if not chunks:
        return 0
    size = max(offset + len(data) for offset, data in chunks.items())
    frame = bytearray(size)
    for offset, data in chunks.items():
        frame[offset:offset + len(data)] = data
    suffix = "jpgish" if frame.find(b"\xff\xd9") >= 0 else "bin"
    path = out_dir / f"frame_{index:05d}_{frame_key.hex()}.{suffix}"
    path.write_bytes(frame)
    return 1


def parse_flow(value: str) -> Flow:
    left, right = value.split(",", 1)
    src, sport = left.rsplit(":", 1)
    dst, dport = right.rsplit(":", 1)
    return src, int(sport), dst, int(dport)


def format_flow(flow: Flow) -> str:
    src, sport, dst, dport = flow
    return f"{src}:{sport}->{dst}:{dport}"


if __name__ == "__main__":
    raise SystemExit(main())
