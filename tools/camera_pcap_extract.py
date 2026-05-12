#!/usr/bin/env python3
from __future__ import annotations

import argparse
import collections
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drone_control.rtp_jpeg import (
    RtpJpegFrame,
    add_packet,
    assemble_jpeg,
    assemble_scan_data,
    parse_rtp_jpeg_packet,
    start_frame,
)

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

    frames: collections.OrderedDict[bytes, RtpJpegFrame] = collections.OrderedDict()
    last_quantization_tables: bytes | None = None
    packets = 0
    frame_count = 0
    raw_path = args.out_dir / "stream_payloads.bin"
    with raw_path.open("wb") as raw:
        for _ts, payload in iter_flow_payloads(args.pcap, flow):
            packet = parse_rtp_jpeg_packet(payload)
            if packet is None or packet.frame_key != payload[4:8] or payload[8:12] != magic:
                continue
            packets += 1
            raw.write(payload)
            if packet.quantization_tables:
                last_quantization_tables = packet.quantization_tables
            frame = frames.get(packet.frame_key)
            if frame is None:
                frame = start_frame(packet, last_quantization_tables)
                frames[packet.frame_key] = frame
            add_packet(frame, packet)
            while len(frames) > args.keep:
                _, frame = frames.popitem(last=False)
                frame_count += write_frame(args.out_dir, frame_count, frame)

    for frame in frames.values():
        frame_count += write_frame(args.out_dir, frame_count, frame)

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


def write_frame(out_dir: Path, index: int, frame: RtpJpegFrame) -> int:
    if not frame.chunks:
        return 0
    jpeg = assemble_jpeg(frame)
    if jpeg:
        path = out_dir / f"frame_{index:05d}_{frame.frame_key.hex()}.jpg"
        path.write_bytes(jpeg)
        return 1

    scan = assemble_scan_data(frame)
    suffix = "jpgish" if scan.find(b"\xff\xd9") >= 0 else "bin"
    path = out_dir / f"frame_{index:05d}_{frame.frame_key.hex()}.{suffix}"
    path.write_bytes(scan)
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
