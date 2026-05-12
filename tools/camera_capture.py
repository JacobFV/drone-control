#!/usr/bin/env python3
from __future__ import annotations

import argparse
import socket
import struct
import sys
import time
from collections import OrderedDict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drone_control.transport import SO_BINDTODEVICE


AUX_REQUEST = bytes.fromhex(
    "81c900077b7a79797b7a797800ffffff0001ed6e000000eb8c274b0200016a5f"
    "81ca00047b7a797901096c6f63616c686f737400"
)
STREAM_START = b"\xef\x00\x04\x00"
VIDEO_MAGIC = b"\x7b\x7a\x79\x78"


def main() -> int:
    parser = argparse.ArgumentParser(description="Capture the WIFI_8K UDP camera stream.")
    parser.add_argument("--iface", required=True)
    parser.add_argument("--drone-ip", default="192.168.1.1")
    parser.add_argument("--video-port", type=int, default=32124, help="Local UDP port that receives video.")
    parser.add_argument("--aux-port", type=int, default=32125, help="Local UDP port used by the app for video aux traffic.")
    parser.add_argument("--drone-video-port", type=int, default=53797, help="Drone UDP aux/video-control port observed in captures.")
    parser.add_argument("--start-ip", action="append", default=[], help="IP to send ef000400 stream-start probe to. Can repeat.")
    parser.add_argument("--start-port", type=int, default=8800)
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--out-dir", type=Path, default=Path("camera_captures"))
    parser.add_argument("--no-bind-device", action="store_true")
    args = parser.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    raw_path = args.out_dir / f"camera_udp_{stamp}.bin"
    frame_dir = args.out_dir / f"frames_{stamp}"
    frame_dir.mkdir(parents=True, exist_ok=True)

    bind_device = not args.no_bind_device
    video_sock = make_udp_socket(args.iface, bind_device)
    video_sock.bind(("", args.video_port))
    video_sock.setblocking(False)

    aux_sock = make_udp_socket(args.iface, bind_device)
    aux_sock.bind(("", args.aux_port))
    aux_sock.setblocking(False)

    start_sock = make_udp_socket(args.iface, bind_device)
    start_sock.bind(("", 0))
    start_sock.setblocking(False)

    start_ips = args.start_ip or ["192.168.169.1", args.drone_ip]
    frames: OrderedDict[bytes, dict[int, bytes]] = OrderedDict()
    frame_count = 0
    packet_count = 0
    byte_count = 0
    last_probe = 0.0
    deadline = time.monotonic() + args.seconds

    print(f"Video socket: 0.0.0.0:{args.video_port}")
    print(f"Aux socket:   0.0.0.0:{args.aux_port} -> {args.drone_ip}:{args.drone_video_port}")
    print(f"Start probe:  {start_ips} port={args.start_port}")
    print(f"Raw output:   {raw_path}")
    print(f"Frame output: {frame_dir}")

    with raw_path.open("wb") as raw:
        try:
            while time.monotonic() < deadline:
                now = time.monotonic()
                if now - last_probe >= 0.5:
                    send_start_probes(start_sock, start_ips, args.start_port)
                    aux_sock.sendto(AUX_REQUEST, (args.drone_ip, args.drone_video_port))
                    last_probe = now

                try:
                    payload, source = video_sock.recvfrom(65535)
                except BlockingIOError:
                    time.sleep(0.005)
                    continue

                packet_count += 1
                byte_count += len(payload)
                raw.write(struct.pack("!dHI", time.time(), len(source[0]), len(payload)))
                raw.write(source[0].encode())
                raw.write(struct.pack("!H", source[1]))
                raw.write(payload)

                frame_key, offset, data = parse_video_packet(payload)
                if frame_key is None or offset is None or data is None:
                    continue
                chunks = frames.setdefault(frame_key, {})
                chunks.setdefault(offset, data)
                while len(frames) > 4:
                    old_key, old_chunks = frames.popitem(last=False)
                    frame_count += write_frame(frame_dir, frame_count, old_key, old_chunks)
        finally:
            for frame_key, chunks in frames.items():
                frame_count += write_frame(frame_dir, frame_count, frame_key, chunks)
            video_sock.close()
            aux_sock.close()
            start_sock.close()

    print(f"Captured packets={packet_count} bytes={byte_count} frames={frame_count}")
    return 0 if packet_count else 1


def make_udp_socket(iface: str, bind_device: bool) -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if bind_device:
        sock.setsockopt(socket.SOL_SOCKET, SO_BINDTODEVICE, iface.encode() + b"\0")
    return sock


def send_start_probes(sock: socket.socket, ips: list[str], port: int) -> None:
    for ip in ips:
        try:
            sock.sendto(STREAM_START, (ip, port))
        except OSError as exc:
            print(f"start probe error {ip}:{port}: {exc}")


def parse_video_packet(payload: bytes) -> tuple[bytes | None, int | None, bytes | None]:
    if len(payload) < 24 or payload[8:12] != VIDEO_MAGIC:
        return None, None, None
    frame_key = payload[4:8]
    offset = int.from_bytes(payload[14:16], "big")
    header_len = 156 if offset == 0 and len(payload) >= 156 else 24
    return frame_key, offset, payload[header_len:]


def write_frame(frame_dir: Path, index: int, frame_key: bytes, chunks: dict[int, bytes]) -> int:
    if not chunks:
        return 0
    size = max(offset + len(data) for offset, data in chunks.items())
    if size <= 0:
        return 0
    frame = bytearray(size)
    for offset, data in chunks.items():
        frame[offset:offset + len(data)] = data
    suffix = "jpgish" if frame.find(b"\xff\xd9") >= 0 else "bin"
    path = frame_dir / f"frame_{index:05d}_{frame_key.hex()}.{suffix}"
    path.write_bytes(frame)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
