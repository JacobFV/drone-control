#!/usr/bin/env python3
from __future__ import annotations

import argparse
import socket
import struct
import sys
import time
from collections import OrderedDict
from itertools import cycle
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drone_control.actions import DroneAction
from drone_control.protocols import make_protocol
from drone_control.transport import SO_BINDTODEVICE

from pcap_summary import parse_udp_from_radiotap, read_pcap


AUX_REQUEST = bytes.fromhex(
    "81c900077b7a79797b7a797800ffffff0001ed6e000000eb8c274b0200016a5f"
    "81ca00047b7a797901096c6f63616c686f737400"
)
VIDEO_INIT = bytes.fromhex("800000000000000000000000")
AUX_INIT = bytes.fromhex("80c9000100000000")
STREAM_START = b"\xef\x00\x04\x00"


def main() -> int:
    parser = argparse.ArgumentParser(description="Capture the WIFI_8K UDP camera stream.")
    parser.add_argument("--iface", required=True)
    parser.add_argument("--drone-ip", default="192.168.1.1")
    parser.add_argument("--video-port", type=int, default=32124, help="Local UDP port that receives video.")
    parser.add_argument("--aux-port", type=int, default=32125, help="Local UDP port used by the app for video aux traffic.")
    parser.add_argument("--drone-video-port", type=int, default=53797, help="Drone UDP aux/video-control port observed in captures.")
    parser.add_argument(
        "--drone-port-scan",
        help="Optional drone video-port range to probe as start:end[:step]; aux port is video port + 1.",
    )
    parser.add_argument("--drone-port-scan-batch", type=int, default=64, help="Camera port pairs to probe per scan tick.")
    parser.add_argument("--control-port", type=int, default=56906, help="Local UDP port for neutral control packets.")
    parser.add_argument("--drone-control-port", type=int, default=7099)
    parser.add_argument("--no-control", action="store_true", help="Do not send neutral control while capturing.")
    parser.add_argument("--start-local-port", type=int, default=49474, help="Local UDP port used for ef000400 stream-start probes.")
    parser.add_argument("--start-ip", action="append", default=[], help="IP to send ef000400 stream-start probe to. Can repeat.")
    parser.add_argument("--start-port", type=int, default=8800)
    parser.add_argument("--aux-replay-pcap", type=Path, help="Replay app aux-video requests from a monitor pcap.")
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
    start_sock.bind(("", args.start_local_port))
    start_sock.setblocking(False)

    control_sock = make_udp_socket(args.iface, bind_device)
    control_sock.bind(("", args.control_port))
    control_protocol = make_protocol("wifi_8k_prefixed_short")
    neutral_packet = control_protocol.build(DroneAction.neutral())
    keepalive_packet = getattr(control_protocol, "keepalive", lambda: b"")()

    start_ips = args.start_ip or ["192.168.169.1", args.drone_ip]
    default_pcap = Path("captures/drone_monitor_20260512_110655_ch1.pcap")
    aux_replay_pcap = args.aux_replay_pcap or (default_pcap if default_pcap.exists() else None)
    aux_requests = load_aux_requests(aux_replay_pcap) if aux_replay_pcap else [AUX_REQUEST]
    aux_iter = cycle(aux_requests)
    scan_range = parse_port_range(args.drone_port_scan) if args.drone_port_scan else None
    scan_cursor = scan_range[0] if scan_range else 0
    frames: OrderedDict[bytes, dict[int, bytes]] = OrderedDict()
    frame_count = 0
    packet_count = 0
    byte_count = 0
    last_probe = 0.0
    last_scan = 0.0
    last_control = 0.0
    last_keepalive = 0.0
    deadline = time.monotonic() + args.seconds

    print(f"Video socket: 0.0.0.0:{args.video_port}")
    print(f"Aux socket:   0.0.0.0:{args.aux_port} -> {args.drone_ip}:{args.drone_video_port}")
    if args.drone_port_scan:
        print(f"Port scan:    {args.drone_port_scan} video ports; aux is video+1")
    if not args.no_control:
        print(f"Control:      0.0.0.0:{args.control_port} -> {args.drone_ip}:{args.drone_control_port}")
    print(f"Aux requests: {len(aux_requests)}" + (f" from {aux_replay_pcap}" if aux_replay_pcap else " built-in"))
    print(f"Start probe:  0.0.0.0:{args.start_local_port} -> {start_ips} port={args.start_port}")
    print(f"Raw output:   {raw_path}")
    print(f"Frame output: {frame_dir}")

    with raw_path.open("wb") as raw:
        try:
            while time.monotonic() < deadline:
                now = time.monotonic()
                if now - last_probe >= 0.5:
                    send_start_probes(start_sock, start_ips, args.start_port)
                    send_camera_init(video_sock, aux_sock, args.drone_ip, args.drone_video_port - 1)
                    for _ in range(min(4, len(aux_requests))):
                        aux_sock.sendto(next(aux_iter), (args.drone_ip, args.drone_video_port))
                    last_probe = now

                if scan_range and now - last_scan >= 0.05:
                    scan_cursor = scan_camera_ports(
                        video_sock,
                        aux_sock,
                        args.drone_ip,
                        scan_range,
                        scan_cursor,
                        max(1, args.drone_port_scan_batch),
                    )
                    last_scan = now

                if not args.no_control and now - last_control >= 0.05:
                    control_sock.sendto(neutral_packet, (args.drone_ip, args.drone_control_port))
                    last_control = now
                if not args.no_control and keepalive_packet and now - last_keepalive >= 1.0:
                    control_sock.sendto(keepalive_packet, (args.drone_ip, args.drone_control_port))
                    last_keepalive = now

                drain_aux_replies(aux_sock)

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
            control_sock.close()

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


def send_camera_init(video_sock: socket.socket, aux_sock: socket.socket, ip: str, video_port: int) -> None:
    if video_port <= 0:
        return
    try:
        video_sock.sendto(VIDEO_INIT, (ip, video_port))
        aux_sock.sendto(AUX_INIT, (ip, video_port + 1))
    except BlockingIOError:
        return
    except OSError as exc:
        print(f"camera init error {ip}:{video_port}/{video_port + 1}: {exc}")


def scan_camera_ports(
    video_sock: socket.socket,
    aux_sock: socket.socket,
    ip: str,
    port_range: tuple[int, int, int],
    cursor: int,
    batch: int,
) -> int:
    start, end, step = port_range
    video_port = cursor
    for _ in range(batch):
        if video_port > end:
            video_port = start
        send_camera_init(video_sock, aux_sock, ip, video_port)
        video_port += step
    return video_port


def parse_port_range(spec: str) -> tuple[int, int, int]:
    parts = spec.split(":")
    if len(parts) not in {2, 3}:
        raise ValueError("expected start:end[:step]")
    start = int(parts[0])
    end = int(parts[1])
    step = int(parts[2]) if len(parts) == 3 else 2
    if not (1 <= start <= 65535 and 1 <= end <= 65535 and step >= 1):
        raise ValueError("ports must be in 1..65535 and step must be positive")
    if start > end:
        raise ValueError("start must be <= end")
    return start, end, step


def drain_aux_replies(sock: socket.socket) -> None:
    while True:
        try:
            sock.recvfrom(4096)
        except BlockingIOError:
            return


def load_aux_requests(path: Path) -> list[bytes]:
    requests: list[bytes] = []
    seen = set()
    for frame in read_pcap(path):
        parsed = parse_udp_from_radiotap(frame)
        if parsed is None:
            continue
        src, sport, dst, dport, payload = parsed
        if src == "192.168.1.101" and sport == 32125 and dst == "192.168.1.1" and dport == 53797:
            if payload in seen:
                continue
            seen.add(payload)
            requests.append(payload)
    return requests or [AUX_REQUEST]


def parse_video_packet(payload: bytes) -> tuple[bytes | None, int | None, bytes | None]:
    if len(payload) < 24:
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
