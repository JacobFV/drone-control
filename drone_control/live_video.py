from __future__ import annotations

import os
import queue
import re
import socket
import threading
import time
from abc import ABC, abstractmethod
from collections import OrderedDict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterator

from drone_control.rtp_jpeg import (
    RtpJpegFrame,
    add_packet,
    assemble_jpeg,
    parse_rtp_jpeg_packet,
    start_frame,
)
from drone_control.transport import SO_BINDTODEVICE


VIDEO_INIT = bytes.fromhex("800000000000000000000000")
AUX_INIT = bytes.fromhex("80c9000100000000")
STREAM_START = b"\xef\x00\x04\x00"


@dataclass(frozen=True, slots=True)
class Frame:
    data: bytes
    metadata: dict[str, object] = field(default_factory=dict)


class FrameSource(ABC):
    """Source of real JPEG frames.

    Implementations must not synthesize fallback images. ``read`` returns
    ``None`` only when no real frame is available before the timeout or after
    the source has stopped.
    """

    @abstractmethod
    def start(self) -> None:
        """Open resources and make frames available to ``read``."""

    @abstractmethod
    def stop(self) -> None:
        """Close resources. Safe to call more than once."""

    @abstractmethod
    def read(self, timeout: float | None = None) -> Frame | None:
        """Return the next JPEG frame, or ``None`` if none is available."""

    def __enter__(self) -> FrameSource:
        self.start()
        return self

    def __exit__(self, *_exc: object) -> None:
        self.stop()


class DirectoryFrameSource(FrameSource):
    def __init__(self, frame_dir: str | Path, *, fps: float = 10.0) -> None:
        if fps <= 0:
            raise ValueError("fps must be positive")
        self.frame_dir = Path(frame_dir)
        self.fps = fps
        self._frames: list[Path] = []
        self._index = 0
        self._next_at = 0.0
        self._started = False

    def start(self) -> None:
        frames = sorted(self.frame_dir.glob("*.jpg"))
        if not self.frame_dir.is_dir():
            raise FileNotFoundError(f"frame directory not found: {self.frame_dir}")
        if not frames:
            raise FileNotFoundError(f"no .jpg frames found in: {self.frame_dir}")
        self._frames = frames
        self._index = 0
        self._next_at = time.monotonic()
        self._started = True

    def stop(self) -> None:
        self._started = False

    def read(self, timeout: float | None = None) -> Frame | None:
        if not self._started:
            raise RuntimeError("DirectoryFrameSource.start() must be called before read()")

        now = time.monotonic()
        wait = self._next_at - now
        if wait > 0:
            if timeout is not None and timeout < wait:
                time.sleep(max(0.0, timeout))
                return None
            time.sleep(wait)

        path = self._frames[self._index]
        loop = self._index == 0
        self._index = (self._index + 1) % len(self._frames)

        interval = 1.0 / self.fps
        self._next_at += interval
        if self._next_at < time.monotonic() - interval:
            self._next_at = time.monotonic() + interval

        return Frame(
            data=path.read_bytes(),
            metadata={
                "source": "directory",
                "path": str(path),
                "name": path.name,
                "index": self._index - 1 if self._index else len(self._frames) - 1,
                "loop": loop,
                "timestamp": time.time(),
            },
        )


@dataclass(slots=True)
class LiveDroneFrameSourceConfig:
    iface: str
    local_ip: str = ""
    drone_ip: str = "192.168.1.1"
    rtsp_port: int = 7070
    video_port: int = 32124
    aux_port: int = 32125
    drone_video_port: int = 53797
    bind_device: bool = True
    use_rtsp: bool = True
    start_ips: tuple[str, ...] = ()
    start_port: int = 8800
    queue_size: int = 8
    socket_poll_seconds: float = 0.005
    probe_interval_seconds: float = 0.5


class LiveDroneFrameSource(FrameSource):
    """Live WIFI_8K/Taixin camera source.

    ``start`` opens UDP sockets, optionally performs RTSP OPTIONS/DESCRIBE/
    SETUP/PLAY, and starts a background read loop. ``read`` returns completed
    JPEGs assembled from real RTP/JPEG packets via ``drone_control.rtp_jpeg``.
    If the stream has not delivered a complete decodable JPEG, ``read`` returns
    ``None`` after the requested timeout. No synthetic or placeholder frames
    are emitted.
    """

    def __init__(self, config: LiveDroneFrameSourceConfig) -> None:
        self.config = config
        self._frames: queue.Queue[Frame] = queue.Queue(maxsize=max(1, config.queue_size))
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._video_sock: socket.socket | None = None
        self._aux_sock: socket.socket | None = None
        self._start_sock: socket.socket | None = None
        self._rtsp_sock: socket.socket | None = None
        self._last_error: BaseException | None = None

    @property
    def last_error(self) -> BaseException | None:
        return self._last_error

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._last_error = None
        self._video_sock = self._make_udp_socket()
        self._video_sock.bind((self.config.local_ip, self.config.video_port))
        self._video_sock.setblocking(False)
        self._aux_sock = self._make_udp_socket()
        self._aux_sock.bind((self.config.local_ip, self.config.aux_port))
        self._aux_sock.setblocking(False)
        self._start_sock = self._make_udp_socket()
        self._start_sock.bind((self.config.local_ip, 0))
        self._start_sock.setblocking(False)

        self._thread = threading.Thread(target=self._run, name="live-drone-camera", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        for sock in (self._video_sock, self._aux_sock, self._start_sock, self._rtsp_sock):
            if sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass
        if self._thread and self._thread.is_alive() and self._thread is not threading.current_thread():
            self._thread.join(timeout=2.0)
        self._thread = None
        self._video_sock = None
        self._aux_sock = None
        self._start_sock = None
        self._rtsp_sock = None

    def read(self, timeout: float | None = None) -> Frame | None:
        try:
            return self._frames.get(timeout=timeout)
        except queue.Empty:
            return None

    def _run(self) -> None:
        assert self._video_sock is not None
        assert self._aux_sock is not None
        assert self._start_sock is not None

        frame_parts: OrderedDict[bytes, RtpJpegFrame] = OrderedDict()
        last_quantization_tables: bytes | None = None
        packet_count = 0
        decoded_count = 0
        last_probe = 0.0

        try:
            if self.config.use_rtsp:
                self._rtsp_sock, server_ports = _rtsp_start_stream(
                    self.config,
                    self._video_sock,
                    self._aux_sock,
                )
                if server_ports:
                    self.config.drone_video_port = server_ports[1]

            while not self._stop.is_set():
                now = time.monotonic()
                if now - last_probe >= self.config.probe_interval_seconds:
                    _send_camera_init(
                        self._video_sock,
                        self._aux_sock,
                        self.config.drone_ip,
                        self.config.drone_video_port - 1,
                    )
                    _send_start_probes(
                        self._start_sock,
                        self.config.start_ips or ("192.168.169.1", self.config.drone_ip),
                        self.config.start_port,
                    )
                    last_probe = now

                try:
                    payload, source = self._video_sock.recvfrom(65535)
                except BlockingIOError:
                    time.sleep(self.config.socket_poll_seconds)
                    continue
                except OSError:
                    if self._stop.is_set():
                        return
                    raise

                packet_count += 1
                packet = parse_rtp_jpeg_packet(payload)
                if packet is None:
                    continue
                if packet.quantization_tables:
                    last_quantization_tables = packet.quantization_tables

                frame = frame_parts.get(packet.frame_key)
                if frame is None:
                    frame = start_frame(packet, last_quantization_tables)
                    frame_parts[packet.frame_key] = frame
                add_packet(frame, packet)

                if frame.marker_seen:
                    jpeg = assemble_jpeg(frame)
                    del frame_parts[packet.frame_key]
                    if jpeg is not None:
                        decoded_count += 1
                        self._put_frame(
                            Frame(
                                data=jpeg,
                                metadata={
                                    "source": "live-drone",
                                    "source_addr": f"{source[0]}:{source[1]}",
                                    "frame_key": packet.frame_key.hex(),
                                    "width": frame.width,
                                    "height": frame.height,
                                    "packets": packet_count,
                                    "decoded": decoded_count,
                                    "timestamp": time.time(),
                                },
                            )
                        )

                while len(frame_parts) > 4:
                    _, old_frame = frame_parts.popitem(last=False)
                    jpeg = assemble_jpeg(old_frame)
                    if jpeg is not None:
                        decoded_count += 1
                        self._put_frame(
                            Frame(
                                data=jpeg,
                                metadata={
                                    "source": "live-drone",
                                    "frame_key": old_frame.frame_key.hex(),
                                    "width": old_frame.width,
                                    "height": old_frame.height,
                                    "packets": packet_count,
                                    "decoded": decoded_count,
                                    "timestamp": time.time(),
                                },
                            )
                        )
        except BaseException as exc:
            self._last_error = exc
        finally:
            self.stop()

    def _put_frame(self, frame: Frame) -> None:
        try:
            self._frames.put_nowait(frame)
        except queue.Full:
            try:
                self._frames.get_nowait()
            except queue.Empty:
                pass
            self._frames.put_nowait(frame)

    def _make_udp_socket(self) -> socket.socket:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        if self.config.bind_device and self.config.iface:
            if os.geteuid() != 0:
                sock.close()
                raise PermissionError("SO_BINDTODEVICE requires root; use bind_device=False or run as root")
            sock.setsockopt(socket.SOL_SOCKET, SO_BINDTODEVICE, self.config.iface.encode() + b"\0")
        return sock


def mjpeg_chunks(
    source: FrameSource,
    *,
    boundary: str = "frame",
    timeout: float | None = None,
    auto_start: bool = True,
) -> Iterator[bytes]:
    boundary_bytes = boundary.encode("ascii")
    if auto_start:
        source.start()
    try:
        while True:
            frame = source.read(timeout=timeout)
            if frame is None:
                continue
            header = (
                b"--"
                + boundary_bytes
                + b"\r\nContent-Type: image/jpeg\r\n"
                + f"Content-Length: {len(frame.data)}\r\n\r\n".encode("ascii")
            )
            yield header + frame.data + b"\r\n"
    finally:
        if auto_start:
            source.stop()


def _rtsp_start_stream(
    config: LiveDroneFrameSourceConfig,
    video_sock: socket.socket,
    aux_sock: socket.socket,
) -> tuple[socket.socket | None, tuple[int, int] | None]:
    sock = _make_tcp_socket(config)
    try:
        sock.connect((config.drone_ip, config.rtsp_port))
        base = f"rtsp://{config.drone_ip}:{config.rtsp_port}/webcam"
        _send_rtsp(sock, f"OPTIONS {base} RTSP/1.0\r\nCSeq: 1\r\nUser-Agent: ijkplayer\r\n\r\n")
        _read_rtsp_response(sock)
        _send_rtsp(
            sock,
            f"DESCRIBE {base} RTSP/1.0\r\n"
            "Accept: application/sdp\r\n"
            "CSeq: 2\r\n"
            "User-Agent: ijkplayer\r\n\r\n",
        )
        _read_rtsp_response(sock)
        _send_rtsp(
            sock,
            f"SETUP {base}/track0 RTSP/1.0\r\n"
            f"Transport: RTP/AVP/UDP;unicast;client_port={config.video_port}-{config.aux_port}\r\n"
            "CSeq: 3\r\n"
            "User-Agent: ijkplayer\r\n\r\n",
        )
        setup_response = _read_rtsp_response(sock)
        server_ports = _parse_server_ports(setup_response)
        session = _parse_session(setup_response)
        if server_ports:
            _send_camera_init(video_sock, aux_sock, config.drone_ip, server_ports[0])
        session_header = f"Session: {session}\r\n" if session else ""
        _send_rtsp(
            sock,
            f"PLAY {base}/ RTSP/1.0\r\n"
            "Range: npt=0.000-\r\n"
            "CSeq: 4\r\n"
            "User-Agent: ijkplayer\r\n"
            f"{session_header}\r\n",
        )
        _read_rtsp_response(sock)
        return sock, server_ports
    except OSError:
        sock.close()
        return None, None


def _make_tcp_socket(config: LiveDroneFrameSourceConfig) -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    if config.bind_device and config.iface:
        if os.geteuid() != 0:
            sock.close()
            raise PermissionError("SO_BINDTODEVICE requires root; use bind_device=False or run as root")
        sock.setsockopt(socket.SOL_SOCKET, SO_BINDTODEVICE, config.iface.encode() + b"\0")
    if config.local_ip:
        sock.bind((config.local_ip, 0))
    sock.settimeout(2.0)
    return sock


def _send_rtsp(sock: socket.socket, request: str) -> None:
    sock.sendall(request.encode("ascii"))


def _read_rtsp_response(sock: socket.socket) -> str:
    data = bytearray()
    while b"\r\n\r\n" not in data:
        chunk = sock.recv(4096)
        if not chunk:
            break
        data.extend(chunk)
    header, _, rest = bytes(data).partition(b"\r\n\r\n")
    match = re.search(rb"(?im)^Content-Length:\s*(\d+)\s*$", header)
    if match:
        remaining = int(match.group(1)) - len(rest)
        while remaining > 0:
            chunk = sock.recv(remaining)
            if not chunk:
                break
            data.extend(chunk)
            remaining -= len(chunk)
    return bytes(data).decode("ascii", errors="replace")


def _parse_server_ports(response: str) -> tuple[int, int] | None:
    match = re.search(r"server_port=(\d+)-(\d+)", response, re.IGNORECASE)
    if not match:
        return None
    return int(match.group(1)), int(match.group(2))


def _parse_session(response: str) -> str:
    match = re.search(r"(?im)^Session:\s*([^;\r\n]+)", response)
    return match.group(1).strip() if match else ""


def _send_camera_init(video_sock: socket.socket, aux_sock: socket.socket, ip: str, video_port: int) -> None:
    if video_port <= 0:
        return
    try:
        video_sock.sendto(VIDEO_INIT, (ip, video_port))
        aux_sock.sendto(AUX_INIT, (ip, video_port + 1))
    except (BlockingIOError, OSError):
        return


def _send_start_probes(sock: socket.socket, ips: tuple[str, ...], port: int) -> None:
    for ip in ips:
        try:
            sock.sendto(STREAM_START, (ip, port))
        except (BlockingIOError, OSError):
            continue
