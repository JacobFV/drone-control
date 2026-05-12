from __future__ import annotations

import struct
from dataclasses import dataclass, field


JPEG_PAYLOAD_TYPE = 26


@dataclass
class RtpJpegPacket:
    frame_key: bytes
    sequence: int
    marker: bool
    offset: int
    jpeg_type: int
    q: int
    width: int
    height: int
    restart_interval: int | None
    quantization_tables: bytes | None
    data: bytes


@dataclass
class RtpJpegFrame:
    frame_key: bytes
    jpeg_type: int
    width: int
    height: int
    restart_interval: int | None = None
    quantization_tables: bytes | None = None
    chunks: dict[int, bytes] = field(default_factory=dict)
    marker_seen: bool = False


def parse_rtp_jpeg_packet(payload: bytes) -> RtpJpegPacket | None:
    if len(payload) < 20:
        return None
    first = payload[0]
    if first >> 6 != 2:
        return None

    cc = first & 0x0f
    header_len = 12 + (cc * 4)
    if len(payload) < header_len + 8:
        return None
    if first & 0x10:
        if len(payload) < header_len + 4:
            return None
        extension_words = int.from_bytes(payload[header_len + 2:header_len + 4], "big")
        header_len += 4 + (extension_words * 4)
        if len(payload) < header_len + 8:
            return None

    payload_type = payload[1] & 0x7f
    if payload_type != JPEG_PAYLOAD_TYPE:
        return None

    marker = bool(payload[1] & 0x80)
    sequence = int.from_bytes(payload[2:4], "big")
    frame_key = payload[4:8]

    jpeg_header = header_len
    offset = int.from_bytes(payload[jpeg_header + 1:jpeg_header + 4], "big")
    jpeg_type = payload[jpeg_header + 4]
    q = payload[jpeg_header + 5]
    width = payload[jpeg_header + 6] * 8
    height = payload[jpeg_header + 7] * 8
    data_start = jpeg_header + 8

    restart_interval: int | None = None
    if jpeg_type & 0x40:
        if len(payload) < data_start + 4:
            return None
        restart_interval = int.from_bytes(payload[data_start:data_start + 2], "big")
        data_start += 4

    quantization_tables = None
    if q >= 128 and offset == 0:
        if len(payload) < data_start + 4:
            return None
        table_length = int.from_bytes(payload[data_start + 2:data_start + 4], "big")
        data_start += 4
        if len(payload) < data_start + table_length:
            return None
        quantization_tables = payload[data_start:data_start + table_length]
        data_start += table_length

    return RtpJpegPacket(
        frame_key=frame_key,
        sequence=sequence,
        marker=marker,
        offset=offset,
        jpeg_type=jpeg_type,
        q=q,
        width=width,
        height=height,
        restart_interval=restart_interval,
        quantization_tables=quantization_tables,
        data=payload[data_start:],
    )


def start_frame(packet: RtpJpegPacket, fallback_quantization_tables: bytes | None = None) -> RtpJpegFrame:
    return RtpJpegFrame(
        frame_key=packet.frame_key,
        jpeg_type=packet.jpeg_type,
        width=packet.width,
        height=packet.height,
        restart_interval=packet.restart_interval,
        quantization_tables=packet.quantization_tables or fallback_quantization_tables,
    )


def add_packet(frame: RtpJpegFrame, packet: RtpJpegPacket) -> None:
    frame.width = packet.width or frame.width
    frame.height = packet.height or frame.height
    frame.jpeg_type = packet.jpeg_type
    if packet.restart_interval is not None:
        frame.restart_interval = packet.restart_interval
    if packet.quantization_tables:
        frame.quantization_tables = packet.quantization_tables
    frame.chunks.setdefault(packet.offset, packet.data)
    frame.marker_seen = frame.marker_seen or packet.marker


def assemble_scan_data(frame: RtpJpegFrame) -> bytes:
    if not frame.chunks:
        return b""
    size = max(offset + len(data) for offset, data in frame.chunks.items())
    scan = bytearray(size)
    for offset, data in frame.chunks.items():
        scan[offset:offset + len(data)] = data
    return bytes(scan)


def assemble_jpeg(frame: RtpJpegFrame) -> bytes | None:
    if not frame.quantization_tables:
        return None

    scan = assemble_scan_data(frame)
    if not scan:
        return None

    jpeg = bytearray(b"\xff\xd8")
    jpeg.extend(make_dqt(frame.quantization_tables))
    jpeg.extend(make_sof0(frame.width, frame.height, frame.jpeg_type))
    jpeg.extend(STANDARD_DHT)
    if frame.restart_interval:
        jpeg.extend(segment(b"\xff\xdd", struct.pack(">H", frame.restart_interval)))
    jpeg.extend(make_sos())
    jpeg.extend(scan)
    if not jpeg.endswith(b"\xff\xd9"):
        jpeg.extend(b"\xff\xd9")
    return bytes(jpeg)


def make_dqt(tables: bytes) -> bytes:
    body = bytearray()
    if len(tables) >= 64:
        body.append(0x00)
        body.extend(tables[:64])
    if len(tables) >= 128:
        body.append(0x01)
        body.extend(tables[64:128])
    elif len(tables) >= 64:
        body.append(0x01)
        body.extend(tables[:64])
    return segment(b"\xff\xdb", bytes(body))


def make_sof0(width: int, height: int, jpeg_type: int) -> bytes:
    # RFC 2435 type 1 is 4:2:0. Type 0 is 4:2:2. This drone sends type 65,
    # which is type 1 with the restart-marker bit set.
    normalized_type = jpeg_type & 0x3f
    y_sampling = 0x21 if normalized_type == 0 else 0x22
    body = struct.pack(">BHHB", 8, height, width, 3)
    body += bytes(
        [
            1,
            y_sampling,
            0,
            2,
            0x11,
            1,
            3,
            0x11,
            1,
        ]
    )
    return segment(b"\xff\xc0", body)


def make_sos() -> bytes:
    body = bytes(
        [
            3,
            1,
            0x00,
            2,
            0x11,
            3,
            0x11,
            0,
            63,
            0,
        ]
    )
    return segment(b"\xff\xda", body)


def segment(marker: bytes, body: bytes) -> bytes:
    return marker + struct.pack(">H", len(body) + 2) + body


def dht_table(table_class: int, table_id: int, counts: list[int], values: list[int]) -> bytes:
    if len(counts) != 16:
        raise ValueError("DHT counts must contain 16 entries")
    if sum(counts) != len(values):
        raise ValueError("DHT count total must match value count")
    return bytes([(table_class << 4) | table_id, *counts, *values])


_DC_LUMA_COUNTS = [0, 1, 5, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0]
_DC_CHROMA_COUNTS = [0, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0]
_AC_LUMA_COUNTS = [0, 2, 1, 3, 3, 2, 4, 3, 5, 5, 4, 4, 0, 0, 1, 125]
_AC_CHROMA_COUNTS = [0, 2, 1, 2, 4, 4, 3, 4, 7, 5, 4, 4, 0, 1, 2, 119]

_DC_VALUES = list(range(12))
_AC_LUMA_VALUES = [
    0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12,
    0x21, 0x31, 0x41, 0x06, 0x13, 0x51, 0x61, 0x07,
    0x22, 0x71, 0x14, 0x32, 0x81, 0x91, 0xa1, 0x08,
    0x23, 0x42, 0xb1, 0xc1, 0x15, 0x52, 0xd1, 0xf0,
    0x24, 0x33, 0x62, 0x72, 0x82, 0x09, 0x0a, 0x16,
    0x17, 0x18, 0x19, 0x1a, 0x25, 0x26, 0x27, 0x28,
    0x29, 0x2a, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
    0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49,
    0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59,
    0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69,
    0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79,
    0x7a, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
    0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98,
    0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7,
    0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6,
    0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5,
    0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4,
    0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe1, 0xe2,
    0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea,
    0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8,
    0xf9, 0xfa,
]
_AC_CHROMA_VALUES = [
    0x00, 0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x21,
    0x31, 0x06, 0x12, 0x41, 0x51, 0x07, 0x61, 0x71,
    0x13, 0x22, 0x32, 0x81, 0x08, 0x14, 0x42, 0x91,
    0xa1, 0xb1, 0xc1, 0x09, 0x23, 0x33, 0x52, 0xf0,
    0x15, 0x62, 0x72, 0xd1, 0x0a, 0x16, 0x24, 0x34,
    0xe1, 0x25, 0xf1, 0x17, 0x18, 0x19, 0x1a, 0x26,
    0x27, 0x28, 0x29, 0x2a, 0x35, 0x36, 0x37, 0x38,
    0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48,
    0x49, 0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58,
    0x59, 0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68,
    0x69, 0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78,
    0x79, 0x7a, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87,
    0x88, 0x89, 0x8a, 0x92, 0x93, 0x94, 0x95, 0x96,
    0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5,
    0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4,
    0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3,
    0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2,
    0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda,
    0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9,
    0xea, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8,
    0xf9, 0xfa,
]

STANDARD_DHT = segment(
    b"\xff\xc4",
    dht_table(0, 0, _DC_LUMA_COUNTS, _DC_VALUES)
    + dht_table(1, 0, _AC_LUMA_COUNTS, _AC_LUMA_VALUES)
    + dht_table(0, 1, _DC_CHROMA_COUNTS, _DC_VALUES)
    + dht_table(1, 1, _AC_CHROMA_COUNTS, _AC_CHROMA_VALUES),
)
