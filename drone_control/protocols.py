from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

from .actions import DroneAction, clamp_byte


class PacketProtocol(Protocol):
    name: str

    def build(self, action: DroneAction) -> bytes:
        ...


def _xor(values: bytes | bytearray) -> int:
    checksum = 0
    for value in values:
        checksum ^= value
    return checksum & 0xFF


def _right_data(value: int) -> int:
    value &= 0xFF
    if value in (0x66, 0x99):
        return (value + 1) & 0xFF
    return value


@dataclass
class WifiCamShortProtocol:
    """8-byte raw WiFi_CAM command: 66 R P T Y FLAGS XOR 99."""

    name: str = "wifi_cam_short"

    FLAG_TAKEOFF = 0x01
    FLAG_LAND = 0x02
    FLAG_EMERGENCY = 0x04
    FLAG_FLIP = 0x08
    FLAG_HEADLESS = 0x10
    FLAG_CALIBRATE = 0x80

    def build(self, action: DroneAction) -> bytes:
        action = action.sanitized()
        packet = bytearray(8)
        packet[0] = 0x66
        packet[1] = _right_data(action.roll)
        packet[2] = _right_data(action.pitch)
        packet[3] = _right_data(action.throttle)
        packet[4] = _right_data(action.yaw)
        flags = 0
        if action.takeoff:
            flags |= self.FLAG_TAKEOFF
        if action.land:
            flags |= self.FLAG_LAND
        if action.emergency_stop:
            flags |= self.FLAG_EMERGENCY
        if action.flip:
            flags |= self.FLAG_FLIP
        if action.headless:
            flags |= self.FLAG_HEADLESS
        if action.calibrate:
            flags |= self.FLAG_CALIBRATE
        packet[5] = flags & 0xFF
        packet[6] = _right_data(_xor(packet[1:6]))
        packet[7] = 0x99
        return bytes(packet)


@dataclass
class Wifi8kPrefixedShortProtocol:
    """9-byte WIFI_8K command observed in captures: 03 66 R P T Y FLAGS XOR 99."""

    name: str = "wifi_8k_prefixed_short"

    FLAG_TAKEOFF = 0x01
    FLAG_LAND = 0x02
    FLAG_EMERGENCY = 0x04
    FLAG_FLIP = 0x08
    FLAG_HEADLESS = 0x10
    FLAG_CALIBRATE = 0x80

    def build(self, action: DroneAction) -> bytes:
        action = action.sanitized()
        flags = 0
        if action.takeoff:
            flags |= self.FLAG_TAKEOFF
        if action.land:
            flags |= self.FLAG_LAND
        if action.emergency_stop:
            flags |= self.FLAG_EMERGENCY
        if action.flip:
            flags |= self.FLAG_FLIP
        if action.headless:
            flags |= self.FLAG_HEADLESS
        if action.calibrate:
            flags |= self.FLAG_CALIBRATE

        controls = bytes([
            clamp_byte(action.roll),
            clamp_byte(action.pitch),
            clamp_byte(action.throttle),
            clamp_byte(action.yaw),
            flags & 0xFF,
        ])
        return bytes([0x03, 0x66]) + controls + bytes([_xor(controls), 0x99])

    def keepalive(self) -> bytes:
        return bytes([0x01, 0x01])


@dataclass
class WifiCamExtendedProtocol:
    """20-byte raw WiFi_CAM/WiFi-UAV stick command: 66 14 R P T Y ... XOR 99."""

    name: str = "wifi_cam_extended"

    FLAG_TAKEOFF_OR_LAND = 0x01
    FLAG_EMERGENCY = 0x02
    FLAG_CALIBRATE = 0x04
    FLAG_FLIP = 0x08
    FLAG_HEADLESS = 0x01
    FLAG_ALTITUDE_HOLD = 0x02

    def build(self, action: DroneAction) -> bytes:
        action = action.sanitized()
        packet = bytearray(20)
        packet[0] = 0x66
        packet[1] = 0x14
        packet[2] = _right_data(action.roll)
        packet[3] = _right_data(action.pitch)
        packet[4] = _right_data(action.throttle)
        packet[5] = _right_data(action.yaw)
        flags1 = 0
        flags2 = self.FLAG_ALTITUDE_HOLD
        if action.takeoff or action.land:
            flags1 |= self.FLAG_TAKEOFF_OR_LAND
        if action.emergency_stop:
            flags1 |= self.FLAG_EMERGENCY
        if action.calibrate:
            flags1 |= self.FLAG_CALIBRATE
        if action.flip:
            flags1 |= self.FLAG_FLIP
        if action.headless:
            flags2 |= self.FLAG_HEADLESS
        packet[6] = flags1 & 0xFF
        packet[7] = flags2 & 0xFF
        packet[18] = _right_data(_xor(packet[2:18]))
        packet[19] = 0x99
        return bytes(packet)


@dataclass
class WifiUavEnvelopeProtocol:
    """
    Extended WiFi-UAV packet used by the WiFi UAV app family.

    This wraps a 20-byte 66 14 command inside the observed ef 02 envelope and
    maintains the rolling counters present in captures.
    """

    name: str = "wifi_uav_envelope"
    ctr1: int = 0x0000
    ctr2: int = 0x0001
    ctr3: int = 0x0002

    _HEADER = bytes([0xEF, 0x02, 0x7C, 0x00, 0x02, 0x02, 0x00, 0x01, 0x02, 0x00, 0x00, 0x00])
    _COUNTER1_SUFFIX = bytes([0x00, 0x00, 0x14, 0x00, 0x66, 0x14])
    _CONTROL_SUFFIX = bytes(10)
    _CHECKSUM_SUFFIX = bytes([0x99]) + bytes(44) + bytes([0x32, 0x4B, 0x14, 0x2D, 0x00, 0x00])
    _COUNTER2_SUFFIX = bytes([
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
        0x00, 0x00, 0x14, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF,
    ])
    _COUNTER3_SUFFIX = bytes([
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x03, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00,
    ])

    FLAG_TAKEOFF_OR_LAND = 0x01
    FLAG_EMERGENCY = 0x02
    FLAG_CALIBRATE = 0x04
    FLAG_FLIP = 0x08

    def build(self, action: DroneAction) -> bytes:
        action = action.sanitized()
        c1 = self.ctr1.to_bytes(2, "little")
        c2 = self.ctr2.to_bytes(2, "little")
        c3 = self.ctr3.to_bytes(2, "little")
        self.ctr1 = (self.ctr1 + 1) & 0xFFFF
        self.ctr2 = (self.ctr2 + 1) & 0xFFFF
        self.ctr3 = (self.ctr3 + 1) & 0xFFFF

        command = 0
        if action.takeoff or action.land:
            command |= self.FLAG_TAKEOFF_OR_LAND
        if action.emergency_stop:
            command |= self.FLAG_EMERGENCY
        if action.calibrate:
            command |= self.FLAG_CALIBRATE
        if action.flip:
            command |= self.FLAG_FLIP
        headless = 0x03 if action.headless else 0x02
        controls = bytes([
            clamp_byte(action.roll),
            clamp_byte(action.pitch),
            clamp_byte(action.throttle),
            clamp_byte(action.yaw),
            command & 0xFF,
            headless & 0xFF,
        ])

        packet = bytearray()
        packet += self._HEADER
        packet += c1 + self._COUNTER1_SUFFIX
        packet += controls
        packet += self._CONTROL_SUFFIX
        packet.append(_xor(controls))
        packet += self._CHECKSUM_SUFFIX
        packet += c2 + self._COUNTER2_SUFFIX
        packet += c3 + self._COUNTER3_SUFFIX
        return bytes(packet)


def make_protocol(name: str) -> PacketProtocol:
    key = name.strip().lower().replace("-", "_")
    if key in {"wifi_8k", "wifi8k", "wifi_8k_prefixed_short", "wifi8k_prefixed_short", "prefixed_short", "raw9"}:
        return Wifi8kPrefixedShortProtocol()
    if key in {"wifi_uav", "wifi_uav_envelope", "uav"}:
        return WifiUavEnvelopeProtocol()
    if key in {"wifi_cam_extended", "cam_extended", "extended", "raw20"}:
        return WifiCamExtendedProtocol()
    if key in {"wifi_cam_short", "cam_short", "short", "raw8"}:
        return WifiCamShortProtocol()
    raise ValueError(f"Unknown protocol {name!r}")
