from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any

from drone_control.discovery import AccessPoint
from drone_control.identity import drone_identity_id, normalize_token


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


@dataclass(slots=True)
class DroneFingerprint:
    ssid: str | None = None
    bssid: str | None = None
    channel: str | None = None
    signal: int | None = None
    control_endpoint: str | None = None
    control_protocol: str | None = None
    control_ack: str | None = None
    camera_rtsp_path: str | None = None
    camera_geometry: str | None = None
    camera_width: int | None = None
    camera_height: int | None = None
    camera_fps: float | None = None
    first_seen: str | None = None
    last_seen: str | None = None

    def resolved_camera_geometry(self) -> str | None:
        if self.camera_geometry:
            return self.camera_geometry
        if self.camera_width is None or self.camera_height is None:
            return None
        if self.camera_fps is None:
            return f"{self.camera_width}x{self.camera_height}"
        return f"{self.camera_width}x{self.camera_height}@{self.camera_fps:g}"

    def protocol_camera_fingerprint(self) -> str | None:
        parts = [
            self.control_protocol,
            self.control_endpoint,
            self.control_ack,
            self.camera_rtsp_path,
            self.resolved_camera_geometry(),
        ]
        fingerprint = "|".join(normalize_token(str(part)) for part in parts if _has_value(part))
        return fingerprint or None

    def identity_id(self) -> str:
        bssid = self.bssid if _has_value(self.bssid) else None
        fingerprint = None if bssid else self.protocol_camera_fingerprint()
        return drone_identity_id(self.ssid, bssid, fingerprint)

    def to_identity_dict(self) -> dict[str, Any]:
        return {
            "identity_id": self.identity_id(),
            "ssid": self.ssid,
            "bssid": self.bssid,
            "fingerprint": self.protocol_camera_fingerprint(),
            "first_seen": self.first_seen,
            "last_seen": self.last_seen,
        }

    def to_connection_dict(self) -> dict[str, Any]:
        return {
            "wifi": {
                "ssid": self.ssid,
                "bssid": self.bssid,
                "channel": self.channel,
                "signal": self.signal,
            },
            "control": {
                "endpoint": self.control_endpoint,
                "protocol": self.control_protocol,
                "ack": self.control_ack,
            },
            "camera": {
                "rtsp_path": self.camera_rtsp_path,
                "width": self.camera_width,
                "height": self.camera_height,
                "fps": self.camera_fps,
                "geometry": self.resolved_camera_geometry(),
            },
            "observed": {
                "first_seen": self.first_seen,
                "last_seen": self.last_seen,
            },
        }

    def to_dict(self) -> dict[str, Any]:
        return {
            "ssid": self.ssid,
            "bssid": self.bssid,
            "channel": self.channel,
            "signal": self.signal,
            "control_endpoint": self.control_endpoint,
            "control_protocol": self.control_protocol,
            "control_ack": self.control_ack,
            "camera_rtsp_path": self.camera_rtsp_path,
            "camera_geometry": self.resolved_camera_geometry(),
            "camera_width": self.camera_width,
            "camera_height": self.camera_height,
            "camera_fps": self.camera_fps,
            "first_seen": self.first_seen,
            "last_seen": self.last_seen,
            "identity_id": self.identity_id(),
            "fingerprint": self.protocol_camera_fingerprint(),
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> DroneFingerprint:
        if "wifi" in data or "control" in data or "camera" in data:
            return cls.from_connection_dict(data)
        return cls(
            ssid=_clean_str(data.get("ssid")),
            bssid=_clean_str(data.get("bssid")),
            channel=_clean_str(data.get("channel")),
            signal=_optional_int(data.get("signal")),
            control_endpoint=_clean_str(data.get("control_endpoint")),
            control_protocol=_clean_str(data.get("control_protocol")),
            control_ack=_clean_str(data.get("control_ack")),
            camera_rtsp_path=_clean_str(data.get("camera_rtsp_path")),
            camera_geometry=_clean_str(data.get("camera_geometry")),
            camera_width=_optional_int(data.get("camera_width")),
            camera_height=_optional_int(data.get("camera_height")),
            camera_fps=_optional_float(data.get("camera_fps")),
            first_seen=_clean_str(data.get("first_seen")),
            last_seen=_clean_str(data.get("last_seen")),
        )

    @classmethod
    def from_connection_dict(cls, data: dict[str, Any]) -> DroneFingerprint:
        wifi = _dict_value(data.get("wifi"))
        control = _dict_value(data.get("control"))
        camera = _dict_value(data.get("camera"))
        observed = _dict_value(data.get("observed"))
        return cls(
            ssid=_clean_str(wifi.get("ssid", data.get("ssid"))),
            bssid=_clean_str(wifi.get("bssid", data.get("bssid"))),
            channel=_clean_str(wifi.get("channel", data.get("channel"))),
            signal=_optional_int(wifi.get("signal", data.get("signal"))),
            control_endpoint=_clean_str(control.get("endpoint", data.get("control_endpoint"))),
            control_protocol=_clean_str(control.get("protocol", data.get("control_protocol"))),
            control_ack=_clean_str(control.get("ack", data.get("control_ack"))),
            camera_rtsp_path=_clean_str(camera.get("rtsp_path", data.get("camera_rtsp_path"))),
            camera_geometry=_clean_str(camera.get("geometry", data.get("camera_geometry"))),
            camera_width=_optional_int(camera.get("width", data.get("camera_width"))),
            camera_height=_optional_int(camera.get("height", data.get("camera_height"))),
            camera_fps=_optional_float(camera.get("fps", data.get("camera_fps"))),
            first_seen=_clean_str(observed.get("first_seen", data.get("first_seen"))),
            last_seen=_clean_str(observed.get("last_seen", data.get("last_seen"))),
        )


def access_point_fingerprint(ap: AccessPoint, *, observed_at: str | None = None) -> DroneFingerprint:
    seen = observed_at or utc_now_iso()
    return DroneFingerprint(
        ssid=ap.ssid or None,
        bssid=ap.bssid or None,
        channel=ap.channel or None,
        signal=ap.signal,
        first_seen=seen,
        last_seen=seen,
    )


def merge_fingerprints(existing: DroneFingerprint, observed: DroneFingerprint) -> DroneFingerprint:
    return DroneFingerprint(
        ssid=_prefer_stable(existing.ssid, observed.ssid),
        bssid=_prefer_stable(existing.bssid, observed.bssid),
        channel=_prefer_recent(existing.channel, observed.channel),
        signal=_prefer_recent(existing.signal, observed.signal),
        control_endpoint=_prefer_stable(existing.control_endpoint, observed.control_endpoint),
        control_protocol=_prefer_stable(existing.control_protocol, observed.control_protocol),
        control_ack=_prefer_stable(existing.control_ack, observed.control_ack),
        camera_rtsp_path=_prefer_stable(existing.camera_rtsp_path, observed.camera_rtsp_path),
        camera_geometry=_prefer_stable(existing.camera_geometry, observed.camera_geometry),
        camera_width=_prefer_stable(existing.camera_width, observed.camera_width),
        camera_height=_prefer_stable(existing.camera_height, observed.camera_height),
        camera_fps=_prefer_stable(existing.camera_fps, observed.camera_fps),
        first_seen=_earliest(existing.first_seen, observed.first_seen),
        last_seen=_latest(existing.last_seen, observed.last_seen),
    )


def resolve_identity_id(fingerprint: DroneFingerprint | dict[str, Any]) -> str:
    fp = fingerprint if isinstance(fingerprint, DroneFingerprint) else DroneFingerprint.from_dict(fingerprint)
    return fp.identity_id()


def fingerprint_from_store_dicts(
    identity_json: dict[str, Any] | None,
    connection_json: dict[str, Any] | None,
) -> DroneFingerprint:
    identity = identity_json or {}
    connection = connection_json or {}
    fingerprint = DroneFingerprint.from_connection_dict(connection)
    return merge_fingerprints(
        DroneFingerprint(
            ssid=_clean_str(identity.get("ssid")),
            bssid=_clean_str(identity.get("bssid")),
            first_seen=_clean_str(identity.get("first_seen")),
            last_seen=_clean_str(identity.get("last_seen")),
        ),
        fingerprint,
    )


def _prefer_stable(existing: Any, observed: Any) -> Any:
    return existing if _has_value(existing) else observed


def _prefer_recent(existing: Any, observed: Any) -> Any:
    return observed if _has_value(observed) else existing


def _has_value(value: Any) -> bool:
    return value is not None and value != ""


def _clean_str(value: Any) -> str | None:
    if value is None:
        return None
    text = str(value).strip()
    return text or None


def _optional_int(value: Any) -> int | None:
    if value is None or value == "":
        return None
    return int(value)


def _optional_float(value: Any) -> float | None:
    if value is None or value == "":
        return None
    return float(value)


def _dict_value(value: Any) -> dict[str, Any]:
    return value if isinstance(value, dict) else {}


def _earliest(left: str | None, right: str | None) -> str | None:
    if not left:
        return right
    if not right:
        return left
    return min(left, right)


def _latest(left: str | None, right: str | None) -> str | None:
    if not left:
        return right
    if not right:
        return left
    return max(left, right)


if __name__ == "__main__":
    ap = AccessPoint("WIFI_8K_123", "AA:BB:CC:DD:EE:FF", "6", "2437 MHz", 72, "--", True)
    provisional = access_point_fingerprint(ap, observed_at="2026-05-12T12:00:00+00:00")
    assert provisional.identity_id() == "drone-bssid-aa-bb-cc-dd-ee-ff-ssid-wifi-8k-123"
    enriched = DroneFingerprint(
        ssid="WIFI_8K_123",
        control_endpoint="192.168.0.1:8888",
        control_protocol="udp-e99",
        camera_rtsp_path="/live",
        camera_width=1280,
        camera_height=720,
        first_seen="2026-05-12T12:01:00+00:00",
        last_seen="2026-05-12T12:01:00+00:00",
    )
    merged = merge_fingerprints(provisional, enriched)
    assert merged.bssid == "AA:BB:CC:DD:EE:FF"
    assert merged.control_protocol == "udp-e99"
    assert merged.first_seen == "2026-05-12T12:00:00+00:00"
    assert DroneFingerprint.from_dict(merged.to_dict()).to_dict() == merged.to_dict()
