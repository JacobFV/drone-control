from __future__ import annotations

import re


def drone_identity_id(ssid: str | None, bssid: str | None, fingerprint: str | None = None) -> str:
    parts = ["drone"]
    if bssid:
        parts.append("bssid")
        parts.append(normalize_token(bssid))
    if ssid:
        parts.append("ssid")
        parts.append(normalize_token(ssid))
    if fingerprint:
        parts.append("fp")
        parts.append(normalize_token(fingerprint)[:24])
    return "-".join(parts)


def normalize_token(value: str) -> str:
    text = value.strip().lower()
    text = re.sub(r"[^a-z0-9]+", "-", text)
    return text.strip("-") or "unknown"
