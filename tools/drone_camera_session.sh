#!/usr/bin/env bash
set -euo pipefail

IFACE="${1:-wlP9s9}"
DRONE_SSID="${2:-}"
DURATION="${3:-10}"

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DRONE_ENV_FILE="${DRONE_ENV_FILE:-$ROOT_DIR/.drone.env}"
if [[ -f "$DRONE_ENV_FILE" ]]; then
  # shellcheck disable=SC1090
  source "$DRONE_ENV_FILE"
fi

HOME_SSID="${HOME_SSID:-CircularEconomy}"
DRONE_IP="${DRONE_IP:-192.168.1.1}"
CAMERA_OUT_DIR="${CAMERA_OUT_DIR:-$ROOT_DIR/camera_captures}"
DRONE_CAMERA_PORT_SCAN="${DRONE_CAMERA_PORT_SCAN:-}"
BIND_DEVICE="${BIND_DEVICE:-0}"

if [[ -z "$DRONE_SSID" ]]; then
  echo "usage: $0 IFACE DRONE_SSID [SECONDS]" >&2
  echo "example: $0 wlP9s9 WIFI_8K-0c5b90 10" >&2
  exit 2
fi

LOG_DIR="${LOG_DIR:-$ROOT_DIR/logs}"
mkdir -p "$LOG_DIR"
LOG_FILE="${LOG_FILE:-$LOG_DIR/drone_camera_session_$(date +%Y%m%d_%H%M%S).log}"
exec > >(tee -a "$LOG_FILE") 2>&1

PREV_CONNECTION="$(nmcli -t -f DEVICE,CONNECTION dev status | awk -F: -v dev="$IFACE" '$1 == dev {print $2; exit}')"
RECONNECT_TARGET="${PREV_CONNECTION:-$HOME_SSID}"

reconnect_home() {
  local status=$?
  echo
  echo "Reconnecting $IFACE to ${RECONNECT_TARGET}..."
  nmcli dev wifi connect "$RECONNECT_TARGET" ifname "$IFACE" >/dev/null 2>&1 \
    || nmcli con up "$RECONNECT_TARGET" ifname "$IFACE" >/dev/null 2>&1 \
    || nmcli dev wifi connect "$HOME_SSID" ifname "$IFACE" >/dev/null 2>&1 \
    || true
  ip -brief addr show "$IFACE" || true
  echo "Log: $LOG_FILE"
  exit "$status"
}
trap reconnect_home EXIT INT TERM

disable_wifi_power_save() {
  if sudo -n iw dev "$IFACE" set power_save off 2>/dev/null; then
    return 0
  fi
  if [[ -n "${DRONE_SUDO_PASS:-}" ]] &&
    printf '%s\n' "$DRONE_SUDO_PASS" | sudo -S -p '' iw dev "$IFACE" set power_save off >/dev/null 2>&1; then
    return 0
  fi
  echo "warning: could not disable Wi-Fi power save without sudo password; continuing"
}

echo "Repo: $ROOT_DIR"
echo "Log: $LOG_FILE"
echo "Interface: $IFACE"
echo "Drone SSID: $DRONE_SSID"
echo "Reconnect target: $RECONNECT_TARGET"
echo "Drone IP: $DRONE_IP"
echo "Duration: ${DURATION}s"
echo

disable_wifi_power_save

echo "Connecting to drone AP..."
nmcli dev wifi connect "$DRONE_SSID" ifname "$IFACE"

echo
echo "Interface after drone connect:"
ip -brief addr show "$IFACE"

echo
echo "Starting camera capture..."
ARGS=(
  tools/camera_capture.py
  --iface "$IFACE"
  --drone-ip "$DRONE_IP"
  --seconds "$DURATION"
  --out-dir "$CAMERA_OUT_DIR"
)
if [[ -n "$DRONE_CAMERA_PORT_SCAN" ]]; then
  ARGS+=(--drone-port-scan "$DRONE_CAMERA_PORT_SCAN")
fi
if [[ "$BIND_DEVICE" != "1" ]]; then
  ARGS+=(--no-bind-device)
fi

cd "$ROOT_DIR"
PYTHONUNBUFFERED=1 python3 "${ARGS[@]}"

echo
echo "Camera session complete."
