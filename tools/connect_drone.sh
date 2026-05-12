#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 2 ]]; then
  echo "usage: $0 IFACE SSID [DRONE_IP]" >&2
  echo "example: $0 wlan1 WIFI_8K-0c5b90 192.168.169.1" >&2
  exit 2
fi

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DRONE_ENV_FILE="${DRONE_ENV_FILE:-$ROOT_DIR/.drone.env}"
if [[ -f "$DRONE_ENV_FILE" ]]; then
  # shellcheck disable=SC1090
  source "$DRONE_ENV_FILE"
fi

IFACE="$1"
SSID="$2"
DRONE_IP="${3:-}"

disable_wifi_power_save() {
  if sudo -n iw dev "$IFACE" set power_save off 2>/dev/null; then
    return 0
  fi
  if [[ -n "${DRONE_SUDO_PASS:-}" ]] &&
    printf '%s\n' "$DRONE_SUDO_PASS" | sudo -S -p '' iw dev "$IFACE" set power_save off >/dev/null 2>&1; then
    return 0
  fi
  echo "warning: could not disable Wi-Fi power save without sudo password; continuing" >&2
}

disable_wifi_power_save

nmcli dev wifi connect "$SSID" ifname "$IFACE"

echo
echo "Interface address:"
ip -brief addr show "$IFACE"

echo
echo "Routes touching the interface:"
ip route show dev "$IFACE" || true

if [[ -n "$DRONE_IP" ]]; then
  echo
  echo "Ping check: $DRONE_IP via $IFACE"
  ping -c 2 -W 1 -I "$IFACE" "$DRONE_IP" || true
fi
