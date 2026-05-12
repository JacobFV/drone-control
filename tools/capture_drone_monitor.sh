#!/usr/bin/env bash
set -euo pipefail

IFACE="${1:-wlP9s9}"
CHANNEL="${2:-1}"
DURATION="${3:-30}"
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DRONE_ENV_FILE="${DRONE_ENV_FILE:-$ROOT_DIR/.drone.env}"
if [[ -f "$DRONE_ENV_FILE" ]]; then
  # shellcheck disable=SC1090
  source "$DRONE_ENV_FILE"
fi

HOME_SSID="${HOME_SSID:-CircularEconomy}"
MON_IFACE="${MON_IFACE:-mon-drone0}"
CAPTURE_DIR="${CAPTURE_DIR:-$ROOT_DIR/captures}"
mkdir -p "$CAPTURE_DIR"
PCAP_FILE="${PCAP_FILE:-$CAPTURE_DIR/drone_monitor_$(date +%Y%m%d_%H%M%S)_ch${CHANNEL}.pcap}"
PREV_CONNECTION="$(nmcli -t -f DEVICE,CONNECTION dev status | awk -F: -v dev="$IFACE" '$1 == dev {print $2; exit}')"
RECONNECT_TARGET="${PREV_CONNECTION:-$HOME_SSID}"

sudo_cmd() {
  if sudo -n true 2>/dev/null; then
    sudo "$@"
    return
  fi
  if [[ -n "${DRONE_SUDO_PASS:-}" ]]; then
    printf '%s\n' "$DRONE_SUDO_PASS" | sudo -S -p '' "$@"
    return
  fi
  sudo "$@"
}

cleanup() {
  local status=$?
  echo
  echo "Stopping monitor capture and restoring Wi-Fi..."
  sudo_cmd ip link set "$MON_IFACE" down >/dev/null 2>&1 || true
  sudo_cmd iw dev "$MON_IFACE" del >/dev/null 2>&1 || true
  nmcli dev wifi connect "$RECONNECT_TARGET" ifname "$IFACE" >/dev/null 2>&1 \
    || nmcli con up "$RECONNECT_TARGET" ifname "$IFACE" >/dev/null 2>&1 \
    || nmcli dev wifi connect "$HOME_SSID" ifname "$IFACE" >/dev/null 2>&1 \
    || true
  ip -brief addr show "$IFACE" || true
  echo "Capture file: $PCAP_FILE"
  exit "$status"
}
trap cleanup EXIT INT TERM

echo "Interface: $IFACE"
echo "Monitor: $MON_IFACE"
echo "Channel: $CHANNEL"
echo "Duration: ${DURATION}s"
echo "Output: $PCAP_FILE"
echo
echo "Use the phone app now: connect phone to the drone AP, open camera/control view, move sticks/calibrate."
echo "This laptop will disconnect from Wi-Fi during capture and reconnect afterward."
echo "Capture starts in 3 seconds."
sleep 3

sudo_cmd iw dev "$MON_IFACE" del >/dev/null 2>&1 || true
nmcli dev disconnect "$IFACE" >/dev/null 2>&1 || true
sudo_cmd ip link set "$IFACE" down
sudo_cmd iw dev "$IFACE" interface add "$MON_IFACE" type monitor
sudo_cmd ip link set "$MON_IFACE" up
sudo_cmd iw dev "$MON_IFACE" set channel "$CHANNEL"

if sudo -n true 2>/dev/null; then
  timeout "$DURATION" sudo tcpdump -i "$MON_IFACE" -s 0 -w "$PCAP_FILE"
elif [[ -n "${DRONE_SUDO_PASS:-}" ]]; then
  timeout "$DURATION" bash -c 'printf "%s\n" "$3" | sudo -S -p "" tcpdump -i "$1" -s 0 -w "$2"' _ "$MON_IFACE" "$PCAP_FILE" "$DRONE_SUDO_PASS"
else
  timeout "$DURATION" sudo tcpdump -i "$MON_IFACE" -s 0 -w "$PCAP_FILE"
fi
