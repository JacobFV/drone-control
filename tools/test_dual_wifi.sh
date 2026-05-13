#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 2 || $# -gt 3 ]]; then
  echo "usage: $0 <phy> <drone-ssid> [virtual-iface]" >&2
  echo "example: $0 phy0 WIFI_8K-0c5b90 dronev0" >&2
  exit 2
fi

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_FILE="$ROOT_DIR/.drone.env"
if [[ -f "$ENV_FILE" ]]; then
  # shellcheck disable=SC1090
  source "$ENV_FILE"
fi

PHY="$1"
DRONE_SSID="$2"
VIF="${3:-dronev0}"
CON_NAME="${VIF}-${DRONE_SSID}"

sudo_run() {
  if sudo -n "$@" 2>/dev/null; then
    return 0
  fi
  if [[ -z "${DRONE_SUDO_PASS:-}" ]]; then
    echo "error: sudo is required and DRONE_SUDO_PASS is not set" >&2
    return 1
  fi
  printf '%s\n' "$DRONE_SUDO_PASS" | sudo -S -p '' "$@"
}

cleanup() {
  nmcli con down "$CON_NAME" >/dev/null 2>&1 || true
  nmcli con delete "$CON_NAME" >/dev/null 2>&1 || true
  nmcli dev disconnect "$VIF" >/dev/null 2>&1 || true
  sudo_run iw dev "$VIF" del >/dev/null 2>&1 || true
}
trap cleanup EXIT

cleanup

echo "Creating virtual managed interface $VIF on $PHY..."
sudo_run iw phy "$PHY" interface add "$VIF" type managed
sudo_run ip link set "$VIF" up
nmcli dev set "$VIF" managed yes >/dev/null 2>&1 || true

echo
echo "Interfaces:"
iw dev

echo
echo "Existing NetworkManager status:"
nmcli -t -f DEVICE,TYPE,STATE,CONNECTION dev status

echo
echo "Checking internet on existing wlP9s9..."
if ping -I wlP9s9 -c 1 -W 2 1.1.1.1 >/dev/null; then
  echo "wlP9s9 internet ping ok"
else
  echo "wlP9s9 internet ping failed"
fi

echo
echo "Scanning for $DRONE_SSID on $VIF..."
nmcli dev wifi rescan ifname "$VIF" >/dev/null 2>&1 || true
sleep 3
SCAN="$(nmcli -t -f SSID,BSSID,CHAN,FREQ,SIGNAL,SECURITY dev wifi list ifname "$VIF" --rescan no | awk -F: -v ssid="$DRONE_SSID" '$1==ssid {print; exit}')"
if [[ -z "$SCAN" ]]; then
  echo "drone SSID not visible on $VIF"
  exit 3
fi
echo "$SCAN"
BSSID_ESC="$(printf '%s\n' "$SCAN" | cut -d: -f2-7)"

echo
echo "Connecting $VIF to $DRONE_SSID without installing a default route..."
nmcli con add type wifi ifname "$VIF" con-name "$CON_NAME" ssid "$DRONE_SSID" >/dev/null
nmcli con mod "$CON_NAME" \
  802-11-wireless.bssid "$BSSID_ESC" \
  ipv4.method auto \
  ipv4.never-default yes \
  ipv4.route-metric 9000 \
  ipv6.method disabled \
  connection.autoconnect no
nmcli --wait 20 con up "$CON_NAME"

echo
echo "Addresses:"
ip -brief addr show wlP9s9
ip -brief addr show "$VIF"

echo
echo "Routes:"
ip route | sed -n '1,20p'

echo
echo "Connectivity:"
ping -I wlP9s9 -c 1 -W 2 1.1.1.1 >/dev/null && echo "wlP9s9 internet ping ok" || echo "wlP9s9 internet ping failed"
ping -I "$VIF" -c 1 -W 2 192.168.1.1 >/dev/null && echo "$VIF drone ping ok" || echo "$VIF drone ping failed"
