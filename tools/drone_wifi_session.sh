#!/usr/bin/env bash
set -euo pipefail

IFACE="${1:-wlP9s9}"
DRONE_SSID="${2:-}"

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DRONE_ENV_FILE="${DRONE_ENV_FILE:-$ROOT_DIR/.drone.env}"
if [[ -f "$DRONE_ENV_FILE" ]]; then
  # shellcheck disable=SC1090
  source "$DRONE_ENV_FILE"
fi

HOME_SSID="${HOME_SSID:-CircularEconomy}"
SECONDS_PER_PROBE="${SECONDS_PER_PROBE:-0.25}"

if [[ -z "$DRONE_SSID" ]]; then
  echo "usage: $0 IFACE DRONE_SSID" >&2
  echo "example: $0 wlP9s9 WIFI_8K-0c5b90" >&2
  echo >&2
  echo "Set HOME_SSID to override reconnect target. Current default: $HOME_SSID" >&2
  exit 2
fi

LOG_DIR="${LOG_DIR:-$ROOT_DIR/logs}"
mkdir -p "$LOG_DIR"
LOG_FILE="${LOG_FILE:-$LOG_DIR/drone_wifi_session_$(date +%Y%m%d_%H%M%S).log}"
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
echo "Previous connection: ${PREV_CONNECTION:-unknown}"
echo "Drone SSID: $DRONE_SSID"
echo "Reconnect target: $RECONNECT_TARGET"
echo

disable_wifi_power_save

echo "Connecting to drone AP..."
nmcli dev wifi connect "$DRONE_SSID" ifname "$IFACE"

echo
echo "Interface after drone connect:"
ip -brief addr show "$IFACE"

echo
echo "Routes on $IFACE:"
ip route show dev "$IFACE" || true

mapfile -t ROUTE_GATEWAYS < <(ip -4 route show dev "$IFACE" | awk '/default/ {print $3}')
mapfile -t SUBNET_GATEWAYS < <(
  ip -o -4 addr show dev "$IFACE" |
    awk '{print $4}' |
    awk -F'[./]' '$5 == 24 {print $1 "." $2 "." $3 ".1"}'
)
HARDCODED_CANDIDATES=(192.168.1.1 192.168.169.1 192.168.4.1 192.168.4.153 192.168.100.1)
CANDIDATE_IPS=()
add_candidate_ip() {
  local ip="$1"
  [[ -n "$ip" ]] || return 0
  local existing
  for existing in "${CANDIDATE_IPS[@]}"; do
    [[ "$existing" == "$ip" ]] && return 0
  done
  CANDIDATE_IPS+=("$ip")
}
for ip in "${ROUTE_GATEWAYS[@]}"; do add_candidate_ip "$ip"; done
for ip in "${SUBNET_GATEWAYS[@]}"; do add_candidate_ip "$ip"; done
for ip in "${HARDCODED_CANDIDATES[@]}"; do add_candidate_ip "$ip"; done

echo
echo "Candidate gateway pings:"
for ip in "${CANDIDATE_IPS[@]}"; do
  if ping -c 1 -W 1 -I "$IFACE" "$ip" >/dev/null 2>&1; then
    echo "  reachable: $ip"
  else
    echo "  no reply:   $ip"
  fi
done

echo
echo "Neighbor table after ping checks:"
ip neigh show dev "$IFACE" || true

echo
echo "Endpoint fingerprint:"
for ip in "${CANDIDATE_IPS[@]}"; do
  if ip neigh show dev "$IFACE" | grep -q "^$ip "; then
    "$ROOT_DIR/tools/fingerprint_drone.sh" "$IFACE" "$ip"
  fi
done

echo
echo "Neutral UDP probe sweep..."
PROBE_ARGS=("$ROOT_DIR/tools/probe_drone.py" --iface "$IFACE" --no-bind-device --seconds "$SECONDS_PER_PROBE")
for ip in "${CANDIDATE_IPS[@]}"; do
  PROBE_ARGS+=(--ip "$ip")
done
python3 "${PROBE_ARGS[@]}"

echo
echo "Stream/start handshake probe..."
STREAM_ARGS=("$ROOT_DIR/tools/probe_stream.py" --iface "$IFACE" --no-bind-device --seconds 1.0)
for ip in "${CANDIDATE_IPS[@]}"; do
  STREAM_ARGS+=(--ip "$ip")
done
python3 "${STREAM_ARGS[@]}"

echo
echo "Session complete."
