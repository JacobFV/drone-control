#!/usr/bin/env bash
set -euo pipefail

IFACE="${1:-wlP9s9}"
DRONE_SSID="${2:-}"
COMMAND="${3:-neutral}"
DURATION="${4:-5}"

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DRONE_ENV_FILE="${DRONE_ENV_FILE:-$ROOT_DIR/.drone.env}"
if [[ -f "$DRONE_ENV_FILE" ]]; then
  # shellcheck disable=SC1090
  source "$DRONE_ENV_FILE"
fi

HOME_SSID="${HOME_SSID:-CircularEconomy}"
DRONE_IP="${DRONE_IP:-192.168.1.1}"
DRONE_PORT="${DRONE_PORT:-7099}"
DRONE_PROTOCOL="${DRONE_PROTOCOL:-wifi_8k_prefixed_short}"
DRONE_HZ="${DRONE_HZ:-20}"
AXIS_TEST_AMPLITUDE="${AXIS_TEST_AMPLITUDE:-32}"
AXIS_TEST_PULSE_SECONDS="${AXIS_TEST_PULSE_SECONDS:-0.45}"
AXIS_TEST_NEUTRAL_SECONDS="${AXIS_TEST_NEUTRAL_SECONDS:-0.8}"
THROTTLE_SWEEP_VALUES="${THROTTLE_SWEEP_VALUES:-160,192,224,240,255}"
THROTTLE_SWEEP_STEP_SECONDS="${THROTTLE_SWEEP_STEP_SECONDS:-0.8}"
MIX_TEST_THROTTLE="${MIX_TEST_THROTTLE:-224}"
MANUAL_ROLL="${MANUAL_ROLL:-128}"
MANUAL_PITCH="${MANUAL_PITCH:-128}"
MANUAL_THROTTLE="${MANUAL_THROTTLE:-128}"
MANUAL_YAW="${MANUAL_YAW:-128}"
BIND_DEVICE="${BIND_DEVICE:-0}"

if [[ -z "$DRONE_SSID" ]]; then
  echo "usage: $0 IFACE DRONE_SSID [COMMAND] [SECONDS]" >&2
  echo "commands: probe, neutral, axis-test, throttle-sweep, mix-test, manual, calibrate, takeoff, land, stop" >&2
  echo "example: $0 wlP9s9 WIFI_8K-0c5b90 neutral 5" >&2
  echo >&2
  echo "Set HOME_SSID, DRONE_IP, DRONE_PORT, DRONE_PROTOCOL, DRONE_HZ, BIND_DEVICE, or DRONE_SUDO_PASS to override defaults." >&2
  exit 2
fi

LOG_DIR="${LOG_DIR:-$ROOT_DIR/logs}"
mkdir -p "$LOG_DIR"
LOG_FILE="${LOG_FILE:-$LOG_DIR/drone_control_session_$(date +%Y%m%d_%H%M%S).log}"
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
echo "Previous connection: ${PREV_CONNECTION:-unknown}"
echo "Drone SSID: $DRONE_SSID"
echo "Reconnect target: $RECONNECT_TARGET"
echo "Target: $DRONE_IP:$DRONE_PORT protocol=$DRONE_PROTOCOL command=$COMMAND seconds=$DURATION hz=$DRONE_HZ"
if [[ "$COMMAND" == "axis-test" || "$COMMAND" == "axis_test" ]]; then
  echo "Axis test: amplitude=$AXIS_TEST_AMPLITUDE pulse=${AXIS_TEST_PULSE_SECONDS}s neutral=${AXIS_TEST_NEUTRAL_SECONDS}s"
fi
if [[ "$COMMAND" == "throttle-sweep" || "$COMMAND" == "throttle_sweep" ]]; then
  echo "Throttle sweep: values=$THROTTLE_SWEEP_VALUES step=${THROTTLE_SWEEP_STEP_SECONDS}s neutral=${AXIS_TEST_NEUTRAL_SECONDS}s"
fi
if [[ "$COMMAND" == "mix-test" || "$COMMAND" == "mix_test" ]]; then
  echo "Mix test: base_throttle=$MIX_TEST_THROTTLE amplitude=$AXIS_TEST_AMPLITUDE pulse=${AXIS_TEST_PULSE_SECONDS}s neutral=${AXIS_TEST_NEUTRAL_SECONDS}s"
fi
if [[ "$COMMAND" == "manual" ]]; then
  echo "Manual action: roll=$MANUAL_ROLL pitch=$MANUAL_PITCH throttle=$MANUAL_THROTTLE yaw=$MANUAL_YAW"
fi
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

echo
echo "Ping check:"
ping -c 1 -W 1 -I "$IFACE" "$DRONE_IP" || true

echo
cd "$ROOT_DIR"
if [[ "$COMMAND" == "probe" ]]; then
  echo "Starting neutral probe with reply listener..."
  PROBE_ARGS=(
    tools/probe_drone.py
    --iface "$IFACE"
    --ip "$DRONE_IP"
    --port "$DRONE_PORT"
    --protocol "$DRONE_PROTOCOL"
    --seconds "$DURATION"
    --listen
  )
  if [[ "$BIND_DEVICE" != "1" ]]; then
    PROBE_ARGS+=(--no-bind-device)
  fi
  PYTHONUNBUFFERED=1 python3 "${PROBE_ARGS[@]}"
else
  echo "Starting command loop..."
  ARGS=(
    -m drone_control.single
    --iface "$IFACE"
    --ip "$DRONE_IP"
    --port "$DRONE_PORT"
    --protocol "$DRONE_PROTOCOL"
    --command "$COMMAND"
    --hz "$DRONE_HZ"
    --seconds "$DURATION"
    --test-amplitude "$AXIS_TEST_AMPLITUDE"
    --pulse-seconds "$AXIS_TEST_PULSE_SECONDS"
    --neutral-seconds "$AXIS_TEST_NEUTRAL_SECONDS"
    --ramp-values "$THROTTLE_SWEEP_VALUES"
    --ramp-step-seconds "$THROTTLE_SWEEP_STEP_SECONDS"
    --mix-throttle "$MIX_TEST_THROTTLE"
    --roll "$MANUAL_ROLL"
    --pitch "$MANUAL_PITCH"
    --throttle "$MANUAL_THROTTLE"
    --yaw "$MANUAL_YAW"
  )
  if [[ "$BIND_DEVICE" != "1" ]]; then
    ARGS+=(--no-bind-device)
  fi
  PYTHONUNBUFFERED=1 python3 "${ARGS[@]}"
fi

echo
echo "Control session complete."
