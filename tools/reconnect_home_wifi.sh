#!/usr/bin/env bash
set -euo pipefail

IFACE="${1:-wlP9s9}"
HOME_SSID="${2:-${HOME_SSID:-CircularEconomy}}"

nmcli dev wifi connect "$HOME_SSID" ifname "$IFACE" \
  || nmcli con up "$HOME_SSID" ifname "$IFACE"

ip -brief addr show "$IFACE"

