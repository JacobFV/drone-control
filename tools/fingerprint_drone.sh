#!/usr/bin/env bash
set -euo pipefail

IFACE="${1:?usage: $0 IFACE IP [ports...]}"
IP="${2:?usage: $0 IFACE IP [ports...]}"
shift 2

PORTS=("$@")
if [[ ${#PORTS[@]} -eq 0 ]]; then
  PORTS=(21 23 53 80 81 88 443 554 8080 8090 8800 8801 8888 8899 1234 18881 50000)
fi

echo "Fingerprinting $IP via $IFACE"
echo

echo "ARP/neighbor:"
ip neigh show dev "$IFACE" | sed -n '1,40p' || true
echo

echo "TCP connect scan:"
for port in "${PORTS[@]}"; do
  if timeout 0.5 bash -c "cat < /dev/null > /dev/tcp/$IP/$port" 2>/dev/null; then
    echo "  open tcp/$port"
  else
    echo "  closed tcp/$port"
  fi
done
echo

echo "HTTP probes:"
for port in 80 81 88 8080 8090 8800 8801 18881; do
  echo "--- http://$IP:$port/"
  timeout 2 curl -fsS -m 1 "http://$IP:$port/" 2>&1 | sed -n '1,20p' || true
done
echo

echo "UDP listener sockets on local machine:"
ss -lunp | sed -n '1,120p' || true

