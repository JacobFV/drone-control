#!/usr/bin/env bash
set -euo pipefail

PCAP="${1:?usage: $0 capture.pcap}"

if [[ -x "$(dirname "$0")/pcap_summary.py" ]]; then
  "$(dirname "$0")/pcap_summary.py" "$PCAP"
  echo
fi

echo "Packet summary:"
tcpdump -nn -r "$PCAP" 2>/dev/null | sed -n '1,120p'

echo
echo "UDP/TCP candidates:"
tcpdump -nn -r "$PCAP" 'ip and (udp or tcp)' 2>/dev/null |
  awk '
    {
      proto="?"
      for (i=1; i<=NF; i++) {
        if ($i == "UDP," || $i == "UDP") proto="UDP"
        if ($i == "Flags") proto="TCP"
      }
      print proto, $0
    }
  ' |
  sed -n '1,200p'
