# ESP32 Drone Link

PlatformIO firmware for one ESP32-S3 acting as one drone radio link.

The PC sends framed USB-serial messages to the ESP32. The ESP32 joins exactly
one drone AP, opens a UDP socket, and forwards already-built drone control
packets to the configured drone IP and port.

Build and upload:

```bash
cd firmware/esp32_drone_link
pio run -t upload
```

The serial protocol uses `DL` framed binary messages with a version byte,
message type, sequence, payload length, and CRC-16/CCITT. The ESP32 reports
`BOOT`, accepts repeated config frames from the PC, connects to the configured
SSID, and reports `READY <local-ip> -> <drone-ip>:<port>` before control packets
are forwarded. This handles normal ESP32 USB reset/boot noise on connect.

Scan for drone APs from the ESP32 without changing the PC Wi-Fi:

```bash
python3 tools/esp_scan.py --port /dev/ttyACM0
```

Use from the repo:

```bash
python3 -m drone_control.single \
  --link-type esp_serial \
  --serial-port /dev/ttyACM0 \
  --ssid WIFI_8K-3e67bc \
  --ip 192.168.1.1 \
  --port 7099 \
  --command neutral
```

Use one ESP32 per drone AP. Multiple drones are handled by running multiple
bridge links in the Python swarm config, not by associating one ESP32 with
multiple drone APs.
