# drone-control

<p align="center">
  <strong>Attention:</strong> active follow-on work has moved to
  <a href="https://github.com/JacobFV/phys-0">JacobFV/phys-0</a>.
</p>

Local Electron/Python control-station experiments for `WIFI_8K-*` drone APs.
The control path supports mixed drone links:

- `udp`: direct PC Wi-Fi interface associated with one drone AP
- `esp_serial`: one ESP32 per drone AP, connected to the PC over USB serial

The swarm runner can use both link types in the same process, for example two
USB ESP32 bridges plus one direct PC Wi-Fi drone. See
[config/drones.example.json](config/drones.example.json) for a working config
shape and [DRONE_RUNBOOK.md](DRONE_RUNBOOK.md) for operating notes.

For a video-oriented project narrative, see
[docs/video_narrative.md](docs/video_narrative.md). It covers the initial
objective, E99/WIFI_8K protocol discovery, AP-mode networking limitations,
multi-radio experiments, the ESP32-S3 bridge, the successful end-to-end link,
and the future three-layer autonomy direction for civilian robotics use cases
such as firefighting support, inspection, and search-and-rescue training.

Build the ESP32 bridge firmware with PlatformIO:

```bash
cd firmware/esp32_drone_link
pio run
```

Scan drone APs through the ESP32 without touching the PC Wi-Fi:

```bash
python3 tools/esp_scan.py --port /dev/ttyACM0
```

Run local verification:

```bash
python3 -m unittest tools.test_transport
python3 -m unittest tools.test_service_manual_ack
python3 tools/test_smooth_camera_frames.py
npm run check
python3 -m drone_control.swarm --config config/drones.example.json --dry-run --seconds 0.2
```
