# Drone Control Bring-Up

The powered drones currently appear as open APs:

- `WIFI_8K-0c5b90`
- `WIFI_8K-592b10`

The laptop currently has one Wi-Fi interface, `wlP9s9`, so live testing one
drone will temporarily disconnect from the normal Wi-Fi network. Use the session
script so it reconnects automatically.

Local secrets and machine-specific overrides live in `.drone.env`, which is
gitignored. The Wi-Fi scripts source it automatically on startup.

Observed from `WIFI_8K-0c5b90`: DHCP assigned the laptop `192.168.1.100/24`
with default gateway `192.168.1.1`, so this unit's likely drone IP is
`192.168.1.1`.

## Safe Scan

```bash
python3 tools/scan_drones.py --iface wlP9s9
```

The Electron app also supports platform-aware Wi-Fi scanning from the
Connection panel. On this macOS machine the built-in Wi-Fi interface is expected
to be `en0`; on Windows the app uses the first `netsh wlan` interface, usually
named `Wi-Fi`. Linux still uses NetworkManager interface names such as
`wlP9s9`.

## One-Drone Network Session

This connects to the drone, checks likely gateway IPs, sends neutral UDP probe
packets, logs the run, and reconnects to the previous/home Wi-Fi on exit.

```bash
tools/drone_wifi_session.sh wlP9s9 WIFI_8K-0c5b90
```

Current result: `WIFI_8K-0c5b90` uses `192.168.1.1` as its gateway. The phone
app capture found the real stick-control endpoint at `192.168.1.1:7099`.

Verified control packet:

```text
03 66 ROLL PITCH THROTTLE YAW FLAGS CHECKSUM 99
```

The checksum is XOR over `ROLL PITCH THROTTLE YAW FLAGS`. Neutral is:

```text
03 66 80 80 80 80 00 00 99
```

The phone also sends a low-rate keepalive to the same UDP port:

```text
01 01
```

The implemented protocol name is `wifi_8k_prefixed_short`. Stick axes and
checksum are verified from flight traffic. Function-button flags such as
takeoff, land, calibrate, flip, headless, and emergency are still inferred from
related WiFi-CAM protocols until captured explicitly.

Live throttle test result: motors did not require a separate observed unlock
packet. They started once throttle was swept high enough, with lift increasing
at higher throttle bytes. Treat `throttle=0, flags=0` as the proven motor-stop
packet; `throttle=128` is neutral stick center, not a guaranteed motor-off.

Later throttle tests became inconsistent: `throttle=255` and
`throttle-sweep` no longer lifted the drone, including after trying four fresh
batteries. Treat lift/no-lift as a physical-state variable for now, not as proof
that the UDP control mapping changed.

Interactive calibration result: with the current drone state, motors began
spinning when throttle crossed upward from about `168` to `176`, and stopped
when crossed downward from about `160` to `152`. Jumping straight to `0` was not
as reliable as ramping down through the stop threshold.

## Phone App Capture

Start this on the laptop, then use the phone app against the drone while it
captures. The script restores normal Wi-Fi afterward.

```bash
tools/capture_drone_monitor.sh wlP9s9 1 30
```

Analyze the capture:

```bash
tools/analyze_pcap.sh captures/<capture-file>.pcap
```

For a focused UDP/control summary:

```bash
python3 tools/pcap_summary.py captures/<capture-file>.pcap
```

## Camera Capture

The phone-app captures show camera data as RTP/JPEG UDP streams from the drone
to the phone. The payload is JPEG type `65` at `640x384`, with RTP/JPEG headers,
restart markers, dynamic quantization tables, frame ids, and offsets.

Observed stream pairs:

- `drone_monitor_20260512_110655_ch1.pcap`: `192.168.1.1:53796 -> 192.168.1.101:32124`, aux `32125 -> 53797`
- `drone_monitor_20260512_140736_ch1.pcap`: `192.168.1.1:52042 -> 192.168.1.101:31364`, aux `31365 -> 52043`
- `drone_monitor_20260512_140736_ch1.pcap`: `192.168.1.1:53214 -> 192.168.1.101:19402`, aux `19403 -> 53215`
- `drone_monitor_20260512_141413_ch1.pcap`: `192.168.1.1:52612 -> 192.168.1.101:12186`, aux `12187 -> 52613`

The local and drone-side ports are dynamic, but the aux port is consistently
the corresponding video port plus one on both sides.

The dynamic ports are negotiated over RTSP on TCP port `7070`:

```text
OPTIONS  rtsp://192.168.1.1:7070/webcam
DESCRIBE rtsp://192.168.1.1:7070/webcam
SETUP    rtsp://192.168.1.1:7070/webcam/track0
         Transport: RTP/AVP/UDP;unicast;client_port=<video>-<aux>
PLAY     rtsp://192.168.1.1:7070/webcam/
```

The drone responds to `SETUP` with `server_port=<video>-<aux>` and a session
id. The app then sends the small UDP primer packets and `PLAY`; the drone starts
RTP/JPEG-like UDP video shortly after.

For a camera-focused summary:

```bash
python3 tools/pcap_summary.py captures/drone_monitor_20260512_141413_ch1.pcap --limit 10
```

For offline JPEG extraction from a monitor pcap:

```bash
tools/camera_pcap_extract.py captures/drone_monitor_20260512_141413_ch1.pcap \
  --out-dir camera_captures/pcap_20260512_141413
```

This writes ordinary `.jpg` files by rebuilding the omitted RTP/JPEG container
headers: SOI, quantization tables, SOF0, Huffman tables, restart interval, SOS,
and EOI.

First-pass capture:

```bash
tools/drone_camera_session.sh wlP9s9 WIFI_8K-0c5b90 10
```

This connects to the drone AP, sends the observed stream-start and aux-video
probes, performs RTSP `OPTIONS`/`DESCRIBE`/`SETUP`/`PLAY`, listens on UDP port
`32124`, saves raw UDP payloads, and writes per-frame JPEGs under
`camera_captures/`.

The startup capture shows this ordering for `drone_monitor_20260512_141413_ch1.pcap`:

- `4.478s`: phone sends `80 00 00 00 00 00 00 00 00 00 00 00` from local video port `12186` to drone video port `52612`
- `4.478s`: phone sends `80 c9 00 01 00 00 00 00` from local aux port `12187` to drone aux port `52613`
- `4.520s`: phone begins `ef 00 04 00` probes to `192.168.169.1:8800`
- `4.565s`: drone starts video from `52612` to `12186`

Capture `drone_monitor_20260512_145202_ch1.pcap` exposed the RTSP negotiation:
the app requested `client_port=33012-33013`, the drone replied
`server_port=53796-53797`, and video began immediately after `PLAY`.

Autonomous live camera startup and JPEG output are now confirmed working:

```text
RTSP server ports: video=53796 aux=53797
Captured packets=1936 bytes=2490879 frames=170
Captured packets=1635 bytes=2235749 frames=126
frame_00018_0528a40f.jpg: JPEG image data, baseline, precision 8, 640x384, components 3
```

The remaining camera work is convenience display/encoding, such as writing a
preview loop, MJPEG file, or encoded video from the JPEG frame sequence.

For a smoothed frame sequence:

```bash
tools/smooth_camera_frames.py camera_captures/pcap_20260512_145202_jpeg_test \
  --out-dir camera_captures/pcap_20260512_145202_smooth_test \
  --compare-index 13
```

This adds a one-frame-latency temporal layer over the decoded JPEGs. It replaces
pixels that are strong outliers versus both adjacent frames when the adjacent
frames agree, then applies a small exponential moving average. The output
directory contains smoothed `.jpg` frames, `metrics.json`, `metrics.txt`, and a
raw/smoothed comparison sheet.

On `drone_monitor_20260512_145202_ch1.pcap`, this first-pass smoother produced:

```text
frames=326
temporal_mae: raw=10.922 smooth=7.463 delta=-31.7%
speckle_mae:  raw=0.746 smooth=0.708 delta=-5.0%
raw_to_smooth_mae=2.462
replaced_pixel_pct=1.394%
```

Useful metrics:

- `temporal_mae`: average frame-to-frame pixel change; lower means less jitter/flicker.
- `speckle_mae`: average difference from a 3x3 median-filtered version of the same frame; lower means less isolated high-frequency noise.
- `raw_to_smooth_mae`: how much the smoother changed the image; low values mean the filter is conservative.
- `replaced_pixel_pct`: percentage of pixels replaced by the temporal outlier filter.

Optional experimental live scan, no longer needed for normal capture:

```bash
DRONE_CAMERA_PORT_SCAN=52000:54000:2 \
  tools/drone_camera_session.sh wlP9s9 WIFI_8K-0c5b90 10
```

On 2026-05-12, before RTSP was implemented, a narrow scan over `52600:52624:2`
and a broader scan over `52000:54000:2` still produced `Captured packets=0`.

Capture `drone_monitor_20260512_140736_ch1.pcap` showed two simultaneous video
streams already active from the first milliseconds of capture:

- `192.168.1.1:52042 -> 192.168.1.101:31364`
- `192.168.1.1:53214 -> 192.168.1.101:19402`

Both ran for the full 30 seconds at about 14 MB each. Photo/record activity did
not produce an obvious separate command flow; it appears the app may save from
the already-running stream locally. The local video ports were dynamic, not the
older capture's `32124`, so Python capture must reproduce the app's startup
negotiation rather than hardcode one receive port.

Reconnect manually if needed:

```bash
tools/reconnect_home_wifi.sh wlP9s9 CircularEconomy
```

## Dry Runs

```bash
python3 -m drone_control.single --iface wlP9s9 --dry-run --seconds 0.2
python3 -m drone_control.swarm --config config/drones.example.json --dry-run --seconds 0.3
```

## Control Station App

Run the Electron app from the repository root:

```bash
npm start
```

Electron starts the Python service automatically and prefers `.venv/bin/python`
when it exists. Rebuild reconstruction dependencies with:

```bash
SUDO_PASSWORD='<sudo-password>' JOBS=4 tools/setup_reconstruction_deps.sh
```

The Scene panel in the right sidebar owns Gaussian splat reconstruction. It is
separate from the black trajectory simulation view.

Scene controls:

- `MAX IMG`: maximum sampled frames for the reconstruction dataset.
- `STEPS`: `splatfacto` training iterations.
- `BUILD SPLAT`: start dataset preparation, training, export, and record import.
- `STOP`: terminate the active reconstruction process.
- `VIEW`: open the latest `gaussian-splat` record in the external `gsplat.js`
  viewer.

The Records panel also shows a `VIEW` button on individual `gaussian-splat`
records.

For a quick validation run, use a low `STEPS` value such as `20` or `100`. For a
quality run, increase `STEPS` substantially; Nerfstudio's default `splatfacto`
training length is `30000`.

## Live Single-Drone Loop

Only run after confirming the right IP, port, and protocol from the probe:

```bash
tools/drone_control_session.sh wlP9s9 WIFI_8K-0c5b90 probe 3
tools/drone_control_session.sh wlP9s9 WIFI_8K-0c5b90 neutral 5
tools/drone_control_session.sh wlP9s9 WIFI_8K-0c5b90 axis-test
tools/drone_control_session.sh wlP9s9 WIFI_8K-0c5b90 throttle-sweep
tools/drone_control_session.sh wlP9s9 WIFI_8K-0c5b90 mix-test
tools/drone_control_session.sh wlP9s9 WIFI_8K-0c5b90 interactive
```

That wrapper handles the Wi-Fi cutover, runs the command locally while the
laptop is on the drone AP, logs to `logs/`, and reconnects to the previous/home
Wi-Fi afterward.

Use `probe` first. It sends neutral packets plus keepalive and listens for the
5-byte ACKs seen in the capture, without sending takeoff or land.

Use `axis-test` only with the drone restrained in a clear area. It sends a short
throttle pulse, then pitch/yaw/roll pulses in both directions with neutral gaps.
Tune it with `AXIS_TEST_AMPLITUDE`, `AXIS_TEST_PULSE_SECONDS`, and
`AXIS_TEST_NEUTRAL_SECONDS`.

`throttle-sweep` is the primary restrained motor test. It sends throttle values
from `THROTTLE_SWEEP_VALUES` continuously, then ends with `throttle=0`. Current
default values are `160,192,224,240,255`; lift was observed only once the sweep
reached the higher part of that range.

`mix-test` holds a base throttle and pulses pitch/yaw/roll around it so motor
mixing can be observed while the props are already spinning. The default
`MIX_TEST_THROTTLE` is `224` because `180` and `192` were below hover in live
tests. Tune it with `MIX_TEST_THROTTLE`, `AXIS_TEST_AMPLITUDE`,
`AXIS_TEST_PULSE_SECONDS`, and `AXIS_TEST_NEUTRAL_SECONDS`.

For one explicit packet stream, use `manual` with `MANUAL_ROLL`,
`MANUAL_PITCH`, `MANUAL_THROTTLE`, and `MANUAL_YAW`.

`interactive` opens a live keyboard loop:

- Up/Down adjust throttle.
- Left/Right adjust yaw.
- `W`/`S` adjust pitch.
- `A`/`D` adjust roll.
- Space toggles between ramp-stop and resume. If throttle is nonzero, it stores
  the current throttle, ramps down through the observed stop threshold, then to
  `0`. If throttle is already `0`, it ramps back to the stored throttle.
- `Z` sends direct `throttle=0`.
- `C` centers roll/pitch/yaw.
- `N` returns all sticks to neutral.
- `+`/`-` adjust the step size.
- Esc, Enter, or `Q` exits and sends final `throttle=0` packets.

Tune it with `INTERACTIVE_STEP`, `INTERACTIVE_START_THROTTLE`, and
`INTERACTIVE_RESUME_THROTTLE`.

Direct command if already connected to the drone AP:

```bash
python3 -m drone_control.single \
  --iface wlP9s9 \
  --ip 192.168.1.1 \
  --port 7099 \
  --protocol wifi_8k_prefixed_short \
  --no-bind-device \
  --command neutral
```

The script sends the observed `01 01` keepalive automatically for protocols
that provide one.

## Multi-Drone Requirement

Simultaneous two-drone control needs two Wi-Fi interfaces. Eight drones need
eight interfaces or a namespace/router setup that still gives each drone its own
radio link.

On a single-radio macOS or Windows laptop, treat the app as one active drone
connection at a time. Use Ethernet, USB tethering, or a second Wi-Fi adapter if
you need internet while connected to the drone AP.
