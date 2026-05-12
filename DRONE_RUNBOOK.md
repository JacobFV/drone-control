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

Reconnect manually if needed:

```bash
tools/reconnect_home_wifi.sh wlP9s9 CircularEconomy
```

## Dry Runs

```bash
python3 -m drone_control.single --iface wlP9s9 --dry-run --seconds 0.2
python3 -m drone_control.swarm --config config/drones.example.json --dry-run --seconds 0.3
```

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
