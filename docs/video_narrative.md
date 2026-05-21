# Video Narrative: From Toy Drone APs to Mixed-Link Autonomy

## Working Title

From $30 Wi-Fi Drones to a Modular Swarm Control Stack

## Thesis

The project started with a simple question: can cheap AP-mode camera drones be
controlled from our own software instead of the phone app? The answer became a
larger systems lesson. The hard part was not only packet format. The hard part
was the link layer: each drone creates its own Wi-Fi network, and commodity
laptops usually cannot be on several drone APs and the internet at the same
time. The solution was to separate high-level control from radio association by
moving each drone AP connection onto a small ESP32 bridge.

The resulting architecture is useful beyond toy drones. It is the same shape
needed for civilian field robotics: local perception, reliable low-latency
actuation, and a coordinator that can reason across multiple robots while the
robots keep their own radio links.

## Audience

Technical builders who understand Python, embedded boards, Wi-Fi, and robotics
at a practical level. The video should stay concrete: show scans, packets,
firmware uploads, config files, and short control tests. Keep the framing on
civilian field robotics: firefighting support, building inspection,
search-and-rescue training, environmental monitoring, and warehouse or
greenhouse inventory.

## Story Arc

### 1. Initial Objective

Open with the goal:

- Find inexpensive Wi-Fi camera drones.
- Understand how their phone app talks to them.
- Replace the phone app with a local control station.
- Eventually coordinate multiple drones from higher-level perception and
  language-driven control.

Key line:

> We were not trying to build a custom flight controller. We were trying to
> understand and own the link between a normal computer and AP-mode drones.

### 2. Finding the E99/WIFI_8K Drones

Show the physical drones and the Wi-Fi APs they expose:

```text
WIFI_8K-3e67bc
WIFI_8K-592b10
```

Explain that these drones behave like tiny Wi-Fi access points. The controller
device joins the drone network, then sends UDP control packets and RTSP/RTP
camera setup traffic.

What we learned:

- The drone AP is usually `192.168.1.1`.
- Stick control is UDP to port `7099`.
- The current verified control protocol is `wifi_8k_prefixed_short`.
- Neutral stick packet:

```text
03 66 80 80 80 80 00 00 99
```

- Keepalive:

```text
01 01
```

### 3. Protocol Discovery

Show packet captures and how the protocol was reconstructed:

- Capture phone app traffic while it controls the drone.
- Summarize UDP flows with `tools/pcap_summary.py`.
- Identify repeated control packets.
- Confirm checksum behavior.
- Rebuild camera frames from RTP/JPEG payloads.
- Discover RTSP negotiation on port `7070`.

This is the concrete reverse-engineering chapter. Keep it grounded in
observations:

- Stick axes are byte values.
- Flags carry takeoff, land, emergency, calibrate, and related commands.
- Camera ports are dynamic and negotiated over RTSP.
- The video stream is real JPEG data, not a synthetic app preview.

### 4. The First Implementation

Show the initial Python control path:

```text
PC Wi-Fi -> drone AP -> UDP control
```

Relevant commands:

```bash
python3 -m drone_control.single --iface wlP9s9 --ip 192.168.1.1 --port 7099 --command neutral
python3 -m drone_control.swarm --config config/drones.example.json --dry-run --seconds 0.3
```

Explain the first abstraction:

- `DroneAction` describes intent.
- `protocols.py` builds concrete drone packets.
- `UdpDroneLink` sends packets to the drone.
- `single.py` and `swarm.py` run repeated command loops.

This was enough for one drone, but not enough for robust remote operation or
multiple drones.

### 5. AP-Based Protocol Limitations

Explain the central networking constraint:

- Each drone is its own Wi-Fi AP.
- Joining a drone AP often disconnects the laptop from normal internet.
- Many drones reuse the same IP address, commonly `192.168.1.1`.
- One Wi-Fi radio generally cannot maintain arbitrary simultaneous AP
  associations.
- Multiple direct-PC drones require multiple Wi-Fi radios or namespace/router
  isolation.

This matters because remote control and model-driven robotics often need the PC
to keep internet, logs, UI access, or cloud model connectivity while also
talking to drones.

### 6. Trying Multi-Radio Communication

Show the experiments:

- NetworkManager scans and connects to drone APs.
- Virtual interface test with `tools/test_dual_wifi.sh`.
- Using separate Wi-Fi adapters as the clean direct-PC path.
- The practical limitation: single-radio cards are fragile for this workload,
  especially when internet and drone APs need to coexist.

Key line:

> The laptop Wi-Fi card is the wrong place to solve a multi-drone radio problem.

### 7. The ESP32 Bridge Idea

Introduce the shift:

```text
PC -> USB serial -> ESP32-S3 -> drone AP -> UDP control
```

One ESP32 owns one drone AP association. The PC stays on its normal network and
talks to each bridge over USB serial. Multiple drones become multiple USB
devices, not multiple fragile laptop Wi-Fi associations.

Why this is the useful abstraction:

- High-level code does not care whether the link is direct UDP or ESP serial.
- Each drone still gets its own independent radio link.
- The PC can mix links in one run:

```text
drone1: ESP32 serial bridge
drone2: ESP32 serial bridge
drone3: direct UDP Wi-Fi interface
```

### 8. Seeed/ESP32-S3 Affordance

The connected board appears as native USB serial/JTAG:

```text
/dev/ttyACM0
303a:1001 Espressif USB JTAG/serial debug unit
ESP32-S3
```

This was a useful affordance:

- No external USB-UART adapter required.
- The board can be flashed directly from PlatformIO.
- The same USB cable becomes the command link.
- The device can scan drone APs without touching the PC Wi-Fi.

Show the scan command:

```bash
python3 tools/esp_scan.py --port /dev/ttyACM0
```

Example output:

```text
SCAN WIFI_8K-3e67bc  -47  open
SCAN WIFI_8K-592b10  -52  open
SCAN_DONE 14
```

### 9. Bridge Firmware and Link Protocol

Explain the bridge protocol:

- Binary frames start with `DL`.
- Frames include version, message type, sequence, length, and CRC-16/CCITT.
- The PC resends config until the ESP reports `READY`.
- The parser tolerates ESP boot noise and USB reset behavior.
- The ESP joins one drone AP, opens UDP, and forwards packets.

Message types:

- `CONFIG`: SSID/password/drone IP/drone port.
- `SEND`: raw drone packet.
- `SCAN`: ESP-side Wi-Fi scan.
- `STATUS`: `BOOT`, `READY`, `SCAN ...`, `SCAN_DONE`.
- `ERROR`: connect timeout, UDP send failure, parse issues.

### 10. Success

Show the successful flow:

1. Flash ESP32-S3 bridge firmware.
2. Scan drone APs through ESP.
3. Choose `WIFI_8K-3e67bc`.
4. Send neutral control stream:

```text
esp-drone-3e67bc: sent=12 errors=0
```

5. Send a short takeoff/hold/land sequence through the ESP bridge:

```text
settle neutral
takeoff command
neutral hold
land command
motor stop
```

The important claim is not “perfect flight autonomy.” The important claim is:

> The link abstraction works end to end. The PC can control an AP-mode drone
> through an ESP32-S3 without moving the PC off its normal network.

### 11. Current Software Shape

Show the current stack:

```text
Electron UI
  -> Python service
    -> Manual safety state machine
    -> DroneAction
    -> PacketProtocol
    -> DroneLink
      -> UdpDroneLink
      -> EspSerialDroneLink
        -> ESP32-S3 firmware
          -> drone AP UDP
```

The code now has clear boundaries:

- Protocol code builds packets.
- Link code owns transport.
- Swarm code schedules multiple drone runtimes.
- Manual control owns safety policy.
- ESP firmware owns Wi-Fi association and UDP forwarding.

### 12. Future Direction: Three-Layer Real-Time Control

Frame this as a civilian robotics research direction.

Layer 1: Real-Time Scene and State Estimation

- Camera frames stream into a local estimator.
- IMU data, when available, stabilizes pose and motion estimates.
- Gaussian splatting or related scene representations provide a compact 3D
  map of the environment.
- Goal: give the controller a continuously updated spatial model, not just raw
  video.

Use cases:

- Firefighting support: map smoke-filled rooms from safe standoff positions.
- Search-and-rescue training: maintain a rough map while exploring.
- Infrastructure inspection: track camera poses and defects over a structure.

Layer 2: Single-Drone VLA Controller

- A vision-language-action model controls one drone at a time.
- Inputs: current camera, state estimate, recent actions, safety envelope, and
  a natural-language or structured objective.
- Outputs: bounded `DroneAction` targets or high-level primitives.
- The safety layer still clamps throttle, rate limits commands, and handles
  stop/fault behavior.

Example objective:

> Inspect the doorway, keep altitude low, and stop if tracking confidence drops.

Layer 3: Multi-Drone VLM Swarm Coordinator

- A vision-language model reasons across drones.
- It does not directly drive motors.
- It assigns roles, routes, priorities, and constraints.
- Each drone keeps a local VLA controller and link.
- The coordinator sees summaries: map fragments, confidence, battery, link
  health, and task progress.

Example civilian tasks:

- Fire response: one drone maps the entry path, one watches ceiling conditions,
  one tracks exit visibility.
- Search-and-rescue: divide rooms or trail segments among multiple small drones.
- Environmental monitoring: coordinate coverage while avoiding duplicate passes.

Final architecture:

```text
VLM swarm coordinator
  -> task allocation and constraints
VLA single-drone controllers
  -> bounded action targets
Real-time estimator + IMU + Gaussian splat map
  -> pose, scene, confidence
Link layer
  -> direct UDP or ESP32 serial bridge
Drones
  -> AP-mode UDP control and camera streams
```

### 13. Closing

Close with the technical lesson:

> The breakthrough was not one magic model or one packet. It was separating
> intent, packet protocol, and radio link. Once the link became swappable, the
> system stopped being a one-drone laptop trick and started becoming a modular
> robotics control stack.

## Suggested Video Structure

1. Hook: two toy drones, one laptop, one ESP32-S3.
2. Show the drone APs and explain AP-mode control.
3. Packet capture and protocol discovery.
4. First direct-PC implementation.
5. The multi-drone networking wall.
6. ESP32 bridge design.
7. Firmware upload and ESP-side scan.
8. Successful neutral and flight packet sequence.
9. Current code architecture.
10. Future three-layer autonomy stack.

## B-Roll / Screen Capture Checklist

- Drones powered on with visible AP scan output.
- `tools/pcap_summary.py` output showing control packets.
- `drone_control/protocols.py` packet builder.
- `drone_control/transport.py` `DroneLink` abstraction.
- PlatformIO upload to ESP32-S3.
- `python3 tools/esp_scan.py --port /dev/ttyACM0`.
- `python3 -m drone_control.swarm --config config/drones.local.json --command neutral --seconds 0.5`.
- Short controlled takeoff/land sequence in a clear area.
- Electron UI manual IO panel.
- Gaussian splat/pose UI surfaces for future-state discussion.

## Cover Image Prompt

Create a cinematic but technically grounded cover image for a robotics video.
Scene: a clean workbench with two small E99-style Wi-Fi camera drones, an
ESP32-S3 development board connected by USB-C to a laptop, and a subtle screen
showing a network diagram: "PC -> USB serial -> ESP32-S3 -> drone AP -> UDP".
In the background, show a ghosted 3D Gaussian-splat-like reconstruction of an
indoor room and small overlays for "Estimator + IMU", "VLA single-drone
controller", and "VLM swarm coordinator". The mood should be optimistic,
civilian, and technical, suggesting firefighting support, inspection, and
search-and-rescue robotics. Keep the scene calm, constructive, and field-ready,
with no adversarial or threatening imagery. Use realistic lighting, sharp
hardware detail, readable but minimal UI text, modern documentary thumbnail
composition, 16:9 aspect ratio.

## Framing Guardrails

Keep the language centered on safety, inspection, mapping, coordination,
low-cost research, and responsible field robotics. Do not frame the system as
harmful, coercive, or adversarial.

## Preferred Framing

- Firefighting support
- Search-and-rescue training
- Infrastructure inspection
- Environmental monitoring
- Greenhouse or warehouse inventory
- Low-cost robotics research
- Safety-bounded autonomy
