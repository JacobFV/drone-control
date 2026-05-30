# Control Station Architecture

This architecture supports the story in [video_narrative.md](video_narrative.md):
start with AP-mode E99/WIFI_8K drones, learn their UDP/RTSP protocol, hit the
single-radio laptop limitation, then move drone AP association into one ESP32-S3
bridge per drone while keeping high-level control code link-agnostic. The
longer-term direction is civilian, safety-bounded robotics: real-time
scene/state estimation, a single-drone VLA controller, and a VLM coordinator for
multi-drone tasks such as firefighting support, infrastructure inspection, and
search-and-rescue training.

## Storage

The control station stores operational metadata in `data/control_station.sqlite3`.
Heavy records are stored outside SQLite under `data/blobs/` and referenced by
content-ish blob keys from `records.blob_key`.

SQLite owns:

- drones and identity fingerprints
- flights
- policies
- metadata
- metrics
- record indexes

The blob store owns:

- frame directories
- raw UDP captures
- pcaps
- logs
- pose tracks
- reconstruction datasets
- Gaussian splat export directories
- future encoded video files

The `data/` directory is intentionally gitignored.

## Electron/Python Boundary

Electron owns the native window and renderer UI. Python owns drone IO, the
database, blob storage, camera record serving, flight recording, Wi-Fi capability
checks, and manual control.

Electron starts `python3 -m drone_control.service` and communicates with it over
localhost HTTP through main-process IPC. The renderer does not spawn shell
commands and does not talk to drones directly.

Current endpoints:

- `GET /api/health`
- `GET /api/state`
- `POST /api/flights`
- `PATCH /api/flights/<flight-id>`
- `POST /api/flights/<flight-id>/records`
- `GET /api/flights/<flight-id>/session`
- `POST /api/flights/<flight-id>/session/start`
- `POST /api/flights/<flight-id>/session/stop`
- `GET /api/flights/<flight-id>/pose/status`
- `GET /api/flights/<flight-id>/pose/track`
- `POST /api/flights/<flight-id>/pose/compute`
- `GET /api/flights/<flight-id>/reconstruction/status`
- `POST /api/flights/<flight-id>/reconstruction/start`
- `POST /api/flights/<flight-id>/reconstruction/stop`
- `GET /api/records/<record-id>/mjpeg`
- `GET /api/records/<record-id>/artifact`
- `GET /api/records/<record-id>/splat-viewer`
- `POST /api/records/<record-id>/export`
- `POST /api/records/<record-id>/reveal`
- `GET /api/wifi/capabilities`
- `GET /api/wifi/interfaces`
- `GET /api/wifi/access-points`
- `POST /api/wifi/connect`
- `POST /api/wifi/reconnect`
- `GET /api/manual/status`
- `POST /api/manual/arm`
- `POST /api/manual/disarm`
- `POST /api/manual/heartbeat`
- `POST /api/manual/axes`
- `POST /api/manual/stop`
- `POST /api/manual/clear-fault`
- `GET /api/runtime/status`
- `GET /api/runtime/events`
- `POST /api/runtime/start`
- `POST /api/runtime/stop`
- `POST /api/runtime/drones/<drone-id>/controller`
- `POST /api/runtime/drones/<drone-id>/arm`
- `POST /api/runtime/drones/<drone-id>/disarm`
- `POST /api/runtime/drones/<drone-id>/heartbeat`
- `POST /api/runtime/drones/<drone-id>/axes`
- `POST /api/runtime/drones/<drone-id>/stop`
- `POST /api/runtime/drones/<drone-id>/clear-fault`
- `GET /api/mission/progress`
- `POST /api/mission/start`
- `POST /api/mission/stop`

## Live Video Path

The current live/review path is MJPEG over localhost. The Python service streams
JPEG frame records as `multipart/x-mixed-replace`, and the Electron renderer
uses a normal `<img>` element as the sink.

The service also owns the flight session recorder. A session starts a concrete
frame source, writes only real decoded JPEG frames to `data/session_work/`, and
imports that directory into the blob store as a `frames` record when stopped.
There is no synthetic frame fallback. A `directory` source exists for repeatable
tests and review-file import. The app-facing Start Capture action uses the
`live` source, which opens the drone RTSP/RTP camera path.

## Pose Track Path

The pose track is the bridge between captured camera frames and any 3D scene
workflow. It is stored as a `pose-track` record in the blob store and is served
through the flight pose endpoints.

For live sessions, the session recorder can expose a replayable frame source and
pose status. For stored frame records, `POST /api/flights/<flight-id>/pose/compute`
uses the local visual odometry estimator in `drone_control/pose_estimator.py`.
The estimator depends on OpenCV and writes a JSONL pose track. Each pose includes
the source frame index, translation, quaternion rotation, and quality metadata.

The renderer treats a flight with frames but no stored pose track as
`not_computed` when OpenCV is available. In that state it can automatically run
the pose computation. The previous `NO ESTIMATOR` state is now reserved for the
case where required estimator dependencies are unavailable or no usable frame
record exists.

## Gaussian Splat Reconstruction

Gaussian splatting is implemented as an asynchronous backend job in
`drone_control/reconstruction.py`. The UI surface is the right-sidebar Scene
panel, not the black trajectory simulation view.

The reconstruction flow is:

1. Select a flight and a `frames` record.
2. Optionally compute or select the latest `pose-track` record.
3. `POST /api/flights/<flight-id>/reconstruction/start` starts a background
   `ReconstructionJob`.
4. The job copies a sampled image set into a Nerfstudio dataset directory.
5. If a pose track exists, the job writes `transforms.json` and trains directly
   from that frames-plus-poses dataset.
6. If no pose track exists, the job runs `ns-process-data images`, which invokes
   COLMAP to estimate camera poses.
7. The job runs `ns-train splatfacto` with bounded `maxIterations`.
8. The job runs `ns-export gaussian-splat`.
9. The exported splat directory is imported as a `gaussian-splat` record.

The Scene panel exposes:

- `MAX IMG`: sampled image count for the dataset.
- `STEPS`: `splatfacto` training iterations.
- `BUILD SPLAT`: starts reconstruction.
- `STOP`: terminates the active reconstruction process.
- `VIEW`: opens the latest `gaussian-splat` record in an external viewer.

Records of type `gaussian-splat` also get an inline `VIEW` button in the Records
panel. The view opens `GET /api/records/<record-id>/splat-viewer`, an HTML page
that loads `gsplat.js` from a CDN and fetches the local artifact through
`GET /api/records/<record-id>/artifact`.

This architecture deliberately separates the two 3D views:

- The simulation view displays trajectory, pose, and camera path state.
- The Gaussian splat viewer displays the reconstructed scene artifact.

## Reconstruction Dependencies

Runtime Python dependencies live in the single top-level `requirements.txt`.
There is no separate 3D requirements file.

The repeatable setup entry point is:

```bash
tools/setup_reconstruction_deps.sh
```

That script creates or updates `.venv`, installs apt packages, installs Python
requirements, verifies Nerfstudio command-line tools, and builds Open3D from
source on this ARM/aarch64 machine when a compatible wheel is not available.

The dependency roles are:

- `nerfstudio`: provides the `splatfacto` training pipeline and the
  `ns-process-data`, `ns-train`, and `ns-export` commands.
- `gsplat`: the training-side Gaussian splatting implementation pulled by
  Nerfstudio.
- `pymeshlab`: imported by Nerfstudio's exporter path.
- `open3d`: native geometry/point-cloud dependency used by Nerfstudio and its
  supporting tooling.
- `colmap`: used only when the reconstruction job must estimate poses from
  images because no pose track exists.

On this machine, Open3D is built under `vendor/Open3D` with build output under
`vendor/Open3D-build`; both are gitignored. The script also patches Open3D's
runtime dependency on `libidn2.so.0` after the source-built wheel is installed,
because the bundled static curl build leaves `idn2_*` symbols unresolved on this
platform.

Nerfstudio checkpoints are generated locally by the training subprocess.
PyTorch 2.6+ defaults `torch.load()` to `weights_only=True`, which breaks
Nerfstudio's exporter for those trusted local checkpoints. Reconstruction
subprocesses set `TORCH_FORCE_NO_WEIGHTS_ONLY_LOAD=1` so `ns-export` can load the
checkpoint it just produced.

## Drone Identity

Drone identity should be resolved in this order:

1. BSSID/MAC when available.
2. Normalized SSID.
3. Control fingerprint, including `192.168.1.1:7099` ACK payload
   `48 02 00 00 00`.
4. Camera fingerprint, including RTSP path `/webcam`, JPEG payload type, and
   frame geometry.

SSID alone is acceptable only as a provisional identity. If two drones share an
SSID pattern, BSSID and protocol fingerprints must disambiguate them before
records are merged.

## Manual Control Safety

Manual control must remain disabled until a flight is explicitly armed. The
control path should enforce these rules in Python, not just in the UI:

- No command is sent while disarmed.
- Stop command is always accepted.
- Any lost UI heartbeat ramps throttle to zero.
- Any missing local transport acceptance faults manual output to motor stop.
- Throttle commands are rate-limited.
- Maximum throttle is capped by a per-flight policy until deliberately raised.
- Switching away from manual mode sends a ramped stop.
- Closing the app sends a ramped stop when a manual session is active.

The Electron control pad sends desired axes to the local service. The service
holds the safety state machine and a 20 Hz command loop. Packet output is
disabled by default; setting `DRONE_SERVICE_ENABLE_IO=1` enables packet
emission through the configured link. Direct UDP uses `DRONE_LINK_TYPE=udp`,
`DRONE_IFACE`, `DRONE_IP`, `DRONE_PORT`, and `DRONE_PROTOCOL`. ESP32 bridge
manual IO uses `DRONE_LINK_TYPE=esp_serial`, `DRONE_ESP_SERIAL_PORT`,
`DRONE_ESP_SERIAL_BAUD`, `DRONE_SSID`, optional `DRONE_WIFI_PASSWORD`,
`DRONE_IP`, `DRONE_PORT`, and `DRONE_PROTOCOL`. Opening the app without packet
output enabled cannot send motor packets.

## Wi-Fi Concurrency

The control station now treats network operations as platform-specific service
work. The Python service exposes one app contract for Linux, macOS, and Windows:

- Linux uses `nmcli` for interfaces, scans, connect, and reconnect.
- macOS uses `networksetup` for interface/current-network/connect operations
  and the private `airport` utility for scans when it is available.
- Windows uses `netsh wlan`; open drone APs can be connected through a generated
  temporary WLAN profile, while secured networks require an existing profile or
  password-supported profile setup.

The app defaults to the detected Wi-Fi interface (`en0` on macOS, the first
`netsh` WLAN interface on Windows, or the first `nmcli` Wi-Fi device on Linux)
instead of assuming the original Linux `wlP9s9` interface.

The laptop radio currently advertises:

```text
#{ managed, P2P-client } <= 2 ... #channels <= 2
```

This means simultaneous internet Wi-Fi and drone Wi-Fi is plausible on one radio
using a virtual managed interface, but it is not yet proven stable through
NetworkManager. The repeatable experiment is:

```bash
tools/test_dual_wifi.sh phy0 WIFI_8K-0c5b90 dronev0
```

The low-risk virtual-interface test succeeded on 2026-05-12: `dronev0` could be
created while `wlP9s9` stayed connected to `CircularEconomy`, and the virtual
interface saw `WIFI_8K-0c5b90` during scan. A full association attempt was not
confirmed because the drone SSID was not visible on retry.

For multiple simultaneous drones, each drone still needs an independent radio
association to its AP. The control process now abstracts that link, so a swarm
can mix direct PC UDP links and ESP32 USB-serial bridge links. A direct UDP link
uses a PC Wi-Fi interface bound to the drone AP. An ESP32 bridge link uses one
ESP32 per drone; the ESP32 joins that drone AP and forwards PC-built control
packets from USB serial to UDP.

Single-radio multi-interface support can be used opportunistically after a full
connect/ping test passes, but one ESP32 per drone is the cleaner way to add more
drone AP links without adding PC Wi-Fi adapters.

The service exposes Wi-Fi discovery and explicit connect/reconnect endpoints.
`POST /api/wifi/connect` requires `confirmDisconnect: true` because a successful
drone AP association can drop the app's internet path until the reconnect
endpoint is called or NetworkManager restores the previous connection.

On single-radio machines, direct PC Wi-Fi association is still one drone AP at a
time. Mixed-link operation is available by using ESP32 bridges for additional
drone APs; each ESP32 owns one AP association and the PC talks to it over USB
serial.

## Implemented Control-Station Surfaces

The Electron UI now exposes:

- platform/network status, scan, connect, and reconnect controls
- provisional drone creation from likely drone SSIDs such as `WIFI_8K-*`
- manual IO configuration for direct UDP and ESP32 serial links, including
  interface, serial port, SSID, IP, port, protocol, bind-device, and
  packet-emission enablement
- per-flight policy editing for max throttle, command rate, slew rate, and
  heartbeat requirement
- frame-sequence import from repository-local paths
- frame-record reveal and export actions for MJPEG and MP4, with MP4 requiring
  `ffmpeg`
- pose-track status, automatic compute, and trajectory display
- Gaussian splat reconstruction controls and external `gsplat.js` viewer launch
- realtime runtime status, controller selection, runtime arm/disarm, per-drone
  link/controller/safety/observation state, and coordinator assignment summary

## Realtime Runtime

The realtime architecture now exists as bounded Python modules above the
verified packet and transport layer:

- `DroneAction` is the shared action representation.
- `protocols.py` converts actions into verified E99/WIFI_8K control packets.
- `transport.py` exposes `DroneLink`, with direct UDP and ESP32 serial bridge
  implementations behind the same interface.
- `runtime.DroneRuntime` owns one-drone lifecycle, observation creation,
  controller stepping, safety wrapping, packet building, link send, snapshots,
  and typed event emission.
- `runtime.RuntimeManager` owns multi-drone lifecycle, controller selection,
  manual axes forwarding, arm/disarm/heartbeat calls, coordinator-assignment
  constraint application, and event fan-out.
- `perception.state` defines frame, pose, IMU, map-summary, and confidence
  models used in runtime snapshots. `perception.imu` extracts IMU samples from
  JSONL/CSV logs, `perception.pipeline` aggregates frame/pose/IMU/map status,
  and `perception.maps` summarizes stored map and scene records without turning
  reconstruction into a realtime motor path.
- `controllers.base`, `scripted`, `manual`, `autonomy`, `text_command`,
  `local_vla`, `safety`, and `vla` define the
  bounded action-request contract. Scripted and manual controllers share the
  same safety wrapper. Built-in autonomy is a conservative offline policy for
  dry-run and provider-failure operation. The local VLA client is a JSON-lines
  subprocess adapter. The VLA adapter validates structured output, includes
  recent actions in the model input, and faults to motor stop when unavailable
  or invalid.
- `coordinator.tasks`, `scheduler`, and `vlm` define mission, role, assignment,
  constraint, and progress models. The scheduler assigns summary-level roles;
  it never touches packets or motor commands. VLM output is schema-checked and
  cannot assign unknown drones. `coordinator.http_vlm` is the internet-side JSON
  POST adapter for a hosted coordinator.
- `swarm.py` is now a CLI facade over `RuntimeManager`, not a duplicate packet
  loop. The service and CLI share the same safety/runtime path.

The runtime path preserves the full deterministic lower layer:

```text
mission/task request
  -> VLM swarm coordinator
    -> per-drone roles, routes, and constraints
      -> VLA single-drone controller
        -> bounded action requests
          -> realtime estimator and safety envelope
            -> DroneAction
              -> PacketProtocol
                -> DroneLink
                  -> direct UDP or ESP32 serial bridge
```

The module layout is:

```text
drone_control/runtime/
  events.py          typed runtime events and observations
  drone_runtime.py   one-drone loop: link, camera, estimator, controller, safety
  manager.py         multi-drone lifecycle and event fan-out

drone_control/perception/
  frames.py          frame sources and timestamps
  state.py           pose, IMU, map, and confidence data models
  imu.py             JSONL/CSV IMU extraction
  pipeline.py        frame, pose, IMU, and map aggregation
  estimator.py       realtime estimator interface
  slam.py            multi-view SLAM depth front-end (sparse ORB tracks +
                     dense plane-sweep); produces the metric depth map + cloud
  mvs.py             dense plane-sweep multi-view stereo + edge-aware densify
  depth.py           monocular depth prior (DEPRECATED front-end; eval showed it
                     structurally broken on these frames — kept for the point
                     cloud / PLY helpers only)
  segmentation.py    open-vocab detection + world-space grounding via depth

drone_control/controllers/
  base.py            controller protocol
  manual.py          manual controller adapter
  scripted.py        deterministic test controller
  local_vla.py       JSON-lines local model process client
  vla.py             single-drone VLA adapter
  safety.py          command clamping, heartbeat, stop, fault behavior

drone_control/coordinator/
  tasks.py           mission, role, and assignment data models
  http_vlm.py        internet-side VLM JSON POST client
  vlm.py             swarm-level VLM adapter
  scheduler.py       coordinator loop and per-drone constraints
```

Runtime packet emission remains disabled unless `DRONE_RUNTIME_ENABLE_IO=1` is
set. The default service runtime is dry-run (`DRONE_RUNTIME_DRY_RUN=1`), and it
loads `DRONE_RUNTIME_CONFIG`, then ignored `config/drones.local.json`, then the
tracked example config. This lets the UI exercise controller switching and
mission progress on a fresh checkout without opening serial or UDP links.

`POST /api/mission/start` starts the runtime loop and selects the controller
mode from `controllerMode` or `DRONE_MISSION_CONTROLLER` (default:
`autonomy`). It does not arm drones; arming remains an explicit per-drone safety
action. While a mission is active, the service autonomy loop continuously
advances the VLM/scheduler, applies coordinator constraints, and heartbeats the
runtime safety layer for already-armed drones.

Model integration is configured through explicit environment variables:

- `DRONE_LOCAL_VLA_COMMAND`: command string or JSON string list for a local
  JSON-lines VLA process.
- `DRONE_LOCAL_VLA_TIMEOUT`: per-step timeout for that process.
- `DRONE_VLM_ENDPOINT`: internet-side VLM coordinator endpoint.
- `DRONE_VLM_API_KEY`: optional bearer token for the VLM endpoint.
- `DRONE_VLM_TIMEOUT`: HTTP timeout for coordinator calls.

If those are absent, the runtime remains usable with manual, scripted, and
text-command controllers, while VLA/VLM paths fail closed or fall back to the
deterministic scheduler.

Replay and simulation tests use `tools/fixtures/runtime_replay.json` through
`runtime.replay`. Those fixtures validate the controller/coordinator contracts
without requiring drone hardware. Real flight traces should replace or augment
the fixture once hardware runs are recorded.

## Compatibility Cleanup Direction

The duplicated `swarm.py` packet loop has been retired. Remaining cleanup is:

1. Keep `single.py` as a hardware-oriented one-drone utility until each of its
   specialized manual test modes has a runtime equivalent.
2. Keep transport, protocol, and config compatibility intact.
3. Remove unused adapters and docs once tests and runbook commands use the new
   runtime path.

See [../TASKS.md](../TASKS.md) for the implementation checklist intended for
the next development session.
