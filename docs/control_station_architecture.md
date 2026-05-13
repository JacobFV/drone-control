# Control Station Architecture

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
- `GET /api/records/<record-id>/mjpeg`
- `GET /api/wifi/capabilities`
- `GET /api/manual/status`
- `POST /api/manual/arm`
- `POST /api/manual/disarm`
- `POST /api/manual/heartbeat`
- `POST /api/manual/axes`
- `POST /api/manual/stop`
- `POST /api/manual/clear-fault`

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
- Any lost drone ACK/keepalive ramps throttle to zero.
- Throttle commands are rate-limited.
- Maximum throttle is capped by a per-flight policy until deliberately raised.
- Switching away from manual mode sends a ramped stop.
- Closing the app sends a ramped stop when a manual session is active.

The Electron control pad sends desired axes to the local service. The service
holds the safety state machine and a 20 Hz command loop. UDP output is disabled
by default; setting `DRONE_SERVICE_ENABLE_IO=1` enables packet emission through
the configured `DRONE_IFACE`, `DRONE_IP`, `DRONE_PORT`, and `DRONE_PROTOCOL`.
Opening the app without that flag cannot send motor packets.

## Wi-Fi Concurrency

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

For multiple simultaneous drones, the cleanest reliable path is still multiple
Wi-Fi adapters, one managed interface per drone AP. Single-radio multi-interface
support can be used opportunistically after a full connect/ping test passes.
