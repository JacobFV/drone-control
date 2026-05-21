# TASKS: Full Realtime Architecture

This is the handoff point for the next implementation chat. The repo baseline
already supports direct UDP links and ESP32-S3 USB bridge links through the same
`DroneLink` interface. The next job is to build the full realtime runtime above
that foundation without changing the verified transport behavior first.

## Starting Baseline

- One ESP32-S3 bridge controls one drone AP over USB serial.
- Multiple simultaneous drones are supported by mixing multiple ESP serial
  links and optional direct UDP links in one config.
- The PC can keep its normal internet connection while ESP bridges own drone AP
  associations.
- Camera through the ESP bridge is not solved. The current ESP bridge forwards
  control packets only; live camera still uses the direct RTSP/RTP/JPEG path
  from a device associated with the drone AP.
- `DroneAction`, packet protocols, and link transports are the stable lower
  layers.
- The current Electron/Python service has manual IO, camera records, pose-track
  records, and Gaussian-splat reconstruction surfaces.

## Ground Rules

- Keep packet protocol and link transport deterministic and model-free.
- No controller may write UDP packets directly; all output becomes
  `DroneAction` through the safety layer.
- Do not assume ESP bridge camera support exists. Any realtime perception work
  must either use a direct camera network path, recorded frame source, or a new
  explicitly designed ESP/video bridge.
- The swarm coordinator assigns work and constraints, not motor commands.
- Add tests around every new interface before wiring hardware paths through it.
- Preserve ignored local files: `config/drones.local.json` and `.drone.env`.
- Keep the project framing civilian: firefighting support, inspection,
  search-and-rescue training, environmental monitoring, and low-cost robotics
  research.

## Phase 0: Confirm Clean Baseline

- Run the Python compile checks from `README.md`.
- Run `python3 -m unittest tools.test_transport tools.test_service_manual_ack`.
- Run `python3 tools/test_smooth_camera_frames.py`.
- Run `npm run check`.
- Run the dry-run swarm command:

```bash
python3 -m drone_control.swarm --config config/drones.example.json --dry-run --seconds 0.2
```

- Build firmware from `firmware/esp32_drone_link` with `pio run`.
- Confirm `git status --short` is clean except ignored local runtime files.

## Phase 1: Runtime Skeleton

- Add `drone_control/runtime/events.py` with typed events:
  `RuntimeEvent`, `RuntimeErrorEvent`, `LinkStatusEvent`,
  `ObservationEvent`, and `ActionEvent`.
- Add `DroneObservation` with timestamp, drone id, link state, latest frame
  metadata, pose estimate, map summary, battery placeholder, and confidence.
- Add `drone_control/runtime/drone_runtime.py`.
- Add `DroneRuntime` lifecycle methods: `start()`, `stop()`, `step_once()`,
  `set_controller()`, and `snapshot()`.
- Add `drone_control/runtime/manager.py` for multi-drone lifecycle and event
  fan-out.
- Test with fake links and a deterministic scripted controller.

Definition of done:

- Existing direct UDP and ESP serial links can be passed into `DroneRuntime`
  without changing their public API.
- A fake two-drone runtime can run in tests and emit observations/actions.
- Current `swarm.py --dry-run` behavior still works.

## Phase 2: Controller Interfaces

- Add `drone_control/controllers/base.py` with a controller protocol that maps
  observation history plus constraints to a typed action request.
- Add `scripted.py` for deterministic neutral, takeoff, land, and stop
  sequences.
- Add `manual.py` as an adapter for service-driven manual axes.
- Add `safety.py` for rate limits, throttle caps, heartbeat, stop, and fault
  handling.
- Move simple command parsing out of the runtime path and into explicit
  controller selection.
- Update tests so controller output is clamped by safety before becoming
  `DroneAction`.

Definition of done:

- Manual and scripted controllers both share the same safety wrapper.
- No controller test depends on real Wi-Fi or serial hardware.

## Phase 3: Observation and Perception Pipeline

- Treat ESP camera transport as out of scope until a dedicated design exists.
  The current ESP bridge is a control link, not a video relay.
- Add `drone_control/perception/state.py` for pose, IMU sample, map summary,
  frame metadata, and confidence models.
- Add `perception/frames.py` adapters for live camera, stored frame sequence,
  and test frame sources.
- Add `perception/estimator.py` with an estimator interface that can wrap the
  current visual odometry path.
- Emit observation events from runtime even when no camera frame is available.
- Keep Gaussian-splat reconstruction as an artifact pipeline, then add a map
  summary adapter when a record exists.

Definition of done:

- Runtime snapshots include typed perception status.
- Tests cover no-camera, frame-only, and pose-available states.

## Phase 4: Single-Drone VLA Adapter

- Add `drone_control/controllers/vla.py` behind the base controller protocol.
- Define a strict structured output schema for bounded action requests.
- Include recent observations, recent actions, safety constraints, and mission
  context in the adapter input.
- Validate and clamp all model output before `DroneAction` creation.
- Add replay tests using stored observation/action fixtures.

Definition of done:

- VLA adapter can be disabled cleanly when credentials or dependencies are
  absent.
- Bad model output causes a safe stop/fault, not packet emission.

## Phase 5: Swarm Coordinator

- Add `drone_control/coordinator/tasks.py` with mission, role, assignment,
  constraint, and progress models.
- Add `coordinator/scheduler.py` for coordinator loop timing and per-drone
  constraint updates.
- Add `coordinator/vlm.py` as the structured VLM adapter.
- Feed the coordinator summaries, not raw packet or motor interfaces.
- Add two-drone and three-drone dry-run tests with mixed link configurations.

Definition of done:

- The coordinator can assign roles to multiple simulated drones.
- Each drone remains controlled by its own runtime and safety wrapper.

## Phase 6: Service and UI Integration

- Add service endpoints for runtime status, event stream, controller selection,
  mission start, mission stop, and mission progress.
- Keep packet output disabled by default unless explicitly enabled.
- Update the Electron UI to show per-drone link type, controller, safety state,
  observation confidence, and coordinator assignment.
- Keep manual control available as a first-class controller mode.

Definition of done:

- The UI can switch a drone between manual, scripted, and disabled controller
  modes without restarting the service.
- Mission controls cannot bypass arming or safety requirements.

## Phase 7: Replay and Simulation Tests

- Add reusable fixture records for frame metadata, pose tracks, and action
  traces.
- Add runtime replay tests that do not require hardware.
- Add timing tests for command-rate and heartbeat behavior.
- Add coordinator tests for missing drone, low confidence, and link fault
  scenarios.

Definition of done:

- A fresh checkout can validate most architecture behavior without drones.
- Hardware tests are isolated behind explicit commands and local configs.

## Phase 8: Hardware Bring-Up

- Start with one ESP bridge and one drone AP.
- Run scripted runtime neutral, takeoff, hold, land, and stop sequences.
- Add a second ESP bridge and confirm simultaneous two-drone neutral control.
- Add direct UDP as a third link only when the PC Wi-Fi association is
  deliberate.
- Record runbook observations with exact commands, dates, AP SSIDs, and commit
  hashes.

Definition of done:

- The runtime path can control at least two drones simultaneously through two
  ESP bridges in a controlled test area.
- The PC stays on its normal network during ESP bridge tests.

## Files To Read First

- [README.md](README.md)
- [docs/control_station_architecture.md](docs/control_station_architecture.md)
- [docs/video_narrative.md](docs/video_narrative.md)
- [DRONE_RUNBOOK.md](DRONE_RUNBOOK.md)
- [drone_control/transport.py](drone_control/transport.py)
- [drone_control/protocols.py](drone_control/protocols.py)
- [drone_control/swarm.py](drone_control/swarm.py)
- [drone_control/service.py](drone_control/service.py)
- [firmware/esp32_drone_link/README.md](firmware/esp32_drone_link/README.md)

## Commands For The Next Session

```bash
git status --short

python3 -m py_compile \
  drone_control/transport.py \
  drone_control/config.py \
  drone_control/swarm.py \
  drone_control/single.py \
  drone_control/manual_transport.py \
  drone_control/service.py \
  tools/test_transport.py \
  tools/test_service_manual_ack.py \
  tools/esp_scan.py

python3 -m unittest tools.test_transport tools.test_service_manual_ack
python3 tools/test_smooth_camera_frames.py
npm run check
python3 -m drone_control.swarm --config config/drones.example.json --dry-run --seconds 0.2

cd firmware/esp32_drone_link
pio run
```
