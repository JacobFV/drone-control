# TASKS: Full Realtime Architecture

This file is the implementation checklist for the realtime runtime above the
verified direct UDP and ESP32-S3 USB bridge `DroneLink` foundation.

## Current Implementation Status

Implemented:

- typed runtime events and observations in `drone_control/runtime/events.py`
- one-drone runtime lifecycle in `drone_control/runtime/drone_runtime.py`
- multi-drone runtime manager and event fan-out in
  `drone_control/runtime/manager.py`
- controller contracts, scripted controllers, manual adapter, shared safety
  wrapper, built-in bounded autonomy controller, text-command controller,
  process-backed local VLA client, and strict structured VLA adapter in
  `drone_control/controllers/`
- typed perception state, frame-source adapters, and estimator adapter boundary
  in `drone_control/perception/`, plus IMU file extraction, perception
  aggregation, and map-summary adapters for stored scene artifacts
- mission/task/assignment models, scheduler, HTTP internet-side VLM client, and
  strict VLM adapter in `drone_control/coordinator/`; mission assignments now
  update per-drone runtime safety constraints instead of remaining display-only
- `swarm.py` now uses `RuntimeManager` instead of a duplicate packet loop, so
  CLI, service, tests, safety, and event emission share one runtime path
- service endpoints for runtime status, runtime events, controller selection,
  per-drone arm/disarm/heartbeat/axes/stop/clear-fault, mission start/stop, and
  mission progress; mission start launches runtime and an active autonomy loop
  by default but does not arm drones
- Electron runtime panel showing link type/state, controller, safety state,
  observation confidence, active max throttle, and coordinator assignment
- replay fixtures and dry-run runtime tests covering no-camera,
  frame+pose+IMU+map, safety clamping, built-in autonomy, two-drone manager
  behavior, coordinator fault scenarios, process-backed VLA, HTTP VLM, VLA
  schema validation, VLM assignment validation, IMU extraction, perception
  aggregation, and replay loading

Batched VLA + live world model (added):

- batched VLA loop in `drone_control/controllers/batched_vla.py`: a coalescing
  `BatchedVLAHub` turns N per-drone control ticks into one model call per window
  (early-flush when all registered drones submit, else `batch_max_wait_seconds`),
  with per-drone `BatchedVLAController` proxies. Failures/timeouts → `motor_stop`,
  still clamped by the safety wrapper. Mode `batched_vla` in `RuntimeManager`;
  swarm-wide via `/api/runtime/controller`.
- batched JSON-lines protocol + `BatchLocalVLAClient` in
  `controllers/local_vla.py`: stdin `{"batch":[...]}` → stdout `{"results":[...]}`,
  one round-trip per window, with a startup-grace window for model cold start.
- reverse-diffusion image→action reference policy in `tools/diffusion_vla_model.py`
  (x0-parameterised conditional DDPM; vision CNN + proprio MLP; zero-init head ⇒
  untrained output is a safe neutral action), served by `tools/diffusion_vla_policy.py`
  and trained from logged transitions by `tools/train_diffusion_vla.py`
  (`DRONE_VLA_LOG_PATH` enables the runtime transition logger).
- live cross-drone Gaussian-splat world model in
  `drone_control/perception/live_splat.py`: one persistent CUDA gaussian set in a
  shared world frame, fed by `ingest(drone_id, jpeg, pose)` from all drones; a
  background gsplat optimiser with light densify/prune; self-bootstraps by
  back-projecting the first keyframe. Exposed via `RuntimeManager`
  (`start_world_model`/`ingest_frame`/`export_world_model`) and
  `/api/world/splat/{status,snapshot,start,stop,bootstrap}`. `gsplat` verified on
  the GB10 (Blackwell sm_121, cu130).
- shared `LiveFrameRegistry` (`perception/frame_registry.py`) carries real JPEG
  bytes once to both the diffusion policy and the splat engine.
- tests: `tools/test_batched_vla.py` (coalescing, routing, timeout/error/invalid →
  stop, end-to-end through the diffusion subprocess) and `tools/test_live_splat.py`
  (gsplat-gated cross-drone fusion, loss decrease, PLY export, manager wiring).

Cross-drone co-registration, live ingestion, and UI (added):

- automatic COLMAP-union co-registration in `drone_control/perception/cross_drone.py`:
  the union of all drones' frames is fed into one COLMAP SfM run
  (`feature_extractor` → `exhaustive_matcher` → `mapper`, driven via the `colmap`
  CLI), jointly solving every camera into one shared frame. The sparse cloud
  seeds `LiveSplatEngine.seed_from_points`; per-drone `world_T_drone` similarity
  transforms are recovered by Umeyama alignment of VO camera centres to COLMAP
  centres. Exposed via `RuntimeManager.bootstrap_world_model` and
  `POST /api/world/splat/bootstrap {"flightIds": [...]}`. COLMAP binary parsers
  and Umeyama are unit-tested; the CLI driver is exercised in
  `tools/test_cross_drone.py`.
- live frame ingestion in `drone_control/perception/ingestion.py`: a
  `FrameIngestor` pumps any `live_video.FrameSource` (live RTP/JPEG camera or a
  `DirectoryFrameSource` replay) into `RuntimeManager.ingest_frame`, which
  publishes to the `LiveFrameRegistry` (batched VLA) and the splat engine in one
  decode. Pose defaults to the drone's latest runtime observation. Exposed via
  `POST /api/runtime/drones/<id>/camera/{start,stop}` (`{"framesDir": ...}` for
  replay or live camera config). Tested in `tools/test_ingestion.py`.
- React + Vite control-station UI under `ui/` replacing the flat-sidebar vanilla
  renderer: workflow-organised inspector (Connect → Fly → Record → Reconstruct)
  with a persistent Swarm · Batched VLA panel, a separate Settings drawer, and a
  main viewport with Forward / Down / 3D-Sim / World Model views. Electron loads
  `ui/dist` when built (falls back to `app/`); the `.ply` snapshot uses a
  `fetchBinary` IPC bridge. Build: `cd ui && npm install && npm run build`.

Still external hardware/model bring-up:

- live tracking/loop-closure/drift correction for the world model are not solved;
  reconstruction quality depends on upstream VO and frame overlap. COLMAP
  bootstrap needs real overlapping frames to register.
- train the diffusion policy on real transitions before relying on it to fly;
  untrained weights emit neutral actions only (`DRONE_VLA_LOG_PATH` logs
  transitions; `tools/train_diffusion_vla.py` trains; pass `--checkpoint`).
- point `DRONE_BATCHED_VLA_COMMAND` at `tools/diffusion_vla_policy.py` to use the
  real model; otherwise an in-process neutral fallback keeps the loop running.

- record one-ESP and two-ESP runtime bring-up in `DRONE_RUNBOOK.md`
- replace fixture replay records with representative real flight traces when
  available
- verify runtime packet emission with `DRONE_RUNTIME_ENABLE_IO=1` in a
  controlled test area
- configure production VLA/VLM endpoints and credentials:
  `DRONE_LOCAL_VLA_COMMAND`, `DRONE_VLM_ENDPOINT`, and `DRONE_VLM_API_KEY`

## Starting Baseline

- One ESP32-S3 bridge controls one drone AP over USB serial.
- Multiple simultaneous drones are supported by mixing multiple ESP serial
  links and optional direct UDP links in one config.
- The PC can keep its normal internet connection while ESP bridges own drone AP
  associations.
- Camera through the ESP bridge is not solved yet. The current ESP bridge
  forwards control packets only; live camera still uses the direct RTSP/RTP/JPEG
  path from a device associated with the drone AP. Making ESP-side camera
  transport work is an explicit research objective.
- `DroneAction`, packet protocols, and link transports are the stable lower
  layers.
- The current Electron/Python service has manual IO, camera records, pose-track
  records, and Gaussian-splat reconstruction surfaces.

## Ground Rules

- Keep packet protocol and link transport deterministic and model-free.
- No controller may write UDP packets directly; all output becomes
  `DroneAction` through the safety layer.
- Do not assume ESP bridge camera support exists until proven. Any realtime
  perception work must either use a direct camera network path, recorded frame
  source, or the new ESP camera bridge once it is designed and verified.
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

- Add an ESP camera-transport investigation before depending on live perception
  through ESP links:
  - capture the phone/direct-PC RTSP/RTP/JPEG startup sequence for one drone
  - identify which traffic must originate from the device associated with the
    drone AP
  - decide whether the ESP should proxy RTSP control, forward RTP/JPEG frames
    over USB serial, expose a USB network interface, or use another explicit
    relay design
  - prototype the smallest viable path and measure frame rate, latency, USB
    bandwidth, ESP memory pressure, and packet loss
  - document the result as supported, unsupported, or supported only under
    specific settings
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
