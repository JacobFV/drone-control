# Multi-level control stack

The swarm is controlled at three frequencies, with **no analytic fallbacks** —
each tier is the real thing or it is off.

```
 drone  <--|hi-freq ~20Hz|-->  per-drone realtime controller
                                   (SafetyController + link I/O; DroneRuntime.step_once)
        <--|med-freq ~batched|-->  VLA  (BatchedVLAController + BatchedVLAHub)
                                   one batched forward pass, each drone acts individually
        <--|low-freq ~1 call/5s|-->  LLM director  (coordinator/llm.py)
                                   vLLM/Anthropic/OpenAI tool-calling: set_target / set_trajectory
```

## Low-frequency: LLM director (`drone_control/coordinator/llm.py`)
- Provider-agnostic tool-calling over stdlib HTTP — **Anthropic Messages** or
  **OpenAI-compatible** chat completions. Configure provider / model / API key /
  base URL in the **Brain** tab (persisted to `config/coordinator.local.json`,
  gitignored) or via `DRONE_LLM_PROVIDER|MODEL|API_KEY|BASE_URL`.
- Runs at `DRONE_COORDINATOR_HZ` (default **0.2 Hz** = 1 call / 5 s).
- Tools today: `set_target(droneId,x,y,z)` and `set_trajectory(droneId,waypoints,loop)`.
  These land on the `GuidanceBus` and condition the medium tier.
- **No fallback**: if no model is configured the mission reports `unavailable`
  and issues no guidance (the old analytic `CoordinatorScheduler` fallback was
  removed from `service._advance_mission`).

What an LLM at 1 call / 5 s should see/do (current + intended payload): each
drone's id, world position, link state, battery, and the mission objective; it
emits a few target/trajectory tool calls. Good candidates to add as the project
grows: per-drone "believed" coverage, detected world objects (from segmentation),
and nearby unexplored regions.

## Medium-frequency: VLA (`drone_control/controllers/batched_vla.py`)
- `BatchedVLAController` coalesces all drones' observations into one batched call
  to a VLA model process (`DRONE_BATCHED_VLA_COMMAND`, JSON-lines stdin/stdout),
  returning a per-drone action. Each drone acts individually; the batch is just
  for throughput.
- Per-drone payload already includes the camera frame (`frameJpegB64`),
  recent actions, constraints, and the resolved guidance (`goalRel`, `style`,
  `policyId`).
- **No analytic fallback**: medium control requires a VLA model command; without
  one there is no medium tier (drones stay disabled rather than falling back to
  the bounded-autonomy heuristic).

### Trained VLA
A diffusion VLA (`tools/diffusion_vla_model.py`) is trained on simulator
trajectories — swarm (multi-drone) demonstrations from the analytic expert
across several scenes + tasks — with a loss weighted toward the axes that matter
(orientation/yaw and directive-following roll/pitch over raw throttle) and
sample-weighted toward decisive maneuvers. Retrain with `tools/train_vla.sh`;
the checkpoint lands at `runs/vla.pt` and the runtime auto-loads it as the
batched VLA controller. Closed-loop eval (the deploy path): untrained hovers
(goal-distance improvement ~0.00); trained reaches goals (improvement ~0.89).

### Intended VLA input enrichment (next)
In addition to the camera, feed each drone's VLA the **egocentric**:
- immediate depth map (from `perception/depth.py`),
- nearby point-cloud density (from the accumulated cloud),
- nearby gaussian-splat density (from the live splat).
Seam: `BatchedVLAController._build_payload` + a per-drone egocentric summary
sourced from `SessionService.depth` / splat. (Not yet wired.)

## High-frequency: realtime controller
- `DroneRuntime.step_once` at `DRONE_RUNTIME_HZ` (default 20 Hz) wraps the chosen
  controller in `SafetyController` (throttle clamp, heartbeat, slew) and sends the
  sanitized action over the link.

## Future: internal world model + counterfactual exploration
Planned (not yet implemented): maintain an internal world-model estimate
(regardless of whether the environment is sim or real), and give the LLM
director a tool to **dispatch counterfactual exploration simulations** against
that model — "what happens if drone 2 goes left?" — to inform high-level
direction. Tracked as future work; the environment abstraction
(`drone_control/environment/`) is the seam where a model-backed environment would
plug in alongside `SimEnvironment` / `RealEnvironment`.
