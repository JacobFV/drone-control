---
license: mit
tags:
  - robotics
  - drone
  - vla
  - imitation-learning
pipeline_tag: robotics
---

# Tiny Diffusion VLA (drone-control)

Compact image→action reverse-diffusion policy for the multi-drone control
station — the **medium-frequency** controller in the stack
(drone ↔ realtime controller ↔ **VLA** ↔ LLM director).

- **Inputs:** forward camera frame (64×64) + proprioceptive vector
  (pose, link state, recent action, goal-relative target, style).
- **Output:** continuous 4-axis action `(roll, pitch, throttle, yaw)` in `[-1,1]`,
  mapped to `[0,255]` bytes (128 = neutral).
- **Params:** ~0.2M. CPU/GPU. Fast.
- **Training:** imitation from analytic-expert **swarm** trajectories in the
  simulator, across multiple scenes/tasks, with a loss weighted toward
  orientation (yaw) and directive-following (roll/pitch), plus decisive-maneuver
  sample weighting.
- **Eval (closed loop):** goal-distance improvement ~0.89 (untrained ~0.00).

## Files
- `vla.pt` — checkpoint for `tools/diffusion_vla_policy.py --checkpoint vla.pt`.

## Use in the app
Download it from the **Models** tab (cloud icon) and select it as the active
policy. Or train locally with `tools/train_vla.sh`.

Code & docs: https://github.com/jacobfv/drone-control
