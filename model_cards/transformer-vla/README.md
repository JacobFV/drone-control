---
license: mit
tags:
  - robotics
  - drone
  - vla
  - transformer
pipeline_tag: robotics
---

# Transformer VLA (drone-control)

Higher-capacity ViT-style image→action policy for the multi-drone control
station's **medium-frequency** tier — more headroom than the tiny baseline for
orientation, directive-following and swarm behaviour.

- **Architecture:** 8×8 patch embedding over the 64×64 camera frame + a
  proprio/goal/style token, fused by a 6-layer transformer encoder, regressed to
  a continuous 4-axis action in `[-1,1]`.
- **Params:** ~5–8M. GPU recommended.
- **Inputs/outputs:** identical interface to the tiny VLA (same batched serving
  protocol), so policies are interchangeable in the app.
- **Training:** imitation from analytic-expert **swarm** trajectories across
  scenes/tasks, weighted toward orientation + directive-following.
- **Eval (closed loop):** goal-distance improvement ~0.87.

## Files
- `transformer_vla.pt` — checkpoint for
  `tools/transformer_vla_policy.py --checkpoint transformer_vla.pt`.

## Use in the app
Download from the **Models** tab and select it. Train locally with
`tools/train_transformer_vla.py data/sim/vla_train.jsonl`.

This is the in-repo "serious" transformer policy. True foundation VLAs
(OpenVLA, π0, …) can be added as additional registry entries with their own
serving wrappers.

Code & docs: https://github.com/jacobfv/drone-control
