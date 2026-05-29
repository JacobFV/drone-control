#!/usr/bin/env bash
# Train the medium-frequency VLA on simulator trajectories and save runs/vla.pt.
#
# Collects swarm (multi-drone) demonstrations from the analytic expert across
# several scenes + tasks — so the policy learns orientation, directive-following
# (fly toward the goal) and swarm spacing — then trains the diffusion VLA with a
# loss weighted toward those axes. The runtime auto-loads runs/vla.pt as the
# batched VLA controller (see service.default_batched_vla_command).
#
# Usage:  PYTHONPATH=. tools/train_vla.sh
set -euo pipefail
cd "$(dirname "$0")/.."
PY="${PYTHON:-.venv/bin/python}"
DATA=data/sim/vla_train.jsonl
DEV="${DEVICE:-cuda}"

echo "[1/2] collecting swarm demonstrations -> $DATA"
"$PY" tools/collect_sim_data.py --out "$DATA" --scene warehouse --task goto    --num-envs 6 --num-drones 6 --steps 300 --log-every 6 --device "$DEV"
"$PY" tools/collect_sim_data.py --out "$DATA" --scene city      --task goto    --num-envs 6 --num-drones 6 --steps 300 --log-every 6 --seed 1 --append --device "$DEV"
"$PY" tools/collect_sim_data.py --out "$DATA" --scene park      --task formation --num-envs 6 --num-drones 6 --steps 300 --log-every 6 --seed 2 --append --device "$DEV"
"$PY" tools/collect_sim_data.py --out "$DATA" --scene open_field --task goto   --num-envs 6 --num-drones 6 --steps 300 --log-every 6 --seed 3 --append --device "$DEV"

echo "[2/2] training -> runs/vla.pt"
"$PY" tools/train_diffusion_vla.py "$DATA" --out runs/vla.pt --epochs 30 --batch-size 128 --device "$DEV"
echo "done — restart the service to pick up the new VLA."
