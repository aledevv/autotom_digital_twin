#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec /usr/bin/python3 "$PROJECT_DIR/src/experiments/leaf61/run.py" \
  --model skinning --shape real --mesh "$PROJECT_DIR/artifacts/leaf61/real-seed42.npz" \
  --canopy-usd "$PROJECT_DIR/artifacts/branch_collisions/C-organic-leaf-pair-settle60/scene.usda" \
  --layout canopy-drop --drop-test scale --leaves 120 --scenario press --hz 480 \
  --config "$PROJECT_DIR/src/experiments/leaf61/configs/goal-iterations32.json" \
  --ball-mass 0.02 --ball-radius 0.02 --drop-height 0.06 \
  --drop-runtime visual --geometry-method bounded --render --render-hz 30 "$@"
