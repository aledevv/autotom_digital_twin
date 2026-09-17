#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec /usr/bin/python3 "$PROJECT_DIR/src/experiments/leaf61/run.py" \
  --model skinning --shape real --mesh "$PROJECT_DIR/artifacts/leaf61/real-seed42.npz" \
  --canopy-usd "$PROJECT_DIR/artifacts/branch_collisions/C-organic-leaf-pair-settle60/scene.usda" \
  --layout canopy-drop --leaves 1 --scenario press --hz 480 \
  --ball-mass 0.02 --ball-radius 0.02 --drop-height 0.06 \
  --gui --drop-runtime optimized --playback-speed 0.25 "$@"
