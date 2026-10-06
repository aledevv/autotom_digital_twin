#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec /usr/bin/python3 "$PROJECT_DIR/src/experiments/leaf61/run.py" \
  --model skinning --scenario press --gui \
  --config "$PROJECT_DIR/src/experiments/leaf61/configs/skinning_candidate.json" \
  --shape real --mesh "$PROJECT_DIR/artifacts/leaf61/real-seed42.npz" "$@"
