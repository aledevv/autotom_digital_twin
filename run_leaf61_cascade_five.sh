#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec "$PROJECT_DIR/run_leaf61_cascade.sh" \
  --leaves 5 --cascade-layout "$PROJECT_DIR/src/experiments/leaf61/configs/cascade-five.json" "$@"
