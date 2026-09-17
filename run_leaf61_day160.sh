#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec "$PROJECT_DIR/run_leaf61_capacity_optimized.sh" \
  --leaves 127 --canopy-selection primary --collision-filter groups \
  --sleep-threshold 0.00005 --visual-tolerance 0.000001 --skin-writes sdf \
  --gui --recording first-trial "$@"
