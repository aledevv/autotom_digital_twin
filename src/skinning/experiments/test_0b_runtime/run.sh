#!/bin/bash
# Lancia Test 0B (UsdSkel runtime animation) in Isaac Sim.
#
# Uso:
#   ./run.sh
#   (oppure: bash src/skinning/experiments/test_0b_runtime/run.sh)

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "========================================"
echo "  Test 0B — UsdSkel Runtime Animation"
echo "========================================"
echo ""
"$HOME/isaacsim/python.sh" "$SCRIPT_DIR/run.py"
