#!/bin/bash
# Lancia Test 0A (UsdSkel statico) in Isaac Sim.
# Genera il file USDA e lo carica in Isaac Sim per la validazione visiva.
#
# Uso:
#   ./run.sh
#   (oppure: bash src/skinning/experiments/test_0a_static/run.sh)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "========================================"
echo "  Test 0A — UsdSkel Statico"
echo "========================================"
echo ""
echo "Avvio Isaac Sim..."
"$HOME/isaacsim/python.sh" "$SCRIPT_DIR/run.py"
