#!/bin/bash
# Test 3A-A — Data-driven Plant Graph
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "========================================"
echo "  Test 3A-A — Plant Graph"
echo "========================================"
echo ""
echo "  data-driven main + 3 laterals"
echo "  baseline physics = 2D-B2"
echo "  240 Hz / solver 32-4"
echo "  ground collision OFF"
echo "  invisible capsule proxies"
echo "  junction swelling + root flare"
echo ""
echo "Interaction:"
echo "  SHIFT + LEFT CLICK + drag"
echo ""

"$HOME/isaacsim/python.sh" \
"$SCRIPT_DIR/run_plant_graph.py"
