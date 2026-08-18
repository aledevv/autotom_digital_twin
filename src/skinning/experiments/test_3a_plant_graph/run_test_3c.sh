#!/bin/bash
# Test 3C - Physical Truss With Detachable Tomatoes
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "========================================"
echo "  Test 3C - Physical Truss With Detachable Tomatoes"
echo "========================================"
echo ""
echo "  data-driven plant + articulated rachis/pedicels + 3 tomatoes"
echo "  reinforced plant physics = 2.5 MPa"
echo "  dedicated stiff truss physics = exporterV2 profile"
echo "  240 Hz / solver 32-4"
echo "  ground collision OFF"
echo "  invisible capsule proxies"
echo "  junction swelling + root flare"
echo ""
echo "Interaction:"
echo "  SHIFT + LEFT CLICK + drag"
echo ""

"$HOME/isaacsim/python.sh" \
"$SCRIPT_DIR/run_plant_graph_3c.py"
