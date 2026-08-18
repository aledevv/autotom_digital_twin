#!/bin/bash
# Test 2C-A FIXED — Single branch junction
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "========================================"
echo "  Test 2C-A FIXED — Branch junction"
echo "========================================"
echo ""
echo "Fixes:"
echo "  - UsdPhysics.ArticulationRootAPI"
echo "  - exporter-style collision filtering"
echo "  - child-oriented junction D6 frame"
echo ""

"$HOME/isaacsim/python.sh" "$SCRIPT_DIR/run_branch_junction_fixed.py"
