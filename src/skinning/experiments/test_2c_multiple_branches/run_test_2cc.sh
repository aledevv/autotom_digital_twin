#!/bin/bash
# Test 2C-C — blended junction
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "========================================"
echo "  Test 2C-C — Junction visual blend"
echo "========================================"
echo ""
echo "  suspended branch"
echo "  ground collision OFF"
echo "  parent node swelling ON"
echo "  child root flare ON"
echo ""

JUNCTION_BLEND=1 \
"$HOME/isaacsim/python.sh" \
"$SCRIPT_DIR/run_junction_visual.py"
