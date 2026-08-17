#!/bin/bash
# Test 0F — Curved centerline bridge with skinning animation
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "========================================"
echo "  Test 0F — Curved centerline bridge with skinning animation"
echo "========================================"
echo ""
"$HOME/isaacsim/python.sh" "$SCRIPT_DIR/run_curved_bridge.py"
