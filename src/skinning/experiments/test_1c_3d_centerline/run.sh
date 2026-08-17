#!/bin/bash
# Test 1C — 3D curved centerline bridge
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "========================================"
echo "  Test 1C — 3D curved centerline bridge"
echo "========================================"
echo ""
"$HOME/isaacsim/python.sh" "$SCRIPT_DIR/run_3d_bridge.py"
