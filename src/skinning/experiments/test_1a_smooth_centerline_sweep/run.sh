#!/bin/bash
# Test 1A — Smooth centerline sweep (5 configs)
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "========================================"
echo "  Test 1A — Smooth centerline sweep (5 configs)"
echo "========================================"
echo ""
"$HOME/isaacsim/python.sh" "$SCRIPT_DIR/run_smooth_bridge.py"
