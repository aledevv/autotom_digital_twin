#!/bin/bash
# Test 2D-B4 — V2-aligned manual branch interaction
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "========================================"
echo "  Test 2D-B4 — V2 aligned"
echo "========================================"
echo ""
echo "  NO gravity ramp"
echo "  gravity = 9.81 from frame 0"
echo "  rigid main stem (V2-style)"
echo "  480 Hz"
echo "  solver 32 / 4"
echo "  volume-density masses + explicit COM"
echo "  V2-style attachment stiffness"
echo "  soft lateral material preserved"
echo ""
echo "Interaction:"
echo "  SHIFT + LEFT CLICK + drag"
echo ""

"$HOME/isaacsim/python.sh" \
"$SCRIPT_DIR/run_manual_mouse_grab_v2aligned.py"
