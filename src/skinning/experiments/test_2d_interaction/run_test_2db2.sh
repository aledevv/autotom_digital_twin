#!/bin/bash
# Test 2D-B2 — Fixed manual SHIFT+CLICK interaction
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "========================================"
echo "  Test 2D-B2 — Mouse grab fixed"
echo "========================================"
echo ""
echo "  no hidden pre-settle"
echo "  damping ratio = 4.0"
echo "  stiffness unchanged"
echo "  mouse settings applied AFTER stage open"
echo "  invisible collision proxy picking enabled"
echo ""
echo "Interaction:"
echo "  SHIFT + LEFT CLICK + drag"
echo ""

"$HOME/isaacsim/python.sh" \
"$SCRIPT_DIR/run_manual_mouse_grab_v2.py"
