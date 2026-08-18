#!/bin/bash
# Test 2B-E — Double render without extra physics
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "========================================"
echo "  Test 2B-E — Double render"
echo "========================================"
echo ""
echo "Same workload as C1 / D:"
echo "  6 rigid links / 6 bones / 5 D6"
echo "  37 visual rings / 14 radial"
echo "  1 capsule per link = 6 capsules"
echo ""
echo "Runtime:"
echo "  physics step"
echo "    -> PhysX transform write-back"
echo "    -> sync skin"
echo "    -> render #1"
echo "    -> render #2"
echo ""
echo "No physics step occurs between render #1 and #2."
echo ""

"$HOME/isaacsim/python.sh" "$SCRIPT_DIR/run_double_render.py"
