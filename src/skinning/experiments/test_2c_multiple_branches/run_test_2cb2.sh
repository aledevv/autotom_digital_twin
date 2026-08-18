#!/bin/bash
# Test 2C-B2 — Multiple branches with beam-based spring physics
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "========================================"
echo "  Test 2C-B2 — Beam physics"
echo "========================================"
echo ""
echo "  same topology as 2C-B"
echo "  suspended plant"
echo "  ground collision OFF"
echo "  non-zero beam-derived K/D"
echo "  bend limit +/-30 deg"
echo ""

"$HOME/isaacsim/python.sh" "$SCRIPT_DIR/run_multiple_branches_beam.py"
