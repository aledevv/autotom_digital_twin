#!/bin/bash
# Test 2B-C1 — Single-capsule sync diagnostic
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "========================================"
echo "  Test 2B-C1 — Single capsule / link"
echo "========================================"
echo ""
echo "  6 rigid links / 6 bones / 5 D6"
echo "  37 visual rings / 14 radial"
echo "  1 capsule per link = 6 capsules"
echo ""
echo "Compare directly with preset C:"
echo "  C  = 12 capsules -> delay"
echo "  C1 =  6 capsules -> ?"
echo ""

"$HOME/isaacsim/python.sh" "$SCRIPT_DIR/run_sync_c1_single_capsule.py"
