#!/bin/bash
# Test 2C-C RAW — same physics, no visual junction blend
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "========================================"
echo "  Test 2C-C RAW — A/B reference"
echo "========================================"
echo ""
echo "  EXACT SAME physics"
echo "  visual junction blend OFF"
echo ""

JUNCTION_BLEND=0 \
"$HOME/isaacsim/python.sh" \
"$SCRIPT_DIR/run_junction_visual.py"
