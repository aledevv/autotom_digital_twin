#!/bin/bash
# Test 2B-C — Sync scaling preset A
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "========================================"
echo "  Test 2B-C — Preset A"
echo "  A — light physics / light visual"
echo "========================================"
echo ""
echo "  3 links | 37 rings | 14 radial | 6 capsules"
echo ""

export SYNC_PRESET="A"

"$HOME/isaacsim/python.sh" "$SCRIPT_DIR/run_sync_scaling.py"
