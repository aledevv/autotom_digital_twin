#!/bin/bash
# Test 2B-C — Sync scaling preset C
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "========================================"
echo "  Test 2B-C — Preset C"
echo "  C — heavy physics / light visual"
echo "========================================"
echo ""
echo "  6 links | 37 rings | 14 radial | 12 capsules"
echo ""

export SYNC_PRESET="C"

"$HOME/isaacsim/python.sh" "$SCRIPT_DIR/run_sync_scaling.py"
