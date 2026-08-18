#!/bin/bash
# Test 2B-C — Sync scaling preset D
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "========================================"
echo "  Test 2B-C — Preset D"
echo "  D — heavy physics / heavy visual"
echo "========================================"
echo ""
echo "  6 links | 97 rings | 16 radial | 12 capsules"
echo ""

export SYNC_PRESET="D"

"$HOME/isaacsim/python.sh" "$SCRIPT_DIR/run_sync_scaling.py"
