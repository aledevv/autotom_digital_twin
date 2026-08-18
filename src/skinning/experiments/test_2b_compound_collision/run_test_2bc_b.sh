#!/bin/bash
# Test 2B-C — Sync scaling preset B
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "========================================"
echo "  Test 2B-C — Preset B"
echo "  B — light physics / heavy visual"
echo "========================================"
echo ""
echo "  3 links | 97 rings | 16 radial | 6 capsules"
echo ""

export SYNC_PRESET="B"

"$HOME/isaacsim/python.sh" "$SCRIPT_DIR/run_sync_scaling.py"
