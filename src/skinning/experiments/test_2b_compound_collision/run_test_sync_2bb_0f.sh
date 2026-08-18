#!/bin/bash
# Sync diagnostic — 2B-B with Test 0F settings
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "========================================"
echo "  Sync diagnostic — 2B-B / 0F settings"
echo "========================================"
echo ""
echo "Target workload:"
echo "  3 rigid links / 3 bones / 2 D6"
echo "  37 visual rings"
echo "  14 radial vertices"
echo "  radius 12 mm"
echo "  joint damp 0.05 / limit 89 deg"
echo ""

"$HOME/isaacsim/python.sh" "$SCRIPT_DIR/run_sync_2bb_0f_settings.py"
