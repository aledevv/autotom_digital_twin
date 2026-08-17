#!/bin/bash
# Test 1B — Radius taper bridge
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "========================================"
echo "  Test 1B — Radius taper bridge"
echo "========================================"
echo ""
"$HOME/isaacsim/python.sh" "$SCRIPT_DIR/run_taper_bridge.py"
