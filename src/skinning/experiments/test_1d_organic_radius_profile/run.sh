#!/bin/bash
# Test 1D — Organic radius profile bridge
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "========================================"
echo "  Test 1D — Organic radius profile bridge"
echo "========================================"
echo ""
"$HOME/isaacsim/python.sh" "$SCRIPT_DIR/run_organic_bridge.py"
