#!/bin/bash
# Test 2B — Compound collision
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "========================================"
echo "  Test 2B — Compound collision"
echo "========================================"
echo ""
"$HOME/isaacsim/python.sh" "$SCRIPT_DIR/run_compound_collision.py"
