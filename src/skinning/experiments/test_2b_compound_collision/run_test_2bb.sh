#!/bin/bash
# Test 2B-B — Corrected compound capsule collision proxy
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "========================================"
echo "  Test 2B-B — Corrected capsule proxy"
echo "========================================"
echo ""

"$HOME/isaacsim/python.sh" "$SCRIPT_DIR/run_compound_capsule.py"
