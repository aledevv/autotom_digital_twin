#!/bin/bash
# Test 2A — Parametric branch
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "========================================"
echo "  Test 2A — Parametric branch"
echo "========================================"
echo ""
"$HOME/isaacsim/python.sh" "$SCRIPT_DIR/run_parametric_branch_diagnostic.py"
