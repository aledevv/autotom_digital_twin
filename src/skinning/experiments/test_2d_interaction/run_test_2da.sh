#!/bin/bash
# Test 2D-A — Scripted physical grabber
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "========================================"
echo "  Test 2D-A — Scripted grabber"
echo "========================================"
echo ""
echo "  suspended branch"
echo "  softer beam physics"
echo "  kinematic external handle"
echo "  spring D6 excluded from articulation"
echo "  scripted pull + release"
echo ""

"$HOME/isaacsim/python.sh" \
"$SCRIPT_DIR/run_scripted_grabber.py"
