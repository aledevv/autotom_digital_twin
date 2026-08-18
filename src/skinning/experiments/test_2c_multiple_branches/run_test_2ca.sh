#!/bin/bash
# Test 2C-A — Single branch junction
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "========================================"
echo "  Test 2C-A — Single branch junction"
echo "========================================"
echo ""
echo "  main stem + one lateral branch"
echo "  one branching PhysX articulation"
echo "  separate skinned meshes"
echo ""

"$HOME/isaacsim/python.sh" "$SCRIPT_DIR/run_branch_junction.py"
