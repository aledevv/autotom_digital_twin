#!/bin/bash
# run_test_add_branch.sh
#
# Runs the add_branch test script using Isaac Sim's Python
#
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAACSIM_DIR="$HOME/isaacsim"
TEST_SCRIPT="$SCRIPT_DIR/src/experiments/test_add_branch.py"

echo "=== Running test_add_branch.py ==="
cd "$ISAACSIM_DIR"
./python.sh "$TEST_SCRIPT"
