#!/bin/bash
# run_builder_tests.sh
#
# Runs the PlantBuilder unit-test suite using Isaac Sim's Python
# (which has pxr / OpenUSD).
#
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAACSIM_DIR="$HOME/isaacsim"
TEST_SCRIPT="$SCRIPT_DIR/tests/plant_builder/run_tests.py"

echo "=== Running PlantBuilder tests ==="
cd "$ISAACSIM_DIR"
./python.sh "$TEST_SCRIPT"
