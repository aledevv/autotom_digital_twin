#!/bin/bash
# run_experiment.sh
#
# Runs PlantBuilder visual tests in Isaac Sim.
# Usage:
#   ./run_experiment.sh       # defaults to test 1
#   ./run_experiment.sh 1     # trunk only
#   ./run_experiment.sh 2     # trunk + branch
#   ./run_experiment.sh 3     # branch with extensions
#   ./run_experiment.sh 4     # subbranch
#   ./run_experiment.sh 5     # full tree
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAACSIM_DIR="$HOME/isaacsim"
TEST_SCRIPT="$SCRIPT_DIR/tests/plant_builder/visual_tests.py"
TEST_NUM="${1:-1}"

echo "=== PlantBuilder Visual Test $TEST_NUM ==="
cd "$ISAACSIM_DIR"
./python.sh "$TEST_SCRIPT" "$TEST_NUM"
