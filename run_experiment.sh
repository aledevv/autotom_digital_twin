#!/bin/bash
# run_experiment.sh
#
# Generic script to run any experiment in Isaac Sim.
#
# Usage:
#   ./run_experiment.sh <experiment_name> <script_path>
#
# Example:
#   ./run_experiment.sh recursive_tree tests/test_interactive_gui.py
#   ./run_experiment.sh cantilever_test run_cantilever.py
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAACSIM_DIR="$HOME/isaacsim"

# Check arguments
if [ $# -eq 0 ]; then
    echo "Error: No arguments provided"
    echo "Usage: ./run_experiment.sh <experiment_name> <script_path>"
    echo ""
    echo "Examples:"
    echo "  ./run_experiment.sh recursive_tree tests/test_interactive_gui.py"
    echo "  ./run_experiment.sh recursive_tree generate_recursive_tree_usda.py"
    echo "  ./run_experiment.sh cantilever_test run_cantilever.py"
    exit 1
fi

EXPERIMENT_NAME="$1"
SCRIPT_PATH="$2"

# If only one argument, assume it's a full path
if [ -z "$SCRIPT_PATH" ]; then
    FULL_SCRIPT_PATH="$SCRIPT_DIR/$EXPERIMENT_NAME"
else
    FULL_SCRIPT_PATH="$SCRIPT_DIR/src/experiments/$EXPERIMENT_NAME/$SCRIPT_PATH"
fi

# Check if script exists
if [ ! -f "$FULL_SCRIPT_PATH" ]; then
    echo "Error: Script not found: $FULL_SCRIPT_PATH"
    exit 1
fi

echo "=== Loading in Isaac Sim ==="
echo "Experiment: $EXPERIMENT_NAME"
echo "Script: $FULL_SCRIPT_PATH"
echo ""

cd "$ISAACSIM_DIR"
./python.sh "$FULL_SCRIPT_PATH"
