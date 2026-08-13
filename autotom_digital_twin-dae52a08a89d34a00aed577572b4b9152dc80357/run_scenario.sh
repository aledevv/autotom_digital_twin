#!/bin/bash
# run_scenario.sh
#
# Runs a specific CSV test scenario in Isaac Sim.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAACSIM_DIR="$HOME/isaacsim"
LOAD_SCRIPT="$SCRIPT_DIR/src/experiments/articulation_subbranch/load_from_csv_generalized_articulation.py"

if [ -z "$1" ]; then
    echo "Usage: ./run_scenario.sh <path_to_csv>"
    echo "Example: ./run_scenario.sh tests/csv_scenarios/scenario_2_huge_tree.csv"
    exit 1
fi

CSV_FILE="$(realpath "$1")"

echo "=== Loading Scenario $CSV_FILE in Isaac Sim ==="
cd "$ISAACSIM_DIR"
./python.sh "$LOAD_SCRIPT" "$CSV_FILE"
