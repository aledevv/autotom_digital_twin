#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"

echo "Running Three-Point Bending Analytical Verification..."
python "$SCRIPT_DIR/verify_experiment.py"

echo "Running Three-Point Bending Test in Isaac Sim..."
"$HOME/isaacsim/python.sh" "$SCRIPT_DIR/run_threepoint.py"

echo "Generating Three-Point Bending Plot..."
cd "$PROJECT_ROOT"
uv run "$SCRIPT_DIR/plot_threepoint.py"
