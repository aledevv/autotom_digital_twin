#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"

echo "Running Cantilever Bending Test in Isaac Sim..."
"$HOME/isaacsim/python.sh" "$SCRIPT_DIR/run_cantilever.py"

echo "Generating Deflection Plot..."
cd "$PROJECT_ROOT"
uv run "$SCRIPT_DIR/plot_deflection.py"
