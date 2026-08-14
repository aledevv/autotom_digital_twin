#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"

"$HOME/isaacsim/python.sh" "$SCRIPT_DIR/load_tomato_branch.py"

cd "$PROJECT_ROOT"
uv run "$SCRIPT_DIR/plot_forces.py"
