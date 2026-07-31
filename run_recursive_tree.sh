#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=== Recursive Tree Articulation Test ==="
echo "Generating USD stage..."
env -i HOME="$HOME" PATH="$PATH" uv run "$SCRIPT_DIR/src/experiments/recursive_tree/generate_recursive_tree_usda.py"

echo "Launching Isaac Sim..."
~/isaacsim/python.sh "$SCRIPT_DIR/src/experiments/recursive_tree/load_recursive_tree.py"
