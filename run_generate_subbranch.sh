#!/bin/bash
# run_generate_subbranch.sh
#
# Generates the articulation USD file with subbranches using the local uv environment.
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GENERATE_SCRIPT="src/experiments/articulation_subbranch/generate_articulation_usda.py"

echo "=== USD Generation via uv ==="
cd "$SCRIPT_DIR"
uv run python "$GENERATE_SCRIPT"
echo "Generation completed."

