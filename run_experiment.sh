#!/bin/bash
# run_load_subbranch.sh
#
# Loads the articulation USD file with subbranches in Isaac Sim.
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAACSIM_DIR="$HOME/isaacsim"
LOAD_SCRIPT="$SCRIPT_DIR/src/experiments/articulation_subbranch/load_articulation_subbranch.py"

echo "=== Loading in Isaac Sim ==="
cd "$ISAACSIM_DIR"
./python.sh "$LOAD_SCRIPT"
