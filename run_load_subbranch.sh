#!/bin/bash
# run_load_subbranch.sh
#
# Carica il file USD dell'articolazione con subbranch in Isaac Sim.
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAACSIM_DIR="$HOME/isaacsim"
LOAD_SCRIPT="$SCRIPT_DIR/src/experiments/articulation_subbranch/load_articulation_subbranch.py"

echo "=== Caricamento in Isaac Sim ==="
cd "$ISAACSIM_DIR"
./python.sh "$LOAD_SCRIPT"
