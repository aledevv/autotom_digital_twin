#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAACSIM_DIR="$HOME/isaacsim"
MAIN_V2="$SCRIPT_DIR/src/plant_model/main_builder.py"

echo "=== Loading Stem Builder in Isaac Sim ==="
"$ISAACSIM_DIR/python.sh" "$MAIN_V2" --day 10 --plant 1