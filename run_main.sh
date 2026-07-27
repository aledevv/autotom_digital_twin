#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAACSIM_DIR="$HOME/isaacsim"
MAIN_V1="$SCRIPT_DIR/src/plant_model/main.py"

echo "=== Loading Stem V1 in Isaac Sim ==="
"$ISAACSIM_DIR/python.sh" "$MAIN_V1" --day 20 --plant 1