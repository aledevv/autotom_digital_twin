#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAACSIM_DIR="$HOME/isaacsim"
MAIN_V2="$SCRIPT_DIR/src/plant_model/mainV2.py"

echo "=== Loading Stem V2 in Isaac Sim ==="
"$ISAACSIM_DIR/python.sh" "$MAIN_V2" --day 1 --plant 1