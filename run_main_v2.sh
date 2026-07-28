#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAACSIM_DIR="$HOME/isaacsim"
MAIN_V2="$SCRIPT_DIR/src/plant_model/v2/main_v2.py"

echo "=== Running Plant v2 Exporter ==="
"$ISAACSIM_DIR/python.sh" "$MAIN_V2" --day 20 --plant 1