#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAACSIM_DIR="$HOME/isaacsim"
MAIN_V2="$SCRIPT_DIR/src/exporterV2/load_tree.py"

echo "=== Loading ExporterV2 Tree in Isaac Sim ==="
echo "=== Configuration: BRANCHES in src/exporterV2/tree_config.py ==="
"$ISAACSIM_DIR/python.sh" "$MAIN_V2"