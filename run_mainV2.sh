#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAACSIM_DIR="$HOME/isaacsim"
MAIN_V2="$SCRIPT_DIR/src/exporterV2/main.py"

# Parse command line arguments
DAY=""
while [[ $# -gt 0 ]]; do
  case $1 in
    --day)
      DAY="$2"
      shift 2
      ;;
    *)
      echo "Unknown option: $1"
      echo "Usage: $0 [--day N]"
      exit 1
      ;;
  esac
done

# Run with or without --day flag
if [ -n "$DAY" ]; then
  echo "=== Loading ExporterV2 from CSV (day $DAY) ==="
  "$ISAACSIM_DIR/python.sh" "$MAIN_V2" --day "$DAY"
else
  echo "=== Loading ExporterV2 from static config ==="
  echo "=== Configuration: BRANCHES in src/exporterV2/tree_config.py ==="
  "$ISAACSIM_DIR/python.sh" "$MAIN_V2"
fi