#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAACSIM_DIR="$HOME/isaacsim"
MAIN_V2="$SCRIPT_DIR/src/exporterV2/main.py"

# Parse command line arguments
DAY=""
OPTIMIZE=""
while [[ $# -gt 0 ]]; do
  case $1 in
    --day)
      DAY="$2"
      shift 2
      ;;
    --optimize)
      OPTIMIZE="--optimize"
      shift
      ;;
    -h|--help)
      echo "Usage: $0 [--day N] [--optimize]"
      echo ""
      echo "Options:"
      echo "  --day N       Load plant from CSV for day N"
      echo "  --optimize    Apply joint-budget optimization"
      echo "  -h, --help    Show this help message"
      exit 0
      ;;
    *)
      echo "Unknown option: $1"
      echo "Usage: $0 [--day N] [--optimize]"
      exit 1
      ;;
  esac
done

# Build command with optional flags
CMD="$ISAACSIM_DIR/python.sh $MAIN_V2"
if [ -n "$DAY" ]; then
  CMD="$CMD --day $DAY"
fi
if [ -n "$OPTIMIZE" ]; then
  CMD="$CMD --optimize"
fi

# Run with appropriate message
if [ -n "$DAY" ]; then
  if [ -n "$OPTIMIZE" ]; then
    echo "=== Loading ExporterV2 from CSV (day $DAY) with optimization ==="
  else
    echo "=== Loading ExporterV2 from CSV (day $DAY) ==="
  fi
else
  if [ -n "$OPTIMIZE" ]; then
    echo "=== Loading ExporterV2 from static config with optimization ==="
  else
    echo "=== Loading ExporterV2 from static config ==="
  fi
  echo "=== Configuration: BRANCHES in src/exporterV2/tree_config.py ==="
fi

# Execute
eval $CMD