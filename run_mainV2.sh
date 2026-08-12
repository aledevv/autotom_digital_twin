#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAACSIM_DIR="$HOME/isaacsim"
MAIN_V2="$SCRIPT_DIR/src/exporterV2/main.py"

# Parse command line arguments
DAY=""
OPTIMIZE=""
HEADLESS=""
MAX_STEPS=""
DETACHMENT_DEBUG=""
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
    --headless)
      HEADLESS="--headless"
      shift
      ;;
    --max-steps)
      MAX_STEPS="$2"
      shift 2
      ;;
    --detachment-debug)
      DETACHMENT_DEBUG="--detachment-debug"
      shift
      ;;
    -h|--help)
      echo "Usage: $0 [--day N] [--optimize] [--headless] [--max-steps N] [--detachment-debug]"
      echo ""
      echo "Options:"
      echo "  --day N       Load plant from CSV for day N"
      echo "  --optimize    Apply joint-budget optimization"
      echo "  --headless    Run without a viewport"
      echo "  --max-steps N Stop after N physics steps"
      echo "  --detachment-debug Print tomato force/torque diagnostics"
      echo "  -h, --help    Show this help message"
      exit 0
      ;;
    *)
      echo "Unknown option: $1"
      echo "Usage: $0 [--day N] [--optimize] [--headless] [--max-steps N] [--detachment-debug]"
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
if [ -n "$HEADLESS" ]; then
  CMD="$CMD --headless"
fi
if [ -n "$MAX_STEPS" ]; then
  CMD="$CMD --max-steps $MAX_STEPS"
fi
if [ -n "$DETACHMENT_DEBUG" ]; then
  CMD="$CMD --detachment-debug"
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
