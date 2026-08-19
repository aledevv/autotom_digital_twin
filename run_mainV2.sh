#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAACSIM_DIR="$HOME/isaacsim"
MAIN_V2="$SCRIPT_DIR/src/exporterV2/main.py"

# Parse command line arguments
DAY=""
OPTIMIZE=""
BRANCH_BACKEND="legacy"
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
    --branch-backend)
      BRANCH_BACKEND="$2"
      if [[ "$BRANCH_BACKEND" != "legacy" && "$BRANCH_BACKEND" != "skinned" ]]; then
        echo "Invalid branch backend: $BRANCH_BACKEND"
        exit 1
      fi
      shift 2
      ;;
    -h|--help)
      echo "Usage: $0 [--day N] [--optimize] [--branch-backend legacy|skinned]"
      echo ""
      echo "Options:"
      echo "  --day N       Load plant from CSV for day N"
      echo "  --optimize    Apply joint-budget optimization"
      echo "  --branch-backend  Vegetative backend (default: legacy)"
      echo "  -h, --help    Show this help message"
      exit 0
      ;;
    *)
      echo "Unknown option: $1"
      echo "Usage: $0 [--day N] [--optimize] [--branch-backend legacy|skinned]"
      exit 1
      ;;
  esac
done

# Build command with optional flags
CMD=("$ISAACSIM_DIR/python.sh" "$MAIN_V2" "--branch-backend" "$BRANCH_BACKEND")
if [ -n "$DAY" ]; then
  CMD+=("--day" "$DAY")
fi
if [ -n "$OPTIMIZE" ]; then
  CMD+=("--optimize")
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
echo "=== Vegetative branch backend: $BRANCH_BACKEND ==="

# Execute
"${CMD[@]}"
