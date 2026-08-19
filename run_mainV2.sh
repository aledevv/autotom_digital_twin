#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAACSIM_DIR="$HOME/isaacsim"
MAIN_V2="$SCRIPT_DIR/src/exporterV2/main.py"

DAY=""
PLANT_ID=""
OPTIMIZE=""
BRANCH_BACKEND="legacy"
SKINNING_VISUAL_MODE="skinned"

usage() {
  echo "Usage: $0 [--day N] [--plant-id N] [--optimize] [--branch-backend legacy|skinned] [--skinning-visual-mode skinned|static|rigid-single|global|segmented]"
  echo ""
  echo "Options:"
  echo "  --day N                       Load plant from CSV for day N"
  echo "  --plant-id N                  Plant ID (default in main.py: 1)"
  echo "  --optimize                    Apply joint-budget optimization"
  echo "  --branch-backend MODE         Vegetative backend: legacy|skinned (default: legacy)"
  echo "  --skinning-visual-mode MODE   skinned | static | rigid-single | global | segmented"
  echo "  -h, --help                    Show this help message"
}

while [[ $# -gt 0 ]]; do
  case $1 in
    --day)
      [[ $# -ge 2 ]] || { echo "Missing value for --day"; usage; exit 1; }
      DAY="$2"
      shift 2
      ;;
    --plant-id)
      [[ $# -ge 2 ]] || { echo "Missing value for --plant-id"; usage; exit 1; }
      PLANT_ID="$2"
      shift 2
      ;;
    --optimize)
      OPTIMIZE="--optimize"
      shift
      ;;
    --branch-backend)
      [[ $# -ge 2 ]] || { echo "Missing value for --branch-backend"; usage; exit 1; }
      BRANCH_BACKEND="$2"
      if [[ "$BRANCH_BACKEND" != "legacy" && "$BRANCH_BACKEND" != "skinned" ]]; then
        echo "Invalid branch backend: $BRANCH_BACKEND"
        usage
        exit 1
      fi
      shift 2
      ;;
    --skinning-visual-mode)
      [[ $# -ge 2 ]] || { echo "Missing value for --skinning-visual-mode"; usage; exit 1; }
      SKINNING_VISUAL_MODE="$2"
      if [[ "$SKINNING_VISUAL_MODE" != "skinned" && "$SKINNING_VISUAL_MODE" != "static" && "$SKINNING_VISUAL_MODE" != "rigid-single" && "$SKINNING_VISUAL_MODE" != "global" && "$SKINNING_VISUAL_MODE" != "segmented" ]]; then
        echo "Invalid skinning visual mode: $SKINNING_VISUAL_MODE"
        usage
        exit 1
      fi
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1"
      usage
      exit 1
      ;;
  esac
done

if [[ "$BRANCH_BACKEND" != "skinned" && "$SKINNING_VISUAL_MODE" != "skinned" ]]; then
  echo "--skinning-visual-mode requires --branch-backend skinned"
  exit 1
fi

CMD=("$ISAACSIM_DIR/python.sh" "$MAIN_V2" "--branch-backend" "$BRANCH_BACKEND")

if [ -n "$DAY" ]; then
  CMD+=("--day" "$DAY")
fi
if [ -n "$PLANT_ID" ]; then
  CMD+=("--plant-id" "$PLANT_ID")
fi
if [ -n "$OPTIMIZE" ]; then
  CMD+=("--optimize")
fi
if [[ "$BRANCH_BACKEND" == "skinned" ]]; then
  CMD+=("--skinning-visual-mode" "$SKINNING_VISUAL_MODE")
fi

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
  echo "=== Configuration: BRANCHES in src/exporterV2/core/tree_config.py ==="
fi

echo "=== Vegetative branch backend: $BRANCH_BACKEND ==="
if [[ "$BRANCH_BACKEND" == "skinned" ]]; then
  echo "=== Skinning visual mode: $SKINNING_VISUAL_MODE ==="
fi

"${CMD[@]}"
