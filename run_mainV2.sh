#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAACSIM_DIR="$HOME/isaacsim"
MAIN_V2="$SCRIPT_DIR/src/exporterV2/main.py"

# Parse command line arguments
DAY=""
PLANT_ID=""
OPTIMIZE=""
BRANCH_BACKEND="legacy"
SKINNING_PROFILE=""
SKINNING_NO_SYNC=""
SKINNING_SYNC_EVERY=""
SKINNING_PROFILE_WINDOW=""

usage() {
  echo "Usage: $0 [--day N] [--plant-id N] [--optimize] [--branch-backend legacy|skinned] [skinning diagnostics]"
  echo ""
  echo "Options:"
  echo "  --day N                       Load plant from CSV for day N"
  echo "  --plant-id N                  Plant ID (default in main.py: 1)"
  echo "  --optimize                    Apply joint-budget optimization"
  echo "  --branch-backend MODE         Vegetative backend: legacy|skinned (default: legacy)"
  echo "  --skinning-profile            Print detailed skinning performance diagnostics"
  echo "  --skinning-no-sync            Keep skinned meshes but skip runtime SkelAnimation sync"
  echo "  --skinning-sync-every N       Update skinning every N simulation frames"
  echo "  --skinning-profile-window N   Frames per performance report (default: 240)"
  echo "  -h, --help                    Show this help message"
}

while [[ $# -gt 0 ]]; do
  case $1 in
    --day)
      if [[ $# -lt 2 ]]; then
        echo "Missing value for --day"
        usage
        exit 1
      fi
      DAY="$2"
      shift 2
      ;;
    --plant-id)
      if [[ $# -lt 2 ]]; then
        echo "Missing value for --plant-id"
        usage
        exit 1
      fi
      PLANT_ID="$2"
      shift 2
      ;;
    --optimize)
      OPTIMIZE="--optimize"
      shift
      ;;
    --branch-backend)
      if [[ $# -lt 2 ]]; then
        echo "Missing value for --branch-backend"
        usage
        exit 1
      fi
      BRANCH_BACKEND="$2"
      if [[ "$BRANCH_BACKEND" != "legacy" && "$BRANCH_BACKEND" != "skinned" ]]; then
        echo "Invalid branch backend: $BRANCH_BACKEND"
        usage
        exit 1
      fi
      shift 2
      ;;
    --skinning-profile)
      SKINNING_PROFILE="--skinning-profile"
      shift
      ;;
    --skinning-no-sync)
      SKINNING_NO_SYNC="--skinning-no-sync"
      shift
      ;;
    --skinning-sync-every)
      if [[ $# -lt 2 ]]; then
        echo "Missing value for --skinning-sync-every"
        usage
        exit 1
      fi
      SKINNING_SYNC_EVERY="$2"
      shift 2
      ;;
    --skinning-profile-window)
      if [[ $# -lt 2 ]]; then
        echo "Missing value for --skinning-profile-window"
        usage
        exit 1
      fi
      SKINNING_PROFILE_WINDOW="$2"
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

# Build command with optional flags
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
if [ -n "$SKINNING_PROFILE" ]; then
  CMD+=("--skinning-profile")
fi
if [ -n "$SKINNING_NO_SYNC" ]; then
  CMD+=("--skinning-no-sync")
fi
if [ -n "$SKINNING_SYNC_EVERY" ]; then
  CMD+=("--skinning-sync-every" "$SKINNING_SYNC_EVERY")
fi
if [ -n "$SKINNING_PROFILE_WINDOW" ]; then
  CMD+=("--skinning-profile-window" "$SKINNING_PROFILE_WINDOW")
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
  echo "=== Configuration: BRANCHES in src/exporterV2/core/tree_config.py ==="
fi

echo "=== Vegetative branch backend: $BRANCH_BACKEND ==="
if [ -n "$SKINNING_PROFILE" ]; then
  echo "=== Skinning profiler: enabled ==="
fi
if [ -n "$SKINNING_NO_SYNC" ]; then
  echo "=== Skinning runtime sync: DISABLED (diagnostic mode) ==="
elif [ -n "$SKINNING_SYNC_EVERY" ]; then
  echo "=== Skinning runtime sync: every $SKINNING_SYNC_EVERY frame(s) ==="
fi

# Execute
"${CMD[@]}"
