#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAACSIM_DIR="${ISAACSIM_DIR:-$HOME/isaacsim}"
DAY=""
PLANT_ID="1"
INPUT=""
OUTPUT=""
GENERATE_ONLY="false"
HEADLESS="false"
DURATION="5"
PHYSICS_PRESET="locked"
OPTIMIZE="false"
ALLOW_NEAR_BUDGET="false"
STIFFNESS_SCALE="1"
LEAF_STIFFNESS_SCALE="1"
TRUSS_STIFFNESS_SCALE="1"
PHYSICS_HZ="480"
BRANCH_BACKEND="skinned"
SKINNING_VISUAL_MODE="segmented"
DEBUG_PROFILE="full"
POSE_MODE="canonical"
DEBUG_NO_COLLIDERS="false"
DEBUG_NO_DRIVES="false"
DEBUG_NO_ARTICULATION="false"

usage() {
  echo "Usage: $0 [--day N] [options]"
  echo
  echo "With --day: generate V2 from plant_state/1.0 and open it in Isaac Sim."
  echo "Without --day: retain the BRANCHES static demo from tree_config.py."
  echo
  echo "  --plant-id N"
  echo "  --input PATH                 Canonical JSON (no CSV fallback)"
  echo "  --output PATH                Generated USDA"
  echo "  --generate-only              Generate/audit without Isaac Sim"
  echo "  --headless                   Run the finite physics validation"
  echo "  --duration SECONDS           Headless simulated duration (default: 5)"
  echo "  --physics-preset MODE        locked|flexible (default: locked until flexible validation passes)"
  echo "  --optimize                   Optimize physics only when over budget"
  echo "  --allow-near-budget          Permit 221-230 reviewed joints"
  echo "  --stiffness-scale N          1|2|4"
  echo "  --leaf-stiffness-scale N     1|0.5|0.25|0.1"
  echo "  --truss-stiffness-scale N    1|0.5|0.25|0.1"
  echo "  --physics-hz N               480|960"
  echo "  --debug-profile PROFILE      full|stem|leaf-supports|leaves|laterals|truss-supports|fruit-visual"
  echo "  --pose-mode MODE             canonical|legacy (PlantState, default: canonical)"
  echo "  --debug-no-colliders         Diagnostic profiles only"
  echo "  --debug-no-drives            Flexible diagnostic profiles only"
  echo "  --debug-no-articulation      Diagnostic profiles only"
  echo "  --branch-backend MODE        Static demo only: legacy|skinned"
  echo "  --skinning-visual-mode MODE  Static demo only"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --day) DAY="${2:?Missing value for --day}"; shift 2 ;;
    --plant-id) PLANT_ID="${2:?Missing value for --plant-id}"; shift 2 ;;
    --input) INPUT="${2:?Missing value for --input}"; shift 2 ;;
    --output) OUTPUT="${2:?Missing value for --output}"; shift 2 ;;
    --generate-only) GENERATE_ONLY="true"; shift ;;
    --headless) HEADLESS="true"; shift ;;
    --duration) DURATION="${2:?Missing value for --duration}"; shift 2 ;;
    --physics-preset) PHYSICS_PRESET="${2:?Missing value for --physics-preset}"; shift 2 ;;
    --optimize) OPTIMIZE="true"; shift ;;
    --allow-near-budget) ALLOW_NEAR_BUDGET="true"; shift ;;
    --stiffness-scale) STIFFNESS_SCALE="${2:?Missing value for --stiffness-scale}"; shift 2 ;;
    --leaf-stiffness-scale) LEAF_STIFFNESS_SCALE="${2:?Missing value for --leaf-stiffness-scale}"; shift 2 ;;
    --truss-stiffness-scale) TRUSS_STIFFNESS_SCALE="${2:?Missing value for --truss-stiffness-scale}"; shift 2 ;;
    --physics-hz) PHYSICS_HZ="${2:?Missing value for --physics-hz}"; shift 2 ;;
    --debug-profile) DEBUG_PROFILE="${2:?Missing value for --debug-profile}"; shift 2 ;;
    --pose-mode) POSE_MODE="${2:?Missing value for --pose-mode}"; shift 2 ;;
    --debug-no-colliders) DEBUG_NO_COLLIDERS="true"; shift ;;
    --debug-no-drives) DEBUG_NO_DRIVES="true"; shift ;;
    --debug-no-articulation) DEBUG_NO_ARTICULATION="true"; shift ;;
    --branch-backend) BRANCH_BACKEND="${2:?Missing value for --branch-backend}"; shift 2 ;;
    --skinning-visual-mode) SKINNING_VISUAL_MODE="${2:?Missing value for --skinning-visual-mode}"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage; exit 2 ;;
  esac
done

if [[ -z "$DAY" ]]; then
  if [[ -n "$INPUT" || -n "$OUTPUT" || "$GENERATE_ONLY" == "true" || "$HEADLESS" == "true" ]]; then
    echo "--input, --output, --generate-only and --headless require --day" >&2
    exit 2
  fi
  LEGACY_ARGS=(--branch-backend "$BRANCH_BACKEND" --skinning-visual-mode "$SKINNING_VISUAL_MODE")
  [[ "$OPTIMIZE" == "false" ]] || LEGACY_ARGS+=(--optimize)
  exec "$ISAACSIM_DIR/python.sh" "$SCRIPT_DIR/src/exporterV2/main.py" "${LEGACY_ARGS[@]}"
fi

if [[ ! "$DAY" =~ ^[1-9][0-9]*$ ]] || [[ ! "$PLANT_ID" =~ ^[1-9][0-9]*$ ]]; then
  echo "--day and --plant-id must be positive integers" >&2
  exit 2
fi
if [[ "$PHYSICS_PRESET" != "locked" && "$PHYSICS_PRESET" != "flexible" ]]; then
  echo "--physics-preset must be locked or flexible" >&2
  exit 2
fi
if [[ "$STIFFNESS_SCALE" != "1" && "$STIFFNESS_SCALE" != "2" && "$STIFFNESS_SCALE" != "4" ]]; then
  echo "--stiffness-scale must be 1, 2, or 4" >&2
  exit 2
fi
if [[ "$PHYSICS_HZ" != "480" && "$PHYSICS_HZ" != "960" ]]; then
  echo "--physics-hz must be 480 or 960" >&2
  exit 2
fi
case "$LEAF_STIFFNESS_SCALE" in 1|0.5|0.25|0.1) ;; *) echo "Invalid --leaf-stiffness-scale" >&2; exit 2 ;; esac
case "$TRUSS_STIFFNESS_SCALE" in 1|0.5|0.25|0.1) ;; *) echo "Invalid --truss-stiffness-scale" >&2; exit 2 ;; esac
case "$DEBUG_PROFILE" in
  full|stem|leaf-supports|leaves|laterals|truss-supports|fruit-visual) ;;
  *) echo "Invalid --debug-profile: $DEBUG_PROFILE" >&2; exit 2 ;;
esac
case "$POSE_MODE" in
  canonical|legacy) ;;
  *) echo "Invalid --pose-mode: $POSE_MODE" >&2; exit 2 ;;
esac
if [[ "$DEBUG_PROFILE" == "full" && ( "$DEBUG_NO_COLLIDERS" == "true" || "$DEBUG_NO_DRIVES" == "true" || "$DEBUG_NO_ARTICULATION" == "true" ) ]]; then
  echo "Diagnostic physics switches require a non-full --debug-profile" >&2
  exit 2
fi

SUFFIX=""
[[ "$PLANT_ID" == "1" ]] || SUFFIX="_plant_${PLANT_ID}"
[[ -n "$INPUT" ]] || INPUT="$SCRIPT_DIR/data/plant_states/plant_state_day_${DAY}${SUFFIX}.json"
if [[ -z "$OUTPUT" ]]; then
  if [[ "$DEBUG_PROFILE" == "full" ]]; then
    OUTPUT="$SCRIPT_DIR/data/usd_models/tree_v2_day_${DAY}${SUFFIX}.usda"
  else
    OUTPUT="/tmp/autotom-phase-j-debug/day_${DAY}/tree_v2_day_${DAY}${SUFFIX}_${DEBUG_PROFILE}.usda"
  fi
fi
if [[ ! -f "$INPUT" ]]; then
  echo "PlantState input does not exist: $INPUT" >&2
  echo "There is intentionally no CSV fallback." >&2
  exit 2
fi

GENERATOR=(uv run python -m exporterV2 --day "$DAY" --plant-id "$PLANT_ID" --input "$INPUT" --output "$OUTPUT" --physics-preset "$PHYSICS_PRESET" --stiffness-scale "$STIFFNESS_SCALE" --leaf-stiffness-scale "$LEAF_STIFFNESS_SCALE" --truss-stiffness-scale "$TRUSS_STIFFNESS_SCALE" --physics-hz "$PHYSICS_HZ" --debug-profile "$DEBUG_PROFILE" --pose-mode "$POSE_MODE")
[[ "$OPTIMIZE" == "false" ]] || GENERATOR+=(--optimize)
[[ "$ALLOW_NEAR_BUDGET" == "false" ]] || GENERATOR+=(--allow-near-budget)
[[ "$DEBUG_NO_COLLIDERS" == "false" ]] || GENERATOR+=(--debug-no-colliders)
[[ "$DEBUG_NO_DRIVES" == "false" ]] || GENERATOR+=(--debug-no-drives)
[[ "$DEBUG_NO_ARTICULATION" == "false" ]] || GENERATOR+=(--debug-no-articulation)
(
  cd "$SCRIPT_DIR"
  "${GENERATOR[@]}"
)

[[ "$GENERATE_ONLY" == "false" ]] || exit 0
ISAAC_PYTHON="$ISAACSIM_DIR/python.sh"
if [[ ! -x "$ISAAC_PYTHON" ]]; then
  echo "Isaac Sim launcher not found or not executable: $ISAAC_PYTHON" >&2
  exit 2
fi
ISAAC_ARGS=(--usd "$OUTPUT" --duration "$DURATION" --physics-preset "$PHYSICS_PRESET" --physics-hz "$PHYSICS_HZ")
[[ "$DEBUG_PROFILE" == "full" ]] || ISAAC_ARGS+=(--diagnostic-monitor)
[[ "$HEADLESS" == "false" ]] || ISAAC_ARGS+=(--headless)
exec "$ISAAC_PYTHON" "$SCRIPT_DIR/src/exporterV2/isaac_app.py" "${ISAAC_ARGS[@]}"
