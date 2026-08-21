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

usage() {
  echo "Usage: $0 --day N [--plant-id N] [--input PATH] [--output PATH] [--generate-only] [--headless]"
  echo
  echo "Generate V1 from plant_state/1.0, then open the static stage in Isaac Sim."
  echo "--generate-only generates USDA and its audit manifest without starting Isaac Sim."
  echo "--headless starts Isaac Sim for a short stage-loading smoke test."
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --day)
      [[ $# -ge 2 ]] || { echo "Missing value for --day" >&2; exit 2; }
      DAY="$2"
      shift 2
      ;;
    --plant-id)
      [[ $# -ge 2 ]] || { echo "Missing value for --plant-id" >&2; exit 2; }
      PLANT_ID="$2"
      shift 2
      ;;
    --input)
      [[ $# -ge 2 ]] || { echo "Missing value for --input" >&2; exit 2; }
      INPUT="$2"
      shift 2
      ;;
    --output)
      [[ $# -ge 2 ]] || { echo "Missing value for --output" >&2; exit 2; }
      OUTPUT="$2"
      shift 2
      ;;
    --generate-only)
      GENERATE_ONLY="true"
      shift
      ;;
    --headless)
      HEADLESS="true"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage
      exit 2
      ;;
  esac
done

if [[ -z "$DAY" ]]; then
  echo "--day is required" >&2
  usage
  exit 2
fi
if [[ ! "$DAY" =~ ^[1-9][0-9]*$ ]] || [[ ! "$PLANT_ID" =~ ^[1-9][0-9]*$ ]]; then
  echo "--day and --plant-id must be positive integers" >&2
  exit 2
fi

if [[ -z "$INPUT" ]]; then
  if [[ "$PLANT_ID" == "1" ]]; then
    INPUT="$SCRIPT_DIR/data/plant_states/plant_state_day_${DAY}.json"
  else
    INPUT="$SCRIPT_DIR/data/plant_states/plant_state_day_${DAY}_plant_${PLANT_ID}.json"
  fi
fi
if [[ -z "$OUTPUT" ]]; then
  PLANT_SUFFIX=""
  [[ "$PLANT_ID" == "1" ]] || PLANT_SUFFIX="_plant_${PLANT_ID}"
  OUTPUT="$SCRIPT_DIR/data/usd_models/tree_v1_day_${DAY}${PLANT_SUFFIX}.usda"
fi

# Fail before starting Isaac Sim and never fall back to a legacy CSV.
if [[ ! -f "$INPUT" ]]; then
  echo "PlantState input does not exist: $INPUT" >&2
  echo "Prepare it with:" >&2
  echo "  uv run python -m groimp_bridge.extractor --project model/project_bridge.gsz --steps $DAY --plant-id $PLANT_ID --output $INPUT" >&2
  exit 2
fi

GENERATOR=(uv run python -m exporterV1 --day "$DAY" --plant-id "$PLANT_ID" --input "$INPUT" --output "$OUTPUT")
(
  cd "$SCRIPT_DIR"
  "${GENERATOR[@]}"
)

if [[ "$GENERATE_ONLY" == "true" ]]; then
  exit 0
fi

ISAAC_PYTHON="$ISAACSIM_DIR/python.sh"
if [[ ! -x "$ISAAC_PYTHON" ]]; then
  echo "Isaac Sim launcher not found or not executable: $ISAAC_PYTHON" >&2
  exit 2
fi

ISAAC_ARGS=(--usd "$OUTPUT")
[[ "$HEADLESS" == "false" ]] || ISAAC_ARGS+=(--headless)
exec "$ISAAC_PYTHON" "$SCRIPT_DIR/src/exporterV1/isaac_app.py" "${ISAAC_ARGS[@]}"
