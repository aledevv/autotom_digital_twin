#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FROM_DAY="1"
FROM_DAY_SET="false"
TO_DAY=""
DAY=""
PLANT_ID="1"
OUTPUT_DIR="$SCRIPT_DIR/data/plant_states"
OVERWRITE="false"
SKIP_EXISTING="false"
API_URL="http://localhost:58081/api/"

usage() {
  echo "Usage:"
  echo "  $0 --day N [--plant-id N] [--output-dir PATH] [--overwrite]"
  echo "  $0 --from-day N --to-day N [--plant-id N] [--output-dir PATH] [--overwrite|--skip-existing]"
  echo "Examples:"
  echo "  $0 --day 50"
  echo "  $0 --from-day 1 --to-day 160 --skip-existing"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --day) DAY="${2:?Missing value for --day}"; shift 2 ;;
    --from-day) FROM_DAY="${2:?Missing value for --from-day}"; FROM_DAY_SET="true"; shift 2 ;;
    --to-day) TO_DAY="${2:?Missing value for --to-day}"; shift 2 ;;
    --plant-id) PLANT_ID="${2:?Missing value for --plant-id}"; shift 2 ;;
    --output-dir) OUTPUT_DIR="${2:?Missing value for --output-dir}"; shift 2 ;;
    --overwrite) OVERWRITE="true"; shift ;;
    --skip-existing) SKIP_EXISTING="true"; shift ;;
    --api-url) API_URL="${2:?Missing value for --api-url}"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage; exit 2 ;;
  esac
done

if [[ -n "$DAY" && ( -n "$TO_DAY" || "$FROM_DAY_SET" == "true" ) ]]; then
  echo "--day cannot be combined with --from-day or --to-day" >&2
  exit 2
fi
if [[ -z "$DAY" && -z "$TO_DAY" ]]; then
  echo "Provide --day N or --to-day N" >&2
  exit 2
fi
if [[ "$OVERWRITE" == "true" && "$SKIP_EXISTING" == "true" ]]; then
  echo "--overwrite and --skip-existing are mutually exclusive" >&2
  exit 2
fi
VALUES=("$PLANT_ID")
[[ -z "$DAY" ]] || VALUES+=("$DAY")
[[ -z "$TO_DAY" ]] || VALUES+=("$FROM_DAY" "$TO_DAY")
for value in "${VALUES[@]}"; do
  if [[ ! "$value" =~ ^[1-9][0-9]*$ ]]; then
    echo "Days and plant ID must be positive integers" >&2
    exit 2
  fi
done
if [[ -n "$TO_DAY" ]] && (( FROM_DAY > TO_DAY )); then
  echo "--from-day must be <= --to-day" >&2
  exit 2
fi

if ! curl --max-time 5 --silent --show-error --fail \
  -X POST "${API_URL%/}/app/ui/commands/app/listWB" >/dev/null; then
  echo "GroIMP API is not available at $API_URL." >&2
  echo "Start it with ./server.sh, then rerun this command." >&2
  exit 1
fi

ARGS=(
  uv run python -m groimp_bridge.batch_extractor
  --project "$SCRIPT_DIR/model/project_bridge.gsz"
  --plant-id "$PLANT_ID"
  --api-url "$API_URL"
  --output-dir "$OUTPUT_DIR"
)
if [[ -n "$DAY" ]]; then
  ARGS+=(--day "$DAY")
else
  ARGS+=(--from-day "$FROM_DAY" --to-day "$TO_DAY")
fi
[[ "$OVERWRITE" == "false" ]] || ARGS+=(--overwrite)
[[ "$SKIP_EXISTING" == "false" ]] || ARGS+=(--skip-existing)

cd "$SCRIPT_DIR"
exec "${ARGS[@]}"
