#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

DAY=""
ORGAN="stem"
POSE_MODE="canonical"
PHYSICS_PRESET="flexible"
HEADLESS="false"
GENERATE_ONLY="false"
DURATION="5"
PHYSICS_HZ="480"
INTERACTIVE_PHYSICS_HZ="60"
INPUT=""
OUTPUT=""

usage() {
  cat <<'EOF'
Usage: ./run_debugV2.sh --day N [--organ stem|laterals|leaf-supports|leaves] [options]

Run one incremental ExporterV2 organ checkpoint from PlantState.
Each checkpoint is cumulative: leaves includes the validated fixed stem,
native laterals, dynamic petioles, fixed rachides, and rigid leaf visuals.

Options:
  --day N                    PlantState simulation day (required)
  --organ NAME               Organ checkpoint: stem|laterals|leaf-supports|leaves
  --pose-mode MODE           canonical|legacy (default: canonical)
  --physics-preset MODE      flexible|locked (default: flexible)
  --headless                 Run the finite Isaac stability test
  --duration SECONDS         Simulated headless duration (default: 5)
  --physics-hz N             Headless/authoring: 480|960 (default: 480)
  --interactive-physics-hz N GUI only: 60|120|240|480 (default: 60)
  --generate-only            Generate and audit USDA without Isaac Sim
  --input PATH               Override PlantState JSON path
  --output PATH              Override generated USDA path
  -h, --help                 Show this help

Examples:
  ./run_debugV2.sh --day 10 --organ stem
  ./run_debugV2.sh --day 50 --organ laterals --headless --duration 1
  ./run_debugV2.sh --day 50 --organ laterals
  ./run_debugV2.sh --day 50 --organ leaf-supports --headless --duration 1
  ./run_debugV2.sh --day 50 --organ leaf-supports
  ./run_debugV2.sh --day 50 --organ leaves
  ./run_debugV2.sh --day 25 --organ stem --generate-only
  ./run_debugV2.sh --day 10 --organ stem --pose-mode legacy
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --day) DAY="${2:?Missing value for --day}"; shift 2 ;;
    --organ) ORGAN="${2:?Missing value for --organ}"; shift 2 ;;
    --pose-mode) POSE_MODE="${2:?Missing value for --pose-mode}"; shift 2 ;;
    --physics-preset) PHYSICS_PRESET="${2:?Missing value for --physics-preset}"; shift 2 ;;
    --headless) HEADLESS="true"; shift ;;
    --duration) DURATION="${2:?Missing value for --duration}"; shift 2 ;;
    --physics-hz) PHYSICS_HZ="${2:?Missing value for --physics-hz}"; shift 2 ;;
    --interactive-physics-hz) INTERACTIVE_PHYSICS_HZ="${2:?Missing value for --interactive-physics-hz}"; shift 2 ;;
    --generate-only) GENERATE_ONLY="true"; shift ;;
    --input) INPUT="${2:?Missing value for --input}"; shift 2 ;;
    --output) OUTPUT="${2:?Missing value for --output}"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ -z "$DAY" ]]; then
  echo "--day is required" >&2
  usage >&2
  exit 2
fi
if [[ ! "$DAY" =~ ^[1-9][0-9]*$ ]]; then
  echo "--day must be a positive integer" >&2
  exit 2
fi
case "$ORGAN" in
  stem|laterals|leaf-supports|leaves) ;;
  *) echo "Organ checkpoint '$ORGAN' is not implemented yet; available: stem, laterals, leaf-supports, leaves" >&2; exit 2 ;;
esac
if [[ "$POSE_MODE" != "canonical" && "$POSE_MODE" != "legacy" ]]; then
  echo "--pose-mode must be canonical or legacy" >&2
  exit 2
fi
if [[ "$PHYSICS_PRESET" != "flexible" && "$PHYSICS_PRESET" != "locked" ]]; then
  echo "--physics-preset must be flexible or locked" >&2
  exit 2
fi
if [[ "$HEADLESS" == "true" && "$GENERATE_ONLY" == "true" ]]; then
  echo "--headless and --generate-only are mutually exclusive" >&2
  exit 2
fi

COMMAND=(
  "$SCRIPT_DIR/run_mainV2.sh"
  --day "$DAY"
  --debug-profile "$ORGAN"
  --pose-mode "$POSE_MODE"
  --physics-preset "$PHYSICS_PRESET"
  --duration "$DURATION"
  --physics-hz "$PHYSICS_HZ"
  --interactive-physics-hz "$INTERACTIVE_PHYSICS_HZ"
)
[[ -z "$INPUT" ]] || COMMAND+=(--input "$INPUT")
[[ -z "$OUTPUT" ]] || COMMAND+=(--output "$OUTPUT")
[[ "$HEADLESS" == "false" ]] || COMMAND+=(--headless)
[[ "$GENERATE_ONLY" == "false" ]] || COMMAND+=(--generate-only)

echo "=== ExporterV2 incremental checkpoint ==="
echo "day=$DAY organ=$ORGAN pose=$POSE_MODE physics=$PHYSICS_PRESET validation_hz=$PHYSICS_HZ interactive_hz=$INTERACTIVE_PHYSICS_HZ"
exec "${COMMAND[@]}"
