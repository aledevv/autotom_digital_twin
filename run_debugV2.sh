#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

DAY=""
ORGAN="truss-supports"
POSE_MODE="canonical"
APPENDAGE_POSE_MODE="v2-aesthetic"
PHYSICS_PRESET="flexible"
HEADLESS="false"
GENERATE_ONLY="false"
DURATION="5"
PHYSICS_HZ="480"
INTERACTIVE_PHYSICS_HZ="60"
LEAF_JOINT_POLICY="distributed"
LATERAL_JOINT_POLICY="dynamic"
TRUSS_CALIBRATION_PRESET="current"
TRUSS_DAMPING_OVERRIDE=""
TRUSS_ARMATURE_MULTIPLIER="0"
TERMINAL_SOLVER_PRESET="current"
VISUAL_QUALITY="realistic"
INITIAL_OVERLAP_POLICY="filter"
PHYSICAL_PETIOLULES="false"
ALLOW_NEAR_BUDGET="false"
ALLOW_OVER_BUDGET="false"
INPUT=""
OUTPUT=""
ALLOW_EXPERIMENTAL_FRUIT_PHYSICS="false"

usage() {
  cat <<'EOF'
Usage: ./run_debugV2.sh --day N [--organ stem|laterals|leaf-supports|leaves|truss-supports|fruit-visual|full] [options]

Run one incremental ExporterV2 organ checkpoint from PlantState.
Each checkpoint is cumulative: leaves includes the validated fixed stem,
native laterals, dynamic petioles and rachides, and rigid leaf visuals.

Options:
  --day N                    PlantState simulation day (required)
  --organ NAME               Optional diagnostic subset (default: truss-supports)
  --pose-mode MODE           canonical|legacy (default: canonical)
  --appendage-pose-mode MODE v2-aesthetic|canonical (default: v2-aesthetic)
  --physics-preset MODE      flexible|locked (default: flexible)
  --headless                 Run the finite Isaac stability test
  --duration SECONDS         Simulated headless duration (default: 5)
  --physics-hz N             Headless/authoring: 480|960 (default: 480)
  --interactive-physics-hz N GUI only: 60|120|240|480 (default: 60)
  --leaf-joint-policy MODE   optimized|distributed (default: distributed)
  --lateral-joint-policy P   dynamic|fixed (default: dynamic)
  --truss-calibration-preset current|compliant|balanced|firm
  --truss-damping-override N 1|2|4|7, applied after the preset
  --truss-armature-multiplier N 0|1|4, fallback on truss D6 only
  --terminal-solver-preset P current (32/1)|stabilized (64/4)
  --allow-experimental-fruit-physics Required with --organ full
  --visual-quality MODE      realistic|performance (default: realistic)
  --initial-overlap-policy P filter|error (default: filter)
  --physical-petiolules      EXPENSIVE: add rigid bodies/colliders/D6
  --allow-near-budget        Permit 221-230 reviewed D6 joints
  --allow-over-budget        Unsafe diagnostic override with physical petiolules
  --generate-only            Generate and audit USDA without Isaac Sim
  --input PATH               Override PlantState JSON path
  --output PATH              Override generated USDA path
  -h, --help                 Show this help

Examples:
  ./run_debugV2.sh --day 50
  ./run_debugV2.sh --day 50 --headless --duration 1
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
    --appendage-pose-mode) APPENDAGE_POSE_MODE="${2:?Missing value}"; shift 2 ;;
    --physics-preset) PHYSICS_PRESET="${2:?Missing value for --physics-preset}"; shift 2 ;;
    --headless) HEADLESS="true"; shift ;;
    --duration) DURATION="${2:?Missing value for --duration}"; shift 2 ;;
    --physics-hz) PHYSICS_HZ="${2:?Missing value for --physics-hz}"; shift 2 ;;
    --interactive-physics-hz) INTERACTIVE_PHYSICS_HZ="${2:?Missing value for --interactive-physics-hz}"; shift 2 ;;
    --leaf-joint-policy) LEAF_JOINT_POLICY="${2:?Missing value for --leaf-joint-policy}"; shift 2 ;;
    --lateral-joint-policy) LATERAL_JOINT_POLICY="${2:?Missing value}"; shift 2 ;;
    --truss-calibration-preset) TRUSS_CALIBRATION_PRESET="${2:?Missing value}"; shift 2 ;;
    --truss-damping-override) TRUSS_DAMPING_OVERRIDE="${2:?Missing value}"; shift 2 ;;
    --truss-armature-multiplier) TRUSS_ARMATURE_MULTIPLIER="${2:?Missing value}"; shift 2 ;;
    --terminal-solver-preset) TERMINAL_SOLVER_PRESET="${2:?Missing value}"; shift 2 ;;
    --allow-experimental-fruit-physics) ALLOW_EXPERIMENTAL_FRUIT_PHYSICS="true"; shift ;;
    --visual-quality) VISUAL_QUALITY="${2:?Missing value for --visual-quality}"; shift 2 ;;
    --initial-overlap-policy) INITIAL_OVERLAP_POLICY="${2:?Missing value for --initial-overlap-policy}"; shift 2 ;;
    --physical-petiolules) PHYSICAL_PETIOLULES="true"; shift ;;
    --allow-near-budget) ALLOW_NEAR_BUDGET="true"; shift ;;
    --allow-over-budget) ALLOW_OVER_BUDGET="true"; shift ;;
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
  stem|laterals|leaf-supports|leaves|truss-supports|fruit-visual|full) ;;
  *) echo "Organ checkpoint '$ORGAN' is not implemented" >&2; exit 2 ;;
esac
if [[ "$POSE_MODE" != "canonical" && "$POSE_MODE" != "legacy" ]]; then
  echo "--pose-mode must be canonical or legacy" >&2
  exit 2
fi
case "$APPENDAGE_POSE_MODE" in v2-aesthetic|canonical) ;; *) echo "Invalid --appendage-pose-mode" >&2; exit 2 ;; esac
if [[ "$PHYSICS_PRESET" != "flexible" && "$PHYSICS_PRESET" != "locked" ]]; then
  echo "--physics-preset must be flexible or locked" >&2
  exit 2
fi
if [[ "$LEAF_JOINT_POLICY" != "optimized" && "$LEAF_JOINT_POLICY" != "distributed" ]]; then
  echo "--leaf-joint-policy must be optimized or distributed" >&2
  exit 2
fi
case "$LATERAL_JOINT_POLICY" in dynamic|fixed) ;; *) echo "--lateral-joint-policy must be dynamic or fixed" >&2; exit 2 ;; esac
case "$TRUSS_CALIBRATION_PRESET" in current|compliant|balanced|firm) ;; *) echo "Invalid --truss-calibration-preset" >&2; exit 2 ;; esac
if [[ -n "$TRUSS_DAMPING_OVERRIDE" ]]; then
  case "$TRUSS_DAMPING_OVERRIDE" in 1|2|4|7) ;; *) echo "--truss-damping-override must be 1, 2, 4, or 7" >&2; exit 2 ;; esac
fi
case "$TRUSS_ARMATURE_MULTIPLIER" in 0|1|4) ;; *) echo "--truss-armature-multiplier must be 0, 1, or 4" >&2; exit 2 ;; esac
case "$TERMINAL_SOLVER_PRESET" in current|stabilized) ;; *) echo "--terminal-solver-preset must be current or stabilized" >&2; exit 2 ;; esac
if [[ "$VISUAL_QUALITY" != "realistic" && "$VISUAL_QUALITY" != "performance" ]]; then
  echo "--visual-quality must be realistic or performance" >&2
  exit 2
fi
if [[ "$INITIAL_OVERLAP_POLICY" != "filter" && "$INITIAL_OVERLAP_POLICY" != "error" ]]; then
  echo "--initial-overlap-policy must be filter or error" >&2
  exit 2
fi
if [[ "$ALLOW_OVER_BUDGET" == "true" && "$PHYSICAL_PETIOLULES" != "true" ]]; then
  echo "--allow-over-budget requires --physical-petiolules" >&2
  exit 2
fi
if [[ "$HEADLESS" == "true" && "$GENERATE_ONLY" == "true" ]]; then
  echo "--headless and --generate-only are mutually exclusive" >&2
  exit 2
fi
if [[ "$ORGAN" == "full" && "$ALLOW_EXPERIMENTAL_FRUIT_PHYSICS" != "true" ]]; then
  echo "--organ full requires --allow-experimental-fruit-physics" >&2
  exit 2
fi
if [[ "$ORGAN" != "full" && "$ALLOW_EXPERIMENTAL_FRUIT_PHYSICS" == "true" ]]; then
  echo "--allow-experimental-fruit-physics is valid only with --organ full" >&2
  exit 2
fi

if [[ -z "$OUTPUT" ]]; then
  OUTPUT="/tmp/autotom-phase-j-debug/day_${DAY}/tree_v2_day_${DAY}_${ORGAN}.usda"
fi

COMMAND=(
  "$SCRIPT_DIR/run_mainV2.sh"
  --day "$DAY"
  --debug-profile "$ORGAN"
  --pose-mode "$POSE_MODE"
  --appendage-pose-mode "$APPENDAGE_POSE_MODE"
  --physics-preset "$PHYSICS_PRESET"
  --duration "$DURATION"
  --physics-hz "$PHYSICS_HZ"
  --interactive-physics-hz "$INTERACTIVE_PHYSICS_HZ"
  --leaf-joint-policy "$LEAF_JOINT_POLICY"
  --lateral-joint-policy "$LATERAL_JOINT_POLICY"
  --truss-calibration-preset "$TRUSS_CALIBRATION_PRESET"
  --truss-armature-multiplier "$TRUSS_ARMATURE_MULTIPLIER"
  --terminal-solver-preset "$TERMINAL_SOLVER_PRESET"
  --visual-quality "$VISUAL_QUALITY"
  --initial-overlap-policy "$INITIAL_OVERLAP_POLICY"
)
[[ -z "$TRUSS_DAMPING_OVERRIDE" ]] || COMMAND+=(--truss-damping-override "$TRUSS_DAMPING_OVERRIDE")
[[ "$PHYSICAL_PETIOLULES" == "false" ]] || COMMAND+=(--physical-petiolules)
[[ "$ALLOW_NEAR_BUDGET" == "false" ]] || COMMAND+=(--allow-near-budget)
[[ "$ALLOW_OVER_BUDGET" == "false" ]] || COMMAND+=(--allow-over-budget)
[[ -z "$INPUT" ]] || COMMAND+=(--input "$INPUT")
COMMAND+=(--output "$OUTPUT")
[[ "$ALLOW_EXPERIMENTAL_FRUIT_PHYSICS" == "false" ]] || COMMAND+=(--allow-experimental-fruit-physics)
[[ "$HEADLESS" == "false" ]] || COMMAND+=(--headless)
[[ "$GENERATE_ONLY" == "false" ]] || COMMAND+=(--generate-only)

echo "=== ExporterV2 incremental checkpoint ==="
echo "day=$DAY organ=$ORGAN pose=$POSE_MODE appendages=$APPENDAGE_POSE_MODE physics=$PHYSICS_PRESET leaf_joints=$LEAF_JOINT_POLICY lateral_joints=$LATERAL_JOINT_POLICY truss_preset=$TRUSS_CALIBRATION_PRESET truss_damping=${TRUSS_DAMPING_OVERRIDE:-preset} truss_armature=${TRUSS_ARMATURE_MULTIPLIER} terminal_solver=$TERMINAL_SOLVER_PRESET physical_petiolules=$PHYSICAL_PETIOLULES overlap=$INITIAL_OVERLAP_POLICY visual_quality=$VISUAL_QUALITY validation_hz=$PHYSICS_HZ interactive_hz=$INTERACTIVE_PHYSICS_HZ"
exec "${COMMAND[@]}"
