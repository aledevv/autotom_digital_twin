#!/bin/bash
# run_experiment.sh
#
# Unified experiment launcher for the autotom_digital_twin project.
#
# ── Joint Parametrization Experiments ────────────────────────────────────────
#   ./run_experiment.sh exp1    # Phase 1: Static Deflection Test (uses PlantBuilder)
#   ./run_experiment.sh exp1v2  # Phase 1 v2: Static Deflection — raw USD, no PlantBuilder
#                               #   Includes zero-gravity sanity pass, per-step logging,
#                               #   and tests whether K∝N holds across segment counts.
#   ./run_experiment.sh exp2    # Phase 2: Tapering Test (linear vs r⁴)
#   ./run_experiment.sh exp3    # Phase 3: Dynamic Oscillation Test
#
# ── Legacy PlantBuilder Visual Tests ─────────────────────────────────────────
#   ./run_experiment.sh         # defaults to visual test 1
#   ./run_experiment.sh 1       # trunk only
#   ./run_experiment.sh 2       # trunk + branch
#   ./run_experiment.sh 3       # branch with extensions
#   ./run_experiment.sh 4       # subbranch
#   ./run_experiment.sh 5       # full tree
#
# To change which experiment runs, edit the ARG variable below:
#   ARG="exp1"   ← change this line

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAACSIM_DIR="$HOME/isaacsim"

# ── Edit this to pick a default experiment ────────────────────────────────────
ARG="${1:-1}"

# ── Route to the correct script ───────────────────────────────────────────────
cd "$ISAACSIM_DIR"

case "$ARG" in
  exp1)
    EXPERIMENT_SCRIPT="$SCRIPT_DIR/src/experiments/joint_parametrization/phase1_static_deflection/run_phase1.py"
    echo "=== Joint Parametrization — Phase 1: Static Deflection Test ==="
    ./python.sh "$EXPERIMENT_SCRIPT"
    ;;
  exp1v2)
    EXPERIMENT_SCRIPT="$SCRIPT_DIR/src/experiments/joint_parametrization/phase1_version2/run_phase1_v2.py"
    echo "=== Joint Parametrization — Phase 1 v2: Raw USD Articulation (no PlantBuilder) ==="
    ./python.sh "$EXPERIMENT_SCRIPT"
    ;;
  exp2)
    EXPERIMENT_SCRIPT="$SCRIPT_DIR/src/experiments/joint_parametrization/phase2_tapering/run_phase2.py"
    echo "=== Joint Parametrization — Phase 2: Tapering Test ==="
    ./python.sh "$EXPERIMENT_SCRIPT"
    ;;
  exp3)
    EXPERIMENT_SCRIPT="$SCRIPT_DIR/src/experiments/joint_parametrization/phase3_oscillation/run_phase3.py"
    echo "=== Joint Parametrization — Phase 3: Dynamic Oscillation Test ==="
    ./python.sh "$EXPERIMENT_SCRIPT"
    ;;
  [1-9]|[1-9][0-9])
    # Legacy numeric visual tests
    TEST_SCRIPT="$SCRIPT_DIR/tests/plant_builder/visual_tests.py"
    echo "=== PlantBuilder Visual Test $ARG ==="
    ./python.sh "$TEST_SCRIPT" "$ARG"
    ;;
  *)
    echo "ERROR: Unknown argument '$ARG'."
    echo ""
    echo "Usage:"
    echo "  $0 exp1    # Phase 1: Static Deflection Test (PlantBuilder)"
    echo "  $0 exp1v2  # Phase 1 v2: Raw USD articulation, no PlantBuilder"
    echo "  $0 exp2    # Phase 2: Tapering Test"
    echo "  $0 exp3    # Phase 3: Dynamic Oscillation Test"
    echo "  $0 1-5     # Legacy visual tests"
    exit 1
    ;;
esac
