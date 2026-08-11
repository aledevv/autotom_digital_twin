#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAACSIM_DIR="$HOME/isaacsim"

echo "=================================================="
echo " Cantilever Bending Experiment"
echo "=================================================="
echo "This experiment verifies the physical accuracy of"
echo "the articulated plant joints against Euler-Bernoulli"
echo "beam theory."
echo ""

echo "[1/2] Checking analytical formulas..."
UV_CACHE_DIR=/tmp/uv-cache uv run python $SCRIPT_DIR/cantilever_validation.py formula-check >/dev/null

echo "[2/2] Generating, auditing, and running quantitative simulations..."
$ISAACSIM_DIR/python.sh $SCRIPT_DIR/cantilever_validation.py all

echo "=================================================="
echo "Experiment finished!"
echo "Generated USD files: data/usd_models/physics_tests/"
echo "Results: $SCRIPT_DIR/results/"
echo ""
echo "To view the simulation in Isaac Sim UI, run:"
echo "  $ISAACSIM_DIR/python.sh $SCRIPT_DIR/simulate_cantilever_headless.py --gui 10"
echo ""
echo "To visually COMPARE the old (broken) vs new physics side-by-side, run:"
echo "  $ISAACSIM_DIR/python.sh $SCRIPT_DIR/compare_cantilevers.py --benchmark tomato_gao_20cm --n 10"
echo "(You can replace 10 with 3 or 20 to view other resolutions)"
echo "=================================================="
