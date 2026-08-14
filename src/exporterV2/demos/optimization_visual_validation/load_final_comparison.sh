#!/bin/bash
# Load before/after comparison in Isaac Sim
# Usage: ./load_final_comparison.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOADER_SCRIPT="$SCRIPT_DIR/load_final_comparison.py"
ISAACSIM="$HOME/isaacsim/python.sh"

echo "=================================="
echo "  Before/After Comparison Loader"
echo "=================================="
echo ""

# Check if USD files exist
if [ ! -f "$SCRIPT_DIR/usd_output_before_after/day_100_baseline.usda" ]; then
    echo "[ERROR] USD files not found!"
    echo "[HINT] Run: uv run python generate_final_comparison.py"
    exit 1
fi

echo "[INFO] Loading comparison in Isaac Sim..."
echo "  Baseline:  LEFT  (165 joints)"
echo "  Optimized: RIGHT (121 joints)"
echo ""

cd "$SCRIPT_DIR"
"$ISAACSIM" "$LOADER_SCRIPT"