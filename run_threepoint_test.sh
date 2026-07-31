#!/bin/bash
set -e

echo "Running Three-Point Bending Analytical Verification..."
python src/experiments/three_point_test/verify_experiment.py

echo "Running Three-Point Bending Test in Isaac Sim..."
~/isaacsim/python.sh src/experiments/three_point_test/run_threepoint.py

echo "Generating Three-Point Bending Plot..."
uv run src/experiments/three_point_test/plot_threepoint.py
