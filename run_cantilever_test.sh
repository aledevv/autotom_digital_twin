#!/bin/bash
set -e

echo "Running Cantilever Bending Test in Isaac Sim..."
~/isaacsim/python.sh src/experiments/cantilever_test/run_cantilever.py

echo "Generating Deflection Plot..."
uv run src/experiments/cantilever_test/plot_deflection.py
