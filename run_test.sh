#!/bin/bash
set -e

~/isaacsim/python.sh src/experiments/tomato_branch_physics/load_tomato_branch.py

uv run src/experiments/tomato_branch_physics/plot_forces.py
