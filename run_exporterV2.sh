#!/bin/bash

# run_exporterV2.sh
# Launch exporterV2 tree model generator and Isaac Sim simulation

set -e  # Exit on error

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "=========================================="
echo "  Launching exporterV2 Tree Model"
echo "=========================================="
echo ""

# Check if Isaac Sim is available
if [ ! -f "$HOME/isaacsim/python.sh" ]; then
    echo "ERROR: Isaac Sim not found at ~/isaacsim/python.sh"
    echo "Please check your Isaac Sim installation."
    exit 1
fi

# Run the main script
echo "Starting Isaac Sim..."
"$HOME/isaacsim/python.sh" src/exporterV2/main.py

echo ""
echo "=========================================="
echo "  exporterV2 finished"
echo "=========================================="
