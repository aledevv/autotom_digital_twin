#!/bin/bash
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
"$HOME/isaacsim/python.sh" "$SCRIPT_DIR/run_terminal_fork_visual.py"
