#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$PROJECT_DIR"
exec "$HOME/isaacsim-6.1/python.sh" "$PROJECT_DIR/src/experiments/leaf61/model_view.py" "$@"
