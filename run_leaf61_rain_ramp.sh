#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec "$PROJECT_DIR/run_leaf61_rain.sh" --rain-waves 1,1,2,3,4,6,8,10,12 "$@"
