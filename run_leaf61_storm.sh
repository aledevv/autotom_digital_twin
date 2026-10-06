#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec "$PROJECT_DIR/run_leaf61_day160.sh" --storm --storm-rate 200 --storm-gain 1 "$@"
