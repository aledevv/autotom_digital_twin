#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec "$PROJECT_DIR/run_leaf61_full_plant.sh" \
  --leaf-contacts all \
  --exclude-initial-from "$PROJECT_DIR/artifacts/leaf61/full-plant-contact-all-preflight" \
  "$@"
