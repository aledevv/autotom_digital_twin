#!/usr/bin/env bash
# Thesis scene: day-160 tomato with the five original main-stem trusses,
# small tomatoes with calyces and slender pedicels (src/exporterV2/thesis_scene.py).
#
#   ./run_thesis.sh                  build (first time only) and open in Isaac Sim
#   ./run_thesis.sh --rebuild        rebuild the scene, then open it
#   ./run_thesis.sh --generate-only  rebuild the scene only, no Isaac Sim
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAACSIM_DIR="${ISAACSIM_DIR:-$HOME/isaacsim}"
OUTPUT="$SCRIPT_DIR/output/thesis"
REBUILD="false"
OPEN="true"
case "${1:-}" in
  "") ;;
  --rebuild) REBUILD="true" ;;
  --generate-only) REBUILD="true"; OPEN="false" ;;
  -h|--help) sed -n 2,8p "$0"; exit 0 ;;
  *) echo "Unknown option: $1" >&2; exit 2 ;;
esac

cd "$SCRIPT_DIR"
if [[ "$REBUILD" == "true" || ! -f "$OUTPUT/scene.usda" ]]; then
  uv run python -m exporterV2.thesis_scene --output "$OUTPUT"
fi
[[ "$OPEN" == "true" ]] || exit 0

ISAAC_PYTHON="$ISAACSIM_DIR/python.sh"
if [[ ! -x "$ISAAC_PYTHON" ]]; then
  echo "Isaac Sim launcher not found or not executable: $ISAAC_PYTHON" >&2
  exit 2
fi
RUN="$(uv run python -m exporterV2.thesis_scene --output "$OUTPUT" --prepare-gui-run | tail -n 1)"
HZ="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["hz"])' "$RUN/config.json")"
exec "$ISAAC_PYTHON" "$SCRIPT_DIR/src/exporterV2/isaac_app.py" \
  --usd "$RUN/scene.usda" --physics-preset flexible --interactive-physics-hz "$HZ" \
  --duration 60 --fruit-experiment "$RUN/config.json" --gui-until-close
