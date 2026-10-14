#!/usr/bin/env bash
# Build the plant USDA and open it in Isaac Sim.
#
#   ./run_mainV2.sh                  input/skeleton.json -> output/plant.usda -> Isaac Sim GUI
#   ./run_mainV2.sh --generate-only  only write the USDA (no Isaac Sim)
#   ./run_mainV2.sh --headless       run a short physics check without the GUI
#
# Settings live in config/physics.yaml and config/visuals.yaml.
# To convert a GroIMP day first: uv run python -m exporterV2.adapters.groimp --day 160
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAACSIM_DIR="${ISAACSIM_DIR:-$HOME/isaacsim}"
INPUT="$SCRIPT_DIR/input/skeleton.json"
OUTPUT="$SCRIPT_DIR/output/plant.usda"
GENERATE_ONLY="false"
HEADLESS="false"
DURATION="5"

usage() {
  sed -n '2,9p' "$0" | sed 's/^# \{0,1\}//'
  echo
  echo "Options:"
  echo "  --input PATH       skeleton JSON (default: input/skeleton.json)"
  echo "  --output PATH      USDA to write (default: output/plant.usda)"
  echo "  --generate-only    write the USDA and stop"
  echo "  --headless         physics check without GUI"
  echo "  --duration SEC     headless simulated seconds (default: 5)"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --input) INPUT="${2:?Missing value for --input}"; shift 2 ;;
    --output) OUTPUT="${2:?Missing value for --output}"; shift 2 ;;
    --generate-only) GENERATE_ONLY="true"; shift ;;
    --headless) HEADLESS="true"; shift ;;
    --duration) DURATION="${2:?Missing value for --duration}"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage; exit 2 ;;
  esac
done

(
  cd "$SCRIPT_DIR"
  uv run python -m exporterV2 --input "$INPUT" --output "$OUTPUT"
)
[[ "$GENERATE_ONLY" == "false" ]] || exit 0

ISAAC_PYTHON="$ISAACSIM_DIR/python.sh"
if [[ ! -x "$ISAAC_PYTHON" ]]; then
  echo "Isaac Sim launcher not found: $ISAAC_PYTHON (set ISAACSIM_DIR)" >&2
  exit 2
fi

# Simulation rates and fruit mode come from the manifest written next to the USDA.
MANIFEST="${OUTPUT%.*}.manifest.json"
read -r PHYSICS_HZ INTERACTIVE_HZ FRUIT_MODE < <(
  cd "$SCRIPT_DIR" && uv run python -c '
import json, sys
m = json.load(open(sys.argv[1]))["metadata"]
s = m["physics"]["simulation"]
print(s["physics_hz"], s["interactive_physics_hz"], m["fruit_mode"])
' "$MANIFEST"
)

ISAAC_ARGS=(--usd "$OUTPUT" --duration "$DURATION" --physics-preset flexible
  --physics-hz "$PHYSICS_HZ" --interactive-physics-hz "$INTERACTIVE_HZ")
[[ "$FRUIT_MODE" == "physical" ]] || ISAAC_ARGS+=(--diagnostic-monitor)
[[ "$HEADLESS" == "false" ]] || ISAAC_ARGS+=(--headless)
exec "$ISAAC_PYTHON" "$SCRIPT_DIR/src/exporterV2/isaac_app.py" "${ISAAC_ARGS[@]}"
