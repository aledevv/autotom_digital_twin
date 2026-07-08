#!/bin/bash
# run_generate_subbranch.sh
#
# Genera il file USD dell'articolazione con subbranch usando l'ambiente uv locale.
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GENERATE_SCRIPT="src/experiments/articulation_subbranch/generate_articulation_usda.py"

echo "=== Generazione USD tramite uv ==="
cd "$SCRIPT_DIR"
uv run python "$GENERATE_SCRIPT"
echo "Generazione completata."
