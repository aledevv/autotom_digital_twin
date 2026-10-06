#!/usr/bin/env bash
# Replicates the break-threshold matrix on the Isaac Sim installation in $ISAACSIM_DIR.
# Usage: ./run_matrix.sh [OUTPUT.jsonl]   (default: results_<timestamp>.jsonl next to this script)
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PY="${ISAACSIM_DIR:-$HOME/isaacsim}/python.sh"
OUT="${1:-$HERE/results_$(date +%Y%m%d_%H%M%S).jsonl}"
run() { timeout 200 "$PY" "$HERE/probe_break_threshold.py" --output "$OUT" "$@" >/dev/null 2>&1; }
for n in 8 16 32 64; do run --solver TGS --gpu 1 --iterations "$n" --support articulation; done
run --solver TGS --gpu 1 --iterations 32 --support articulation --break-force 3
run --solver TGS --gpu 1 --iterations 32 --support articulation --break-force 12
run --solver TGS --gpu 1 --iterations 32 --support articulation --hz 120
run --solver PGS --gpu 1 --iterations 32 --support articulation
run --solver TGS --gpu 0 --iterations 32 --support articulation
run --solver TGS --gpu 0 --iterations 64 --support articulation
run --solver TGS --gpu 1 --iterations 32 --support kinematic
run --solver PGS --gpu 0 --iterations 32 --support articulation
echo "results: $OUT"
