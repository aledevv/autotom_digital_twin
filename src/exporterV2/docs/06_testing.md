# Testing

The repository now separates ordinary tests, Isaac/PhysX tests, and visual demos.

## Ordinary Python Tests

Run these in the project virtualenv:

```bash
uv run pytest \
  src/exporterV2/adapters/groimp_csv/tests \
  src/exporterV2/core/optimizations/tests \
  src/exporterV2/tests \
  -v
```

These cover CSV parsing, single-read behavior, generation switches, truss builder output, optimizer behavior, collision geometry, and cantilever validation helpers.

## USD and PhysX Tests

Tests that inspect `UsdPhysics` or `PhysxSchema` should run under Isaac Sim when the regular virtualenv does not provide the required modules:

```bash
~/isaacsim/python.sh -m pytest \
  src/exporterV2/core/usd/tests \
  src/exporterV2/core/optimizations/tests/11_truss_static/test_truss_static_usd.py \
  -v
```

These validate authored scene settings, runtime iterations, FixedJoints, break force, `excludeFromArticulation`, terminal body placement, and collision filters.

## Performance comparison

Use the same Isaac process, render cadence and physics rates for legacy/new
comparisons:

```bash
uv run python -m exporterV2.performance_benchmark \
  --baseline legacy=/tmp/tree_legacy.usda \
  --candidate candidate=/tmp/tree_candidate.usda \
  --physics-hz 60,120,240,480 \
  --require-candidate-faster \
  --output /tmp/exporter_v2_performance.json
```

The generated `exporter_v2_performance_comparison/1.0` JSON includes both raw
stage counts and measured render/PhysX throughput. Timing values naturally
vary between runs; field order, labels and rate ordering are deterministic.
The lightweight orchestrator starts one clean Isaac worker per rate; within
each worker legacy and candidate are loaded sequentially in the same process.

To compare the two PlantState leaf-joint policies without changing geometry:

```bash
./run_debugV2.sh --day 50 --organ leaves --leaf-joint-policy optimized --generate-only \
  --output /tmp/day50_leaves_optimized.usda
./run_debugV2.sh --day 50 --organ leaves --leaf-joint-policy distributed --generate-only \
  --output /tmp/day50_leaves_distributed.usda
uv run python -m exporterV2.performance_benchmark \
  --baseline optimized=/tmp/day50_leaves_optimized.usda \
  --candidate distributed=/tmp/day50_leaves_distributed.usda \
  --physics-hz 60,120,240,480 \
  --output /tmp/day50_leaf_joint_comparison.json
```

The day-50 acceptance threshold for `distributed` is 20 rendered updates/s at
the 60 Hz interactive cadence, plus a five-second 480 Hz headless stability
pass. Visual bending and Shift+drag remain manual GUI checks.

To compare visual quality without changing physics topology:

```bash
./run_debugV2.sh --day 50 --organ leaves --visual-quality realistic --generate-only \
  --output /tmp/day50_leaves_realistic.usda
./run_debugV2.sh --day 50 --organ leaves --visual-quality performance --generate-only \
  --output /tmp/day50_leaves_performance.usda
```

The manifest records the selected profile, mesh complexity and visual leaf
mass aggregated into each physical support.

## Manual Demos

Manual visual assets live in `src/exporterV2/demos/` and are not part of pytest collection. They are intended for paper figures, screenshots, videos, and Isaac Sim inspection:

- truss visual lab
- primitive sphere/fixed-joint demos
- optimization before/after visual validation
- generated comparison USDA files

## Runner Scripts

Root contains only:

- `run_mainV1.sh`
- `run_mainV2.sh`

Experiment-specific runners live beside their experiments, for example `src/experiments/cantilever_test/run_cantilever_test.sh` and `src/experiments/three_point_test/run_threepoint_test.sh`.
