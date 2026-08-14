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

## Manual Demos

Manual visual assets live in `src/exporterV2/demos/` and are not part of pytest collection. They are intended for paper figures, screenshots, videos, and Isaac Sim inspection:

- truss visual lab
- primitive sphere/fixed-joint demos
- optimization before/after visual validation
- generated comparison USDA files

## Runner Scripts

Root contains only:

- `run_main.sh`
- `run_mainV2.sh`

Experiment-specific runners live beside their experiments, for example `src/experiments/cantilever_test/run_cantilever_test.sh` and `src/experiments/three_point_test/run_threepoint_test.sh`.
