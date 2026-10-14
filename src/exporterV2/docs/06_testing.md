# Testing

There is no CI. Run the suite by hand before merging:

```bash
uv run pytest -q \
  src/plant_skeleton/tests \
  src/exporterV2/tests \
  src/exporterV2/core/optimizations/tests \
  src/exporterV2/core/skinning/tests \
  src/exporterV2/legacy_csv/tests \
  src/groimp_bridge/tests \
  tests
```

Live GroIMP tests in `src/groimp_bridge/tests` skip unless a GroIMP server is
running.

## What the main tests cover

| Test | Covers |
| --- | --- |
| `src/plant_skeleton/tests/test_skeleton.py` | format, JSON round trip, validation messages |
| `tests/test_groimp_adapter.py` | adapter counts per day, leaflet sides, pedicel scaling, committed `input/skeleton.json` |
| `tests/test_skeleton_branches.py` | chain rules, attachment, physics per kind, visual switches, poses |
| `tests/test_config.py` | commented YAML files equal the defaults, bad values rejected, `global_scale: auto` |
| `tests/test_exporter.py` | small plant end to end, fruit modes, scale, joint budget, CLI, day-160 regression |
| `tests/test_joint_budget*.py` | lock and merge on the day-160 plant |
| `tests/test_real_leaf_shapes.py` | `simple`, `real_leaves`, `real_leaves_i3` leaflets |
| `tests/test_fruit_diagnostics.py` | physical-fruit diagnostics |
| `legacy_csv/tests`, `core/*/tests`, root `tests/` | generic core builder, using the legacy CSV builder as a fixture |

(`tests/` in the table means `src/exporterV2/tests/`.)

The day-160 regression compares against
`src/exporterV2/tests/data/groimp_day160_reference.json`, recorded from the
previous PlantState exporter: same bodies (216), D6 (206) and fixed (10)
joints, capsules, meshes, link poses, masses and drive gains. If a deliberate
change alters them, regenerate the reference and say why in the commit.

## USD and PhysX tests under Isaac Python

Tests that need `PhysxSchema` run with Isaac Sim's Python:

```bash
~/isaacsim/python.sh -m pytest src/exporterV2/core/usd/tests -v
```

## Manual checks in Isaac Sim

```bash
./run_mainV2.sh                    # GUI: Shift+drag a branch, a leaf and a truss
./run_mainV2.sh --headless         # 5 s physics run, prints the stability check
```

## Performance comparison

```bash
uv run python -m exporterV2.performance_benchmark \
  --baseline before=/tmp/before.usda \
  --candidate after=/tmp/after.usda \
  --physics-hz 60,120,240,480 \
  --output /tmp/performance.json
```

One clean Isaac worker runs per rate; both stages load in the same process.

## Manual demos

`src/exporterV2/demos/` holds small Isaac demos (sphere, fixed joint, real leaf
shape) outside pytest collection.
