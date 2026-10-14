# exporterV2

Turns a plant skeleton into an articulated OpenUSD plant for Isaac Sim:
rigid links joined by bending D6 joints, organic meshes, leaflets and
tomatoes.

```text
input/skeleton.json ─┐
config/physics.yaml ─┼─> exporterV2 ─> output/plant.usda (+ .manifest.json) ─> Isaac Sim
config/visuals.yaml ─┘
```

## Quick start

```bash
./run_mainV2.sh                       # build output/plant.usda and open it in Isaac Sim
./run_mainV2.sh --generate-only       # build only, no Isaac Sim
./run_mainV2.sh --headless            # 5 s physics check without GUI
./run_mainV2.sh --input input/examples/small_plant.json
```

Or without the launcher:

```bash
uv run python -m exporterV2 [--input PATH] [--physics PATH] [--visuals PATH] [--output PATH]
```

It prints a one-line summary, for example
`[OK] 216 segments -> 216 rigid bodies, 206 D6 joints, 131 leaflets, 72 fruits (scale 2)`.

## What to edit

| I want to... | Edit |
| --- | --- |
| Load a different plant | `input/skeleton.json` ([format](../plant_skeleton/README.md)) |
| Convert another program's plant | write an adapter ([guide](adapters/README.md), GroIMP example in `adapters/groimp.py`) |
| Change stiffness, damping, mass, joint limits, fruit physics, scale | `config/physics.yaml` |
| Change mesh density, leaf generator, decorations, pose | `config/visuals.yaml` |
| Change how a mesh is generated | the generator listed in [visuals/README.md](visuals/README.md) |

Both YAML files list every option with its default and an explanation.
Every key is optional, and unknown keys are rejected so typos are caught.

## Pipeline

`exporter.py` (`export_plant`) runs these steps:

1. **Load** the skeleton (`plant_skeleton.load_skeleton`) and both configs (`config.py`).
2. **Scale**: `global_scale` (a number, or `auto`) multiplies every length.
   PhysX needs bodies at least about 2 mm thick; the GroIMP tomato uses 2.
3. **Branches** (`skeleton_branches.py`): nodes are grouped into chains
   (trunk, side shoots, leaves, truss rachis, pedicels). Each chain becomes
   one branch with one rigid link per node, its physics block from
   `segments.<kind>`, and the leaflet and fruit records hanging from it. The
   default `v2-aesthetic` pose spreads leaflets and pedicels at fixed angles;
   `canonical` uses the skeleton frames.
4. **Joint budget** (`joint_budget.py`, optional): above about 220 D6 joints,
   `joints.optimize: lock` turns the least important joints into fixed
   joints and `full` also merges links.
5. **USD stage** (`core/usd/stage.py`): rigid bodies, capsule colliders,
   D6 joints with drives computed from Young's modulus and radius, organic
   segmented meshes, tomato spheres (visual or physical).
6. **Dressing** (`visuals/leaves.py`, `visuals/trusses.py`): leaflet stalks
   and blades, truss material, gravity-curved pedicels.
7. **Collisions** (`collisions.py`): colliders that already overlap at rest
   are filtered pairwise (or the export stops, with `collisions.initial_overlaps: error`).
8. **Scene settings** and an **audit** that compares the authored counts with
   the plan; the manifest JSON records counts, poses, masses and settings.

`isaac_app.py` opens the USDA in Isaac Sim with the physics rates stored in
the stage and the interactive drag tools; `run_mainV2.sh` calls it.

## Folder map

```text
exporterV2/
├── cli.py, __main__.py   command line (defaults: input/skeleton.json, config/*.yaml, output/plant.usda)
├── config.py             loads and checks config/physics.yaml and config/visuals.yaml
├── exporter.py           the pipeline above
├── skeleton_branches.py  skeleton nodes -> branch chains, poses, leaflet and fruit records
├── joint_budget*.py      optional joint reduction
├── collisions.py         initial-overlap filter
├── visuals/              leaflets and truss dressing; README maps every mesh generator
├── adapters/             other programs -> plant_skeleton (GroIMP example)
├── leaf_shapes/          worker that runs the real_leaves submodule
├── core/                 generic builder: tree_config, usd/, skinning/, optimizations/
├── isaac_app.py          Isaac Sim launcher and interaction tools
├── legacy_csv/           old GroIMP CSV builder, kept only as a fixture for core tests
├── profiles/leaf_shape.yaml  real_leaves worker settings
├── demos/                small manual Isaac demos (sphere, fixed joint, leaf shape)
└── docs/                 technical notes: physics, collisions, visuals, leaf shapes
```

The diagnostic tools next to `isaac_app.py` (`fruit_diagnostics.py`,
`performance_benchmark.py`, `complexity_profile.py`, `drive_experiments.py`,
`settling_gate.py`, ...) are optional helpers used by tests and experiments.

## Tests

No CI: run the suite by hand before merging.

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

Key tests:

- `tests/test_exporter.py`: the hand-written small plant, fruit modes,
  scale, joint budget, CLI, and the **day-160 regression**: the GroIMP day-160
  skeleton must give the same bodies, joints, poses, masses and drive gains as
  the old PlantState exporter (`tests/data/groimp_day160_reference.json`).
- `tests/test_groimp_adapter.py`: adapter output per day and the committed `input/skeleton.json`.
- `tests/test_skeleton_branches.py`: chain rules, attachment, pose, config switches.
- `tests/test_config.py`: the commented YAML files equal the built-in defaults.

The Isaac Sim GUI (dragging, bending, fruit detachment) can only be checked by
hand on a machine with Isaac Sim: `./run_mainV2.sh`.

## Known limits

- Physics parameters are tuned by eye, not calibrated on a real plant.
- Physical tomatoes (`fruits.mode: physical`) are stable only on trusses of
  the main stem; side trusses sag and can drop fruit.
- Fruits are spheres only.
- One plant per scene has been tested.
