# Guide for coding agents

Read this before changing code. It says what the project does, where things
live, what must not break and how to verify a change. Behavioural rules are in
the root [`AGENTS.md`](../AGENTS.md); the human overview is
[`DEVELOPER_GUIDE.md`](DEVELOPER_GUIDE.md).

## Purpose

The exporter turns a `plant_skeleton/1.0` graph (straight segments, leaflets,
fruits) into an articulated OpenUSD plant for NVIDIA Isaac Sim 4.5 (PhysX):
one rigid body per segment, D6 joints with drives, capsule colliders, and
visual-only organic meshes. Any growth model feeds it through an adapter; the
GroIMP dwarf-tomato adapter is the reference.

```text
adapter ─> input/skeleton.json ─┐
           config/physics.yaml ─┼─> python -m exporterV2 ─> output/plant.usda + plant.manifest.json ─> isaac_app.py
           config/visuals.yaml ─┘
```

## Entry points

| Command | Effect |
| --- | --- |
| `./run_main.sh` | Export with defaults, then open Isaac Sim (GUI). |
| `./run_main.sh --generate-only` | Export only. Use this to verify a change without Isaac Sim. |
| `./run_main.sh --headless [--duration S]` | Export and run a short physics check (needs Isaac Sim). |
| `./run_main.sh --input P --output P` | Other skeleton or output path. These five flags are all the launcher has; keep it that way. |
| `uv run python -m exporterV2 [--input] [--physics] [--visuals] [--output]` | Exporter without the launcher. |
| `uv run python -m exporterV2.adapters.groimp --day N [--output P]` | GroIMP snapshot `data/plant_states/plant_state_day_N.json` to skeleton. |
| `./extract_plant_states.sh --day N` | New GroIMP snapshots from a running GroIMP ([details](GROIMP_BATCH_EXTRACTION.md)). |

## Code map

| Path | Role |
| --- | --- |
| `src/plant_skeleton/` | Input format: `model.py` (dataclasses), `json_io.py`, `validation.py` (`validate_skeleton`, `skeleton_warnings`). Spec in its `README.md`. |
| `src/exporterV2/cli.py` | CLI and default paths. |
| `src/exporterV2/config.py` | Loads both YAML files. Unknown keys are errors; defaults must equal the commented YAML (`tests/test_config.py`). |
| `src/exporterV2/exporter.py` | `export_plant`: the whole pipeline in order. |
| `src/exporterV2/skeleton_branches.py` | Nodes to branch chains, poses, leaflet and fruit records, per-link visual profile. |
| `src/exporterV2/joint_budget.py`, `joint_budget_merges.py` | Optional lock/merge above the joint budget. |
| `src/exporterV2/collisions.py` | Filter for colliders overlapping at rest. |
| `src/exporterV2/core/tree_config.py` | Geometry, mechanics, detachment, collision and runtime defaults. |
| `src/exporterV2/core/usd/` | Stage authoring: `stage.py`, `branch_chains.py`, `joints.py`, `terminal_bodies.py`, `materials.py`, `geometry.py`, `pedicel_geometry.py`. |
| `src/exporterV2/core/skinning/` | Organic meshes (`visual_segmented.py`), leaf blade, terminal forks. |
| `src/exporterV2/visuals/` | Leaflet, truss and calyx dressing. `README.md` maps every mesh generator to its code and YAML switch. |
| `src/exporterV2/adapters/` | `groimp.py` (reference adapter), `README.md` (how to write one). `groimp_csv/` is legacy. |
| `src/exporterV2/leaf_shapes/` | Worker that runs the `external/real_leaves` submodule in its own Python 3.12 venv. |
| `src/exporterV2/isaac_app.py` | Isaac Sim launcher and drag tools. |
| `src/exporterV2/legacy_csv/` | Old CSV builder, kept only as a fixture for core tests. Do not extend it. |
| `src/groimp_bridge/`, `src/plant_state/`, `model/` | GroIMP side: API client, snapshot schema, RGG model. |

## Setup

```bash
git clone --recurse-submodules https://github.com/aledevv/autotom_digital_twin
uv sync
```

`external/real_leaves` is pinned at `aaf8890`. Without it the default leaflet
generator (`real_leaves`) fails, and so do `test_real_leaf_shapes.py` and the
tests that export with real leaves. Use `leaves.generator: simple` or init the
submodule. Isaac Sim (`~/isaacsim`, or `ISAACSIM_DIR`) is usually not
available to an agent: GUI behaviour can only be checked by the user.

## Verify a change

No CI. Run before proposing a merge:

```bash
uv run pytest -q \
  src/plant_skeleton/tests \
  src/exporterV2/tests \
  src/exporterV2/core/optimizations/tests \
  src/exporterV2/core/skinning/tests \
  src/exporterV2/legacy_csv/tests \
  src/groimp_bridge/tests \
  tests
./run_main.sh --generate-only
```

Tests that guard behaviour:

- `src/exporterV2/tests/test_exporter.py` holds the **day-160 regression**:
  the GroIMP day-160 skeleton must give the same bodies, joints, poses, masses
  and drive gains as the old exporter
  (`tests/data/groimp_day160_reference.json`). A physics change that breaks it
  needs the user's approval before the reference is updated.
- `test_groimp_adapter.py` checks the committed `input/skeleton.json` equals
  the adapter output. Regenerate it with the adapter when the adapter changes.
- `test_config.py` checks the commented YAML files equal the built-in
  defaults. Change both together.

Tests marked `groimp` need a running GroIMP server and are opt-in.

## Rules

- Physics and visuals are separate. A visual option (`config/visuals.yaml`,
  `visuals/`, `core/skinning/`) must never change masses, colliders or joints.
  `fruits.size_scale` and `fruits.calyx` are visual only, by design.
- The exporter stays source-agnostic. Anything specific to GroIMP or another
  source goes in its adapter, not in `exporter.py` or `skeleton_branches.py`.
- Every config key has a commented default in the YAML and a matching entry in
  `config.py`. Keep the launcher at its five flags; new options go in YAML.
- Fruits are spheres. Physical fruits are supported only on main-stem trusses;
  side trusses are a known limitation, not a bug to fix in passing.
- `global_scale: 2` is the tested value for the GroIMP tomato. Physics
  parameters are not calibrated; never describe them as measured.
- Generated files (`output/`, `data/usd_models/`) are not tracked. Tests must
  not rewrite tracked files.
- Branches: this version lives on `claude/project-thread-eryl3b` and is not
  merged into `main` (the thesis version). Never delete branches, above all
  `thesis-archive`, which holds the thesis experiments referenced below.
- Docs and code comments are in English.

## Adding things

- **A new source of plants:** a new adapter, following
  `src/exporterV2/adapters/README.md`; validate with `validate_skeleton`.
- **A new config option:** YAML (commented) + `config.py` + the code that reads
  it + a test.
- **A new mesh or fruit shape:** see "Changing a look" in
  `src/exporterV2/visuals/README.md`.
- **A new segment kind:** `plant_skeleton/model.py` and `validation.py`
  (allowed parents), a `segments.<kind>` block in `physics.yaml` and
  `config.py`, the chain rules in `skeleton_branches.py`.

## Future work

- **Visual details.** Plant tips and terminal shoots can look better.
- **PhysX time-step bug.** PhysX seems to have a bug related to the time step
  (dt), also visible in the public PhysX source. To confirm it, reproduce it in
  a minimal scene or build PhysX with the fix.
- **Fruit on side branches.** Tomatoes on side trusses make the plant
  unstable. A PhysX TGS-on-GPU bug lowers the effective break threshold to about
  `breakForce / N` (N = solver iterations); this explains spontaneous fruit
  detachment on side trusses. It does not explain the 13-24 cm sagging of side
  branches, which persists with PGS. Evidence on branch `thesis-archive`:
  `src/experiments/detachable_fruit_v2/*.md` and
  `src/experiments/break_threshold_probe/REPORT.md`.
- **Several plants in one scene.** Untested in Isaac Sim. It should be possible
  to turn collisions and physics on or off per plant.
- **Branch generator.** No generator builds a branch from a length and a
  number of joints, because GroIMP has always provided complete branches. A
  natural place is a helper in `src/plant_skeleton/` that emits nodes.
