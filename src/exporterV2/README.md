# ExporterV2 - Modular Tree Model Generator

Modular tomato plant model generator with **clean separation** between generic
tree building, source adapters, cultivar-specific logic, and USD physics
authoring. The general builder and the GroIMP PlantState adapter have different
support boundaries; see the fruit-physics note below.

## Architecture

```
exporterV2/
├── core/              # Generic tree builder (reusable for any plant)
├── adapters/          # Data source adapters (CSV, manual, etc.)
├── profiles/          # Cultivar-specific configurations
└── main.py            # Entry point
```

---

## Quick Start

### Canonical stem checkpoint

The incremental Phase-J migration routes the main stem through the original
V2 segmented backend:

```bash
uv run python -m exporterV2 --day 10 --debug-profile stem \
  --pose-mode canonical --generate-only

./run_mainV2.sh --day 10 --debug-profile stem --pose-mode canonical
```

For day-to-day testing, both wrappers default to the validated fruit-free
`truss-supports` scene. `--organ` remains available to isolate a cumulative
diagnostic checkpoint:

```bash
./run_debugV2.sh --day 50
./run_debugV2.sh --day 50 --headless --duration 1
./run_debugV2.sh --day 10 --organ stem
./run_debugV2.sh --day 25 --organ stem --generate-only
```

The conservative backend implements the cumulative profiles `stem`,
`laterals`, `leaf-supports`, `leaves`, `truss-supports`, `fruit-visual`, and
`full`. `truss-supports` adds canonical rachides and dynamic pedicels;
`fruit-visual` adds static tomato visuals. `full` restores the historical V2
breakable terminal-body physics, but is unsupported for mature PlantState
plants and requires `--allow-experimental-fruit-physics`.

`canonical` uses the rebased GroIMP frame, length and radius of each
internode. `legacy` retains those per-link dimensions but lets V2 construct the
old procedural vertical rest pose. Both modes use segmented organic meshes,
not visual cylinders. The stem is fixed by design; interactive leaf tests
start with the later `leaf-supports` checkpoint.

Appendices have an independent pose switch. The default
`--appendage-pose-mode v2-aesthetic` preserves native topology, attachment,
dimensions and counts, but restores the historical V2 angles for lateral
petiolules and pedicels. `canonical` renders their raw GroIMP frames instead:

```bash
./run_debugV2.sh --day 50 --appendage-pose-mode v2-aesthetic
./run_debugV2.sh --day 50 --appendage-pose-mode canonical
```

Both source and authored poses are retained in the manifest. Leaf blades keep
the historical V2 longitudinal fold, centre arch and tip sag. Truss rachides
remain articulated internally, but their segmented organic meshes form one
continuous visual axis; no per-segment visual cylinders are authored.

PlantState pedicels use
`TrussGeometryConfig.PLANT_STATE_PEDICEL_LENGTH_SCALE` (currently `3.0`). The
GroIMP length remains recorded as the source value; the scale consistently
extends the organic mesh, collider, physical link and tomato attachment.

Day-160 truss calibration can be performed without editing `tree_config.py`:

```bash
./run_debugV2.sh --day 160 --organ truss-supports \
  --lateral-joint-policy fixed --truss-calibration-preset balanced
```

Available presets are `compliant`, `balanced`, `firm` and `current`; an
optional `--truss-damping-override 1|2|4|7` is applied after the preset.
Truss tissue is authored at `2000 kg/m3`, while ordinary vegetative tissue
remains at `1000 kg/m3`. The later fruit-bearing fallback controls are
`--terminal-solver-preset stabilized` (64/4 iterations) and
`--truss-armature-multiplier 1|4`. Neither fallback is active by default.

The selected day-160 values are rachis `20 GPa`, pedicel `4 GPa`, damping
ratio `4` and pedicel drive scale `0.2`. The supported day-160 scene has 216
bodies, 206 D6, 10 Fixed, 432 capsules and no fruit bodies. It passed the
five-second 480 Hz headless test. The unsuccessful gravity-ramp and equilibrium
capture experiments have been removed from the production runtime.

Physical tomatoes keep the explicit historical behavior (external body,
`excludeFromArticulation=true`, break force `6 N`) only behind this warning
gate:

```bash
./run_debugV2.sh --day 160 --organ full \
  --allow-experimental-fruit-physics
```

The day-160 full scene did not settle even with all colliders disabled in
memory and break force `1e9 N`; it is therefore not a supported production
configuration. The method and measurements are in
[`../groimp_bridge/BRANCH_REPORT_2026-08-25.md`](../groimp_bridge/BRANCH_REPORT_2026-08-25.md).

Interactive Isaac runs use 60 Hz by default for the same effective cadence as
the historical V2 loader, while authored/headless validation remains 480 Hz:

```bash
./run_debugV2.sh --day 50
./run_debugV2.sh --day 50 --organ leaves
./run_debugV2.sh --day 50 --organ leaves --leaf-joint-policy optimized
./run_debugV2.sh --day 50 --organ leaves --visual-quality performance
./run_debugV2.sh --day 50 --organ leaves --interactive-physics-hz 120
./run_debugV2.sh --day 50 --organ leaves --headless --physics-hz 480 --duration 5
./run_debugV2.sh --day 50 --organ truss-supports
```

PlantState leaf checkpoints default to `--leaf-joint-policy distributed`:
petioles and main rachides use D6 joints, while petiolules and blades remain
rigid visuals on their support. `optimized` keeps only petioles dynamic and
fixes rachides, preserving the lower-cost checkpoint for larger plants. The
setting does not affect static BRANCHES configurations.

Initial canonical collider overlaps default to precise pair filtering:

```bash
./run_debugV2.sh --day 80 --initial-overlap-policy filter
./run_debugV2.sh --day 80 --initial-overlap-policy error
```

Only the two overlapping rigid bodies are filtered and every pair is recorded
in the manifest. This is permanent for the simulation, so those two bodies can
subsequently pass through each other; use `error` for strict geometry audits.

Petiolules remain visual-only by default. The diagnostic switch below restores
one rigid body, collider set, and D6 joint per petiolule and can be expensive:

```bash
./run_debugV2.sh --day 50 --organ leaves --physical-petiolules
./run_debugV2.sh --day 50 --organ full --physical-petiolules \
  --allow-over-budget --allow-experimental-fruit-physics
```

The second command is deliberately unsafe and intended only for comparison
with the pre-optimization V2 topology.

Incremental PlantState profiles default to `--visual-quality realistic`, the
original V2 skinned/segmented sampling used on stem, laterals, petioles,
rachides and rigid petiolules. `--visual-quality performance` is an explicit
lower-detail fallback; it changes only tessellation, never canonical pose or
physics topology. Canonical leaf dry biomass is aggregated on its supporting
rachis, so visual-only blades still load the branch without additional bodies.

### PlantState mode
```bash
./run_mainV2.sh --day 100
uv run python -m exporterV2 --day 100
```

Day-based execution reads `data/plant_states/plant_state_day_N.json`, defaults
to `truss-supports`, `flexible` and dynamic laterals, and writes
`data/usd_models/tree_v2_day_N.usda`; there is no CSV fallback. The debug
wrapper writes its generated assets under `/tmp` by default.

### Runtime and Debug Configuration

Edit only `core/tree_config.py`. `PhysicsRuntimeConfig` controls physics rate,
solver iterations, and GPU dynamics; `BranchResolutionConfig` sets the
pre-optimization maximum links per chain; `OrganGenerationConfig` enables or
disables complete organ hierarchies. `TrussPhysicsConfig` owns tomato
detachment, overlap-filter policy and the manual tuning values below. The file
itself is the source of truth for current values.

Truss response is intentionally manual rather than auto-tuned. The independent
controls are `TrussPhysicsConfig.RACHIS_YOUNG_MODULUS`,
`PEDICEL_YOUNG_MODULUS`, `RACHIS_DAMPING_RATIO`,
`PEDICEL_DAMPING_RATIO`, and `PEDICEL_DRIVE_STIFFNESS_SCALE`. Regenerate the
asset after changing them. Density, the ±25 degree limit and the 6 N break
force are separate solver/detachment adaptations and should not be changed as
part of ordinary stiffness tuning.

Then regenerate normally:

```bash
./run_mainV2.sh --day 100
./run_mainV2.sh --day 100 --optimize
```

Disabling a parent automatically disables its descendants. The branch limit
preserves total length and child attachment height; optimization may still
reduce the number of links below that maximum.

### Python API
```python
from exporterV2.adapters.groimp_csv import parse_csv_to_branches
from exporterV2.core.usd import build_stage

# Load from CSV (uses tomato profile by default)
branches, json_path = parse_csv_to_branches(day=100)

# Generate USD
stage, stem_path = build_stage("output.usda", branches=branches)
```

### Manual Configuration
```python
from exporterV2.core import tree_config
from exporterV2.core.usd import build_stage

# Define custom tree
tree_config.BRANCHES = [
    {"id": "trunk", "parent": None, "n_links": 10, ...},
    {"id": "branch_1", "parent": "trunk", ...},
]

# Generate USD
stage, stem_path = build_stage("output.usda")
```

---

## Core Modules (Generic)

### `core/tree_config.py`
- `BRANCHES` - Tree configuration format
- `GLOBAL_SCALE` - World-space scaling
- `PhysicsRuntimeConfig` - PhysX runtime defaults
- `BranchResolutionConfig` - Maximum initial chain resolution
- `OrganGenerationConfig` - Hierarchical debug switches
- `validate_branches()` - Configuration validation
- `clamp_radius()` - PhysX stability constraints

### `core/usd/`
- `build_stage()` - USD stage generation
- `branch_chains.py` - articulated rigid chains shared by legacy and truss paths
- `terminal_bodies.py` - tomatoes, curved pedicels, detachment, and terminal filters
- `materials.py` - shared leaf, stem, and fruit materials

### `core/skinning/`
- Vegetative rigid physics and continuous visual-axis resolution
- Preserved `segmented`, `skinned`, `static`, and `rigid-single` visual modes
- Procedural leaf blades and terminal leaf-only visual dressing

### `core/physics.py`
- PhysX scene settings for Isaac Sim
- Articulation configuration

---

## Adapters (Data Sources)

### `adapters/groimp_csv/`
Parses groIMP CSV export files and converts to generic BRANCHES format.

**Functions:**
- `parse_csv_to_branches(day, plant_id, profile)` - Complete pipeline
- `load_trunk_internodes()` - Load trunk data
- `load_lateral_branches()` - Load lateral branches with filtering
- `load_leaves()` - Load leaves with filtering
- `load_trusses()` - Load CSV-derived trusses, pedicels, and tomato metadata

**Profile-driven:**
- Filtering logic controlled by cultivar profile
- Default: tomato profile with opposite pair filtering

---

## Profiles (Cultivar-Specific)

### `profiles/tomato_default.py`

Configuration for standard tomato plant:
- Lateral branches: opposite pairs (organ_index 0+1), 45° tilt
- Trunk leaves: 180° opposite pair filtering
- Lateral leaves: clone missing, random orientation

**Create your own profile:**
```python
MY_PROFILE = {
    "lateral_branches": {
        "organ_indices": [0, 1, 2, 3],  # All 4 branches
        "tilt_deg": 60.0,                # Different tilt
    },
    # ...
}

branches, _ = parse_csv_to_branches(day=100, profile=MY_PROFILE)
```

---

## Output

### JSON Configuration
```json
{
  "metadata": {
    "day": 100,
    "n_branches": 108,
    "total_links": 133,
    "profile": "Tomato Default"
  },
  "branches": [...]
}
```

### USD Stage
- Articulated physics (PhysX)
- Flexible joints with automatic spring/damping
- Targeted branch and terminal-body collision filtering
- Detachable tomato terminal bodies under `/World/TerminalBodies`
- Breakable FixedJoints for tomato detachment
- Compatible with Isaac Sim

---

## Documentation

Comprehensive documentation in `docs/`:

- **[Architecture](docs/01_architecture.md)** - How ExporterV2 works (pipeline, modules, data flow)
- **[Physics & Mechanics](docs/02_physics_and_mechanics.md)** - Physical paradigms, joint mechanics, and collision setup
- **[vs V1](docs/03_vs_v1.md)** - Differences from V1, migration guide, when to use each
- **[CSV Modifications](docs/04_csv_modifications.md)** - How we deviate from raw CSV (angles, filtering, jitter)
- **[Collision Checks](docs/05_collision_checks.md)** - Anti-collision system for lateral branches
- **[Testing](docs/06_testing.md)** - Test suite, how to run, expected results
- **[Implementation Notes](docs/07_implementation_notes.md)** - Lessons learned, tricks, common pitfalls
- **[Troubleshooting](docs/08_troubleshooting.md)** - Common issues and solutions
- **[Vegetative Visual Modes](docs/09_segmented_branch_visuals.md)** - Organic geometry, mode ownership, and running options

---

## Testing

### Quick Tests
```bash
# Test with different days
./run_mainV2.sh --day 1
./run_mainV2.sh --day 50
./run_mainV2.sh --day 160

# Check output
ls output/day_100/
cat output/day_100/branches_v2_day_100.json
```

### Automated Test Suite
```bash
# Run all tests from project root
cd /home/alessandro/isaacsim/autotom_digital_twin

# Ordinary pytest suite
uv run pytest src/exporterV2/adapters/groimp_csv/tests \
              src/exporterV2/core/optimizations/tests \
              src/exporterV2/tests -v

# Isaac/PhysX tests
~/isaacsim/python.sh -m pytest src/exporterV2/core/usd/tests -v
```

See **[tests/README.md](tests/README.md)** for details.

---

## Historical Notes

The old refactoring reports have been consolidated into `docs/`. The current
reference points are:

- `docs/01_architecture.md` for the active pipeline and module boundaries.
- `docs/07_implementation_notes.md` for implementation decisions, conservative
  refactors, CSV single-read behavior, config caching, and detachment notes.
- `docs/06_testing.md` for the split between ordinary tests, Isaac/PhysX tests,
  and manual demos.

---

## Related

- **exporterV1:** Original static baseline exporter
- **recursive_tree:** Generic tree experiments (uses core/ directly)
- **example_custom_tree.py:** Manual BRANCHES configuration example

---

## Maintainer
Alessandro - Digital Twin Project
