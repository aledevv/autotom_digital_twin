# AutoTom Digital Twin

[![Version](https://img.shields.io/badge/version-v2.0-blue.svg)](#)
[![Python](https://img.shields.io/badge/python-3.14%2B-blue.svg)](https://www.python.org/)
[![USD](https://img.shields.io/badge/USD-OpenUSD-green.svg)](https://openusd.org/)
[![Isaac Sim](https://img.shields.io/badge/Isaac%20Sim-PhysX-nvidia.svg)](https://developer.nvidia.com/isaac-sim)
[![Dependency Manager](https://img.shields.io/badge/dependency_manager-uv-purple.svg)](https://github.com/astral-sh/uv)

AutoTom Digital Twin converts GroIMP dwarf-tomato growth simulations into
OpenUSD scenes for inspection and physics simulation in NVIDIA Isaac Sim. The
repository contains two exporter generations: V1 is retained as a static visual
baseline, while V2 is the active physics-oriented pipeline used for current
experiments, videos, and final reporting.

The central research goal is to preserve the organ-level topology produced by
the plant model while making the result usable in an interactive simulator:
stems, leaves, trusses, pedicels, and tomatoes are reconstructed from CSV graph
data, assigned USD geometry and materials, and optionally equipped with
articulated PhysX dynamics.

## Visual Comparison

V2 adds articulated branch dynamics, colored organ materials, truss geometry,
rigid tomato bodies, detachment joints, collision filtering, joint-budget
optimization, and organic branch rendering. V1 remains useful as a simpler
static reference.

| Exporter | Role | Main capabilities |
| --- | --- | --- |
| V1, `src/exporterV1` | Legacy baseline | Static USD reconstruction, colored geometry, simple inspection pipeline |
| V2, `src/exporterV2` | Active pipeline | CSV-derived articulated plant, organic branch visuals, compound leaves, physical trusses, detachable tomatoes, runtime PhysX settings, optimizer |

### Exporter V2 Demo
https://github.com/user-attachments/assets/357638f1-c9d2-485e-94ee-cd6e2d5e7535

### Exporter V1 / V1.5 Baseline
https://github.com/user-attachments/assets/ab07fde7-d866-470d-9056-e0a2215c22d8

## Pipeline

```mermaid
graph LR
    A[GroIMP growth simulation] --> B[Daily CSV graph export]
    B --> C[Exporter V2 CSV adapter]
    C --> D[Branch, leaf, truss, tomato model]
    D --> E[Optional joint-budget optimizer]
    E --> F[PhysX branch graph]
    F --> G[Organic visual backend]
    G --> H[OpenUSD / Isaac Sim scene]
```

1. GroIMP exports daily graph data under `data/simulation_output/dynamic_output/graphs/`.
2. Exporter V2 reads each CSV once, reconstructs topology, and converts the graph into a profile-driven branch representation.
3. Leaf and truss builders add compound leaves, rachis links, one-link pedicels, tomato sizing, maturity colors, and debug switches.
4. The USD stage authoring layer creates articulated physics links, joints, collision proxies, terminal tomato bodies, and the selected visual representation.
5. The optimizer can reduce the number of simulated joints while preserving a compatible USD structure and an optimizer-independent visual profile.

## Realtime Organic Branch Rendering

V2 now separates **physics discretization** from **visual geometry**. PhysX still
uses articulated rigid links and hidden capsule collision proxies, while the
visible stem/branch surface is generated from a smooth swept-tube profile with
taper, radius transitions, junction bulges, and local overlap between adjacent
rigid pieces.

The default visual mode is `segmented`. Each PhysX link owns one rigid piece of
the organic mesh, so the visual surface follows physics directly without
`UsdSkel`, `SkelAnimation`, or a per-frame skinning synchronization step. Small
visual overlaps at internal joints reduce visible cracks during bending.

Terminal lateral branches receive an additional leaf-only visual treatment. A
terminal petiole is centered on the structural branch centerline and acts as the
real continuation of the branch. The host tip is shaped around that contact and
a very small rigid young twig with a leaf is added as a secondary visual fork.
The twig azimuth is deterministically varied around the branch axis to avoid
repeating the same fork orientation. Truss/tomato terminal geometry is
intentionally excluded from this visual-fork logic.

The original `skinned` mode remains available as the high-quality continuous
deformation reference. It uses UsdSkel and is useful for focused demonstrations
and comparisons, while `segmented` is the realtime-oriented representation for
full plants.

Observed day-40 interactive results during development were approximately:

| Visual configuration | Observed FPS | Purpose |
| --- | ---: | --- |
| Legacy cylinder visuals | 18–20 | Original realtime baseline |
| Static smooth organic mesh | 19–20 | Geometry-cost isolation test |
| Full UsdSkel skinning | ~8 | Continuous high-quality deformation |
| Shared/global Skeleton experiment | ~10 | Skinning architecture diagnostic |
| Segmented organic visuals | ~20 | Current realtime visual representation |

These values are empirical development measurements and depend on scene,
hardware, renderer, and simulation settings. The important result is that the
smooth geometry itself was not the dominant cost; runtime UsdSkel deformation
was. The segmented representation therefore preserves the organic geometry
while avoiding that runtime cost.

See `src/exporterV2/docs/09_segmented_branch_visuals.md` for the implementation
details and geometry equations.

## Repository Layout

```text
.
├── assets/                         # Demo videos and visual assets
├── data/                           # GroIMP input data and preserved datasets
├── output/                         # Generated day outputs
├── run_main.sh                     # Legacy V1 runner
├── run_mainV2.sh                   # Active V2 runner
└── src/
    ├── exporterV1/                 # Static baseline exporter
    ├── exporterV2/                 # Active modular exporter
    └── experiments/                # Focused validation and mechanics experiments
```

Exporter V2 is organized around reusable layers:

- `adapters/groimp_csv/`: CSV parsing and conversion from GroIMP graph nodes.
- `core/`: generic configuration, mechanics, USD stage construction, skinning/segmented visuals, and optimization.
- `profiles/`: tomato-specific biological and geometric assumptions.
- `demos/`: manual visual validation scripts for videos, screenshots, and paper figures.
- `docs/`: canonical technical documentation for architecture, physics, visuals, tests, and troubleshooting.

## Current V2 Features

| Category | Feature | Status |
| --- | --- | :---: |
| Architecture | Modular core/adapters/profiles pipeline | Done |
| Parsing | Single-read CSV topology reconstruction | Done |
| Geometry | Trunk, lateral branches, compound leaves, trusses, pedicels, tomatoes | Done |
| Visuals | Organic swept branch geometry with taper, smooth radius transitions, segmented realtime rendering | Done |
| Visuals | Optional continuous UsdSkel branch skinning | Done |
| Visuals | Centered terminal leaf continuation and small deterministic visual fork | Done |
| Physics | Articulated branch links with PhysX D6 drives | Done |
| Trusses | CSV-derived rachis, one-link pedicels, detachable terminal tomatoes | Done |
| Detachment | Breakable FixedJoints under `/World/TerminalBodies` | Done |
| Collision | Geometric validation plus runtime tomato-pedicel-rachis filters | Done |
| Optimization | Joint-budget LOD techniques and visual validation demos | Done |
| Runtime | Isaac Sim scene settings, GPU dynamics, solver iterations | Done |

## Running

Use V2 for current work. The skinned branch backend and realtime segmented
organic visuals are the current defaults:

```bash
./run_mainV2.sh --day 40
./run_mainV2.sh --day 40 --optimize
```

The equivalent explicit command is:

```bash
./run_mainV2.sh \
  --day 40 \
  --branch-backend skinned \
  --skinning-visual-mode segmented
```

For the continuous UsdSkel reference mode:

```bash
./run_mainV2.sh \
  --day 40 \
  --branch-backend skinned \
  --skinning-visual-mode skinned
```

Diagnostic visual modes `static` and `rigid-single` are also available for
isolating rendering/skinning behavior.

Use V1 only for the legacy static baseline:

```bash
./run_main.sh
```

Experiment-specific runners live beside their experiments, for example
`src/experiments/cantilever_test/run_cantilever_test.sh` and
`src/experiments/three_point_test/run_threepoint_test.sh`.

## Testing

Ordinary V2 tests can run in the project virtual environment:

```bash
uv run pytest \
  src/exporterV2/adapters/groimp_csv/tests \
  src/exporterV2/core/optimizations/tests \
  src/exporterV2/tests \
  -v
```

USD/PhysX tests that require Isaac Sim modules should run with Isaac's Python:

```bash
~/isaacsim/python.sh -m pytest \
  src/exporterV2/core/usd/tests \
  src/exporterV2/core/optimizations/tests/11_truss_static/test_truss_static_usd.py \
  -v
```

## Documentation

The canonical V2 documentation is in `src/exporterV2/docs/`:

- `01_architecture.md`: data flow, parser, builders, USD stage, optimizer.
- `02_physics_and_mechanics.md`: PhysX runtime, truss profile, detachment, filters.
- `03_vs_v1.md`: technical comparison between the static baseline and V2.
- `04_csv_modifications.md`: profile assumptions and CSV-derived organ generation.
- `05_collision_checks.md`: geometric checks and runtime collision filtering.
- `06_testing.md`: ordinary tests, Isaac tests, demos, and runners.
- `07_implementation_notes.md`: refactor decisions and consolidated engineering notes.
- `08_troubleshooting.md`: common stability, detachment, and import issues.
- `09_segmented_branch_visuals.md`: organic sweep geometry, segmented realtime rendering, terminal leaf forks, and performance rationale.

The optimizer also has focused documentation under
`src/exporterV2/core/optimizations/docs/`.

## Notes for Reproducibility

- Generated outputs under `output/` are preserved as artifacts, but code changes
  should not silently alter them during cleanup or documentation work.
- Python bytecode, cache directories, and scratch USDA files are ignored.
- Manual demo scripts are intentionally outside normal pytest collection so
  paper/video assets do not interfere with automated validation.
- The deterministic terminal-fork variation is keyed from branch identifiers,
  so repeated exports of the same topology keep the same visual orientation.

### USD IDE Stubs

The `typings/` and `.vscode/` folders provide local USD type stubs for editor
completion in VSCode. Keep them when working on USD authoring code.
