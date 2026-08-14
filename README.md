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
rigid tomato bodies, detachment joints, collision filtering, and joint-budget
optimization. V1 remains useful as a simpler static reference.

| Exporter | Role | Main capabilities |
| --- | --- | --- |
| V1, `src/exporterV1` | Legacy baseline | Static USD reconstruction, colored geometry, simple inspection pipeline |
| V2, `src/exporterV2` | Active pipeline | CSV-derived articulated plant, compound leaves, physical trusses, detachable tomatoes, runtime PhysX settings, optimizer |

### Exporter V2 Demo

<video src="assets/demo_v2.mp4" width="100%" controls muted loop>
</video>

[Open `assets/demo_v2.mp4`](assets/demo_v2.mp4)

### Exporter V1 / V1.5 Baseline

<video src="assets/demo_v1_5.mov" width="100%" controls muted loop>
</video>

[Open `assets/demo_v1_5.mov`](assets/demo_v1_5.mov)

## Pipeline

```mermaid
graph LR
    A[GroIMP growth simulation] --> B[Daily CSV graph export]
    B --> C[Exporter V2 CSV adapter]
    C --> D[Branch, leaf, truss, tomato model]
    D --> E[Optional joint-budget optimizer]
    E --> F[OpenUSD / Isaac Sim scene]
```

1. GroIMP exports daily graph data under `data/simulation_output/dynamic_output/graphs/`.
2. Exporter V2 reads each CSV once, reconstructs topology, and converts the graph into a profile-driven branch representation.
3. Leaf and truss builders add compound leaves, rachis links, one-link pedicels, tomato sizing, maturity colors, and debug switches.
4. The USD stage authoring layer creates geometry, materials, articulated joints, terminal tomato bodies, collision filters, and PhysX runtime settings.
5. The optimizer can reduce the number of simulated joints while preserving a compatible USD structure for Isaac Sim.

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
- `core/`: generic configuration, mechanics, USD stage construction, and optimization.
- `profiles/`: tomato-specific biological and geometric assumptions.
- `demos/`: manual visual validation scripts for videos, screenshots, and paper figures.
- `docs/`: canonical technical documentation for architecture, physics, tests, and troubleshooting.

## Current V2 Features

| Category | Feature | Status |
| --- | --- | :---: |
| Architecture | Modular core/adapters/profiles pipeline | Done |
| Parsing | Single-read CSV topology reconstruction | Done |
| Geometry | Trunk, lateral branches, compound leaves, trusses, pedicels, tomatoes | Done |
| Visuals | Organ coloring and tomato ripening colors | Done |
| Physics | Articulated branch links with PhysX D6 drives | Done |
| Trusses | CSV-derived rachis, one-link pedicels, detachable terminal tomatoes | Done |
| Detachment | Breakable FixedJoints under `/World/TerminalBodies` | Done |
| Collision | Geometric validation plus runtime tomato-pedicel-rachis filters | Done |
| Optimization | Joint-budget LOD techniques and visual validation demos | Done |
| Runtime | Isaac Sim scene settings, GPU dynamics, solver iterations | Done |

## Running

Use V2 for current work:

```bash
./run_mainV2.sh --day 100
./run_mainV2.sh --day 100 --optimize
```

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

The optimizer also has focused documentation under
`src/exporterV2/core/optimizations/docs/`.

## Notes for Reproducibility

- Generated outputs under `output/` are preserved as artifacts, but code changes
  should not silently alter them during cleanup or documentation work.
- Python bytecode, cache directories, and scratch USDA files are ignored.
- Manual demo scripts are intentionally outside normal pytest collection so
  paper/video assets do not interfere with automated validation.

### USD IDE Stubs

The `typings/` and `.vscode/` folders provide local USD type stubs for editor
completion in VSCode. Keep them when working on USD authoring code.
