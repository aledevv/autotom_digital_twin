# AutoTom Digital Twin

[![Version](https://img.shields.io/badge/version-v2.2-blue.svg)](#)
[![Python](https://img.shields.io/badge/python-3.14%2B-blue.svg)](https://www.python.org/)
[![USD](https://img.shields.io/badge/USD-OpenUSD-green.svg)](https://openusd.org/)
[![Isaac Sim](https://img.shields.io/badge/Isaac%20Sim-PhysX-nvidia.svg)](https://developer.nvidia.com/isaac-sim)
[![Dependency Manager](https://img.shields.io/badge/dependency_manager-uv-purple.svg)](https://github.com/astral-sh/uv)

AutoTom Digital Twin converts GroIMP dwarf-tomato growth simulations into
OpenUSD scenes for inspection and physics simulation in NVIDIA Isaac Sim. The
repository contains two exporter generations: V1 is retained as a static visual
baseline, while **Exporter V2.2** is the active physics-oriented and
appearance-oriented pipeline used for current experiments, videos, and final
reporting.

The central research goal is to preserve the organ-level topology produced by
the plant model while making the result usable in an interactive simulator:
stems, leaves, trusses, pedicels, and tomatoes are reconstructed from CSV graph
data, assigned USD geometry and materials, and optionally equipped with
articulated PhysX dynamics.

## Visual Comparison

Exporter V2 introduced articulated branch dynamics, physical trusses, rigid
tomato bodies, detachment joints, collision filtering, joint-budget
optimization, and organic branch rendering. **V2.2 extends that pipeline with a
visual-realism pass**: more organic vegetative geometry, 3D leaflet blades,
procedural gravity-shaped pedicels, and shared PBR materials for leaves, stems,
and fruits.

| Exporter | Role | Main capabilities |
| --- | --- | --- |
| V1, `src/exporterV1` | Legacy baseline | Static USD reconstruction, colored geometry, simple inspection pipeline |
| V2.2, `src/exporterV2` | Active pipeline | CSV-derived articulated plant, organic branch visuals, 3D compound leaves, physical trusses, detachable tomatoes, PBR organ materials, runtime PhysX settings, optimizer |

### Exporter V2.0 Demo
https://github.com/user-attachments/assets/357638f1-c9d2-485e-94ee-cd6e2d5e7535

### Exporter V1 / V1.5 Baseline
https://github.com/user-attachments/assets/ab07fde7-d866-470d-9056-e0a2215c22d8

## V2.2 — Realism Upgrade

V2.2 keeps the existing physical model and focuses on improving the visual
fidelity of the same GroIMP-derived plant. The main additions are:

- **Organic vegetative axes:** smooth radius transitions, local taper, root
  flare, junction bulges, terminal-fork shaping, and realtime segmented visual
  meshes built independently from the rigid collision proxies.
- **3D tomato leaflets:** compound-leaf blades use a cultivar-inspired width
  profile together with a longitudinal fold, gentle arch, and static gravity
  sag instead of a flat planar blade.
- **Procedural pedicel appearance:** the physical pedicel proxy can remain
  simple while a separate cubic gravity-curved visual tube provides a more
  natural fruit attachment, including deterministic small radius/side
  variations.
- **Shared organ materials:** realtime `UsdPreviewSurface` materials are reused
  for leaves, vegetative axes, and tomatoes; leaves and stems also expose an
  optional heavier `OmniSurface` look-development preset.
- **Ripening-aware tomato materials:** fruit color remains driven by maturation,
  while eight shared material buckets bound the number of authored shaders.
- **Visual/physics separation:** higher-fidelity appearance geometry is kept
  separate from the simple PhysX representation wherever practical, so the
  realism pass does not require a corresponding increase in collision
  complexity.

### V2.2 — Appearance and Geometry Demo
<!-- Add/embed the video from assets/demo_v2_2.mp4 here. -->

### V2.2 — Gravity Response Demo
<!-- Add/embed the video from assets/demo_v2_2_gravity.mp4 here. -->

## V2.2 Gallery

<p align="center">
  <img src="assets/gallery/Screenshot%20from%202026-08-20%2014-14-53.png" width="48%" alt="AutoTom V2.2 plant render">
  <img src="assets/gallery/Screenshot%20from%202026-08-20%2014-15-15.png" width="48%" alt="AutoTom V2.2 plant render">
</p>
<p align="center">
  <img src="assets/gallery/Screenshot%20from%202026-08-20%2014-15-45.png" width="48%" alt="AutoTom V2.2 detail render">
  <img src="assets/gallery/Screenshot%20from%202026-08-20%2014-16-04.png" width="48%" alt="AutoTom V2.2 detail render">
</p>
<p align="center">
  <img src="assets/gallery/Screenshot%20from%202026-08-20%2010-15-48.png" width="48%" alt="AutoTom leaf and branch detail">
  <img src="assets/gallery/Screenshot%20from%202026-08-20%2011-38-32.png" width="48%" alt="AutoTom tomato truss detail">
</p>
<p align="center">
  <img src="assets/gallery/Screenshot%20from%202026-08-20%2014-27-05.png" width="48%" alt="AutoTom V2.2 plant render">
  <img src="assets/gallery/Screenshot%20from%202026-08-20%2014-29-48.png" width="48%" alt="AutoTom V2.2 plant render">
</p>

## Pipeline

```mermaid
graph LR
    A[GroIMP growth simulation] --> B[Daily CSV graph export]
    B --> C[Exporter V2.2 CSV adapter]
    C --> D[Branch, leaf, truss, tomato model]
    D --> E[Optional joint-budget optimizer]
    E --> F[PhysX branch graph]
    F --> G[Organic visual + material backend]
    G --> H[OpenUSD / Isaac Sim scene]
```

1. GroIMP exports daily graph data under `data/simulation_output/dynamic_output/graphs/`.
2. Exporter V2.2 reads each CSV once, reconstructs topology, and converts the graph into a profile-driven branch representation.
3. Leaf and truss builders add compound 3D leaflets, rachis links, one-link pedicels, tomato sizing, maturity colors, and debug switches.
4. The USD stage authoring layer creates articulated physics links, joints, collision proxies, terminal tomato bodies, shared organ materials, and the selected visual representation.
5. The visual layer adds organic branch shaping and, where relevant, higher-fidelity visual-only geometry such as gravity-curved pedicels.
6. The optimizer can reduce the number of simulated joints while preserving a compatible USD structure and an optimizer-independent visual profile.

## Realtime Organic Branch Rendering

V2.2 keeps physical links and collision proxies independent from organic branch
surfaces. The preserved visual choices are:

| Mode | Role |
| --- | --- |
| `segmented` | Current realtime default; one organic mesh per rigid link |
| `skinned` | Continuous UsdSkel deformation reference |
| `static` | Smooth geometry benchmark |
| `rigid-single` | One-link runtime diagnostic |
| `legacy` backend | Original cylinder branch representation |

The modes share botanical taper, radius transitions, leaf blades, and material
authoring without changing plant physics. See
`src/exporterV2/docs/09_segmented_branch_visuals.md` for geometry, performance
rationale, terminal leaf junctions, and module ownership.

## Repository Layout

```text
.
├── assets/                         # Demo videos and visual assets
│   └── gallery/                    # V2.2 screenshots used in the README
├── data/                           # GroIMP input data and preserved datasets
├── output/                         # Generated day outputs
├── run_main.sh                     # Legacy V1 runner
├── run_mainV2.sh                   # Active V2.2 runner
└── src/
    ├── exporterV1/                 # Static baseline exporter
    ├── exporterV2/                 # Active modular exporter
    └── experiments/                # Focused validation and mechanics experiments
```

Exporter V2.2 is organized around reusable layers:

- `adapters/groimp_csv/`: CSV parsing and conversion from GroIMP graph nodes.
- `core/`: generic configuration, mechanics, USD stage construction, skinning/segmented visuals, material authoring, and optimization.
- `profiles/`: tomato-specific biological and geometric assumptions.
- `demos/`: manual visual validation scripts for videos, screenshots, and paper figures.
- `docs/`: canonical technical documentation for architecture, physics, visuals, tests, and troubleshooting.

## Current V2.2 Features

| Category | Feature | Status |
| --- | --- | :---: |
| Architecture | Modular core/adapters/profiles pipeline | Done |
| Parsing | Single-read CSV topology reconstruction | Done |
| Geometry | Trunk, lateral branches, compound leaves, trusses, pedicels, tomatoes | Done |
| Geometry | 3D leaflet blades with width profile, fold, arch, and gravity sag | Done |
| Visuals | Organic swept branch geometry with taper, smooth radius transitions, root flare, and junction shaping | Done |
| Visuals | Segmented realtime rendering with one organic mesh per physical link | Done |
| Visuals | Optional continuous UsdSkel branch skinning | Done |
| Visuals | Centered terminal leaf continuation and small deterministic visual fork | Done |
| Visuals | Gravity-curved procedural pedicel mesh with deterministic visual variation | Done |
| Materials | Shared realtime PBR materials for leaves, vegetative axes, and fruits | Done |
| Materials | Optional OmniSurface look-development presets for leaves and stems | Done |
| Materials | Maturation-aware tomato color with eight shared material buckets | Done |
| Physics | Articulated branch links with PhysX D6 drives | Done |
| Trusses | CSV-derived rachis, one-link pedicels, detachable terminal tomatoes | Done |
| Detachment | Breakable FixedJoints under `/World/TerminalBodies` | Done |
| Collision | Geometric validation plus runtime tomato-pedicel-rachis filters | Done |
| Optimization | Joint-budget LOD techniques and visual validation demos | Done |
| Runtime | Isaac Sim scene settings, GPU dynamics, solver iterations | Done |

## Running

Use V2.2 for current work. The skinned branch backend and realtime segmented
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

Ordinary V2.2 tests can run in the project virtual environment:

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

The canonical V2.2 documentation is in `src/exporterV2/docs/` and the codebase
itself now also contains the appearance/material implementation introduced by
the realism pass:

- `01_architecture.md`: data flow, parser, builders, USD stage, optimizer.
- `02_physics_and_mechanics.md`: PhysX runtime, truss profile, detachment, filters.
- `03_vs_v1.md`: technical comparison between the static baseline and V2.
- `04_csv_modifications.md`: profile assumptions and CSV-derived organ generation.
- `05_collision_checks.md`: geometric checks and runtime collision filtering.
- `06_testing.md`: ordinary tests, Isaac tests, demos, and runners.
- `07_implementation_notes.md`: refactor decisions and consolidated engineering notes.
- `08_troubleshooting.md`: common stability, detachment, and import issues.
- `09_segmented_branch_visuals.md`: organic sweep geometry, segmented realtime rendering, terminal leaf forks, and performance rationale.
- `core/skinning/leaf_blade.py`: V2.2 3D leaflet geometry and gravity-shaped blade appearance.
- `core/usd/pedicel_geometry.py`: V2.2 procedural gravity-curved pedicel visual geometry.
- `core/usd/materials.py`: V2.2 shared leaf, stem, and fruit material authoring.

The optimizer also has focused documentation under
`src/exporterV2/core/optimizations/docs/`.

## Notes for Reproducibility

- Generated outputs under `output/` are preserved as artifacts, but code changes
  should not silently alter them during cleanup or documentation work.
- Python bytecode, cache directories, and scratch USDA files are ignored.
- Manual demo scripts are intentionally outside normal pytest collection so
  paper/video assets do not interfere with automated validation.
- The deterministic terminal-fork and pedicel visual variations are keyed from
  branch identifiers, so repeated exports of the same topology keep the same
  visual orientation/variation.
- V2.2 material parameters are appearance parameters rather than measured plant
  optical constants; the material system is literature-informed but currently
  tuned empirically for Isaac Sim rendering.

### USD IDE Stubs

The `typings/` and `.vscode/` folders provide local USD type stubs for editor
completion in VSCode. Keep them when working on USD authoring code.
