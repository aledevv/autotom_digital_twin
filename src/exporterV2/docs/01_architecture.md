# Exporter V2 Architecture

Exporter V2 converts GroIMP graph exports into OpenUSD stages for Isaac Sim. Its main invariant is that physical authoring, visual authoring, and source-data adaptation remain separate.

## Pipeline

1. `adapters/groimp_csv/` reads the graph and emits generic branch definitions plus terminal-body metadata.
2. `profiles/` and generation config decide which tomato organs are present.
3. Optional optimization changes the physical joint budget while preserving compatible visual metadata.
4. `core/usd/stage.py` orchestrates stage construction.
5. Isaac Sim applies scene and articulation settings and runs the generated stage.

## Authoring Boundaries

- `core/tree_config.py` is the source of truth for current geometry, mechanics, detachment, collision-filter, material-color, and runtime defaults.
- `core/usd/branch_chains.py` authors articulated rigid chains used by the legacy backend and by legacy truss branches in hybrid scenes.
- `core/usd/terminal_bodies.py` owns tomato and legacy leaf terminal bodies, curved pedicel visuals, breakable attachment joints, clearance validation, and terminal collision filters.
- `core/usd/materials.py` owns shared leaf, stem, and maturation-bucketed fruit materials under `/World/Looks`.
- `core/skinning/` resolves vegetative physics and authors organic visuals independently from truss geometry.
- `core/optimizations/` contains joint-budget techniques. It does not own rendering.

## Backends And Visual Modes

`build_stage()` supports both branch backends:

- `legacy` authors all branches through rigid cylinder chains.
- `skinned` uses the vegetative backend for stems and leaves while retaining the legacy chain authoring required by trusses.

The `skinned` backend supports four visual modes without removing any physical branches:

| Mode | Implementation | Runtime role |
| --- | --- | --- |
| `segmented` | `visual_segmented.py` | Current realtime default; one organic mesh per rigid link |
| `skinned` | `mesh.py` | Continuous UsdSkel reference mode |
| `static` | `visual_static.py` | Smooth world-space benchmark |
| `rigid-single` | `visual_rigid.py` | Direct visual attachment for one-link axes |

`visual_modes.py` only re-exports the non-UsdSkel authoring functions for import compatibility.

## Stage Structure

- `/World` is the default prim.
- `/World/Stem` is the main articulation root.
- `/World/Stem/Vegetative` contains vegetative collision proxies and rigid links.
- `/World/PlantVisual` contains visual-axis roots when the selected mode requires them.
- `/World/TerminalBodies` contains tomatoes excluded from the main articulation when detachment configuration requires it.
- `/World/Looks` contains shared materials.

Generated prim paths, organ order, topology, joint frames, bindings, and collision relationships are compatibility surfaces. Refactors must preserve them unless a behavior change is explicitly requested.
