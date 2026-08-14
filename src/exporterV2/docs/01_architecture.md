# Exporter V2 Architecture

Exporter V2 is the active pipeline for generating articulated tomato-plant USD scenes from GroIMP CSV exports. The design separates data adaptation, plant morphology, physics authoring, optimization, and demos so that production generation stays stable while experiments remain easy to inspect.

## Current Pipeline

1. `parse_csv_to_branches()` reads the GroIMP graph CSV once per run and shares the same DataFrame with the trunk, lateral branch, leaf, and truss loaders.
2. The tomato profile and `OrganGenerationConfig` decide which organs are generated: trunk, laterals, petioles, leaf rachis, petiolules, truss rachis, pedicels, and tomatoes.
3. Leaf and truss builders convert CSV organs into the generic `BRANCHES` list plus terminal tomato metadata.
4. `build_stage()` authors the USD stage: rigid segment meshes, D6 or Fixed joints, collision filters, terminal bodies, runtime PhysX settings, and optional optimization output.
5. The optimizer can reduce D6 joint count through the current techniques while preserving generated geometry and attachment semantics.

## Main Components

- `core/tree_config.py` contains runtime physics defaults, organ-generation switches, geometry constants, material colors, and mechanical helper functions.
- `adapters/groimp_csv/` converts GroIMP CSV rows into the generic branch schema. The public loaders still work standalone; internally the full pipeline avoids repeated CSV reads.
- `core/usd/` builds OpenUSD/PhysX scene structure. It owns chain construction, joint authoring, collision filtering, terminal tomato bodies, and runtime scene settings.
- `core/optimizations/` contains the joint-budget optimizer and individual techniques such as petiole lock, lateral reduce, thin-link lock, leaf-branch reduce, stem collapse, and truss static handling.
- `demos/` contains visual/Isaac assets for paper figures, videos, and manual inspection. These are intentionally outside the pytest suite.

## Stage Structure

The generated stage uses `/World` as the default prim and `/World/Stem` as the main articulation root. Regular plant links are authored under `/World/Stem`. Detachable tomatoes are regular rigid bodies under `/World/TerminalBodies` and are connected back to the pedicel tip through breakable FixedJoints.

Terminal tomato bodies keep:

- `PhysxRigidBodyAPI`
- solver iterations from `PhysicsRuntimeConfig.TERMINAL_BODY_SOLVER_*`
- `physics:excludeFromArticulation = True`
- collision filters to the pedicel and truss rachis that generated them

## Design Rules

- CSV-derived geometry, organ order, prim paths, and generated metadata are treated as compatibility surfaces.
- Truss, pedicel, and tomato structure comes from CSV-derived configuration, not from synthetic test geometry.
- Demos and visual validation scripts are kept separate from automated tests so normal pytest runs stay fast and deterministic.

## Documentation Consolidation

Historical root summaries and task-by-task refactoring reports have been folded
into this documentation set. `README.md` now gives the academic overview and V1
vs V2 comparison, while these V2 docs hold the implementation details that are
needed for final reporting.
