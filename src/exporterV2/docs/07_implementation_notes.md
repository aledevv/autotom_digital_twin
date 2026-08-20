# Implementation Notes

## Compatibility Rules

- Treat CSV organ order, branch identifiers, prim paths, topology, transforms, joint frames, material bindings, and collision relationships as stable output.
- Keep `legacy`, `segmented`, `skinned`, `static`, and `rigid-single` available.
- Keep generated USD/JSON artifacts out of source-only refactors.
- Use `tree_config.py` as the only source of current physical values.

## Module Responsibilities

- `stage.py` validates inputs, selects the backend, resolves branch transforms, and coordinates authoring.
- `branch_chains.py` owns rigid chain geometry, mass, drive gains, joints, and link colors.
- `terminal_bodies.py` owns terminal authoring, curved pedicels, detachment, clearance checks, and terminal filters.
- `materials.py` creates shared PreviewSurface materials and optional OmniSurface look-development presets.
- `skinning/builder.py` coordinates vegetative physics, visual axes, leaf blades, and the selected visual mode.
- `visual_segmented.py`, `visual_static.py`, and `visual_rigid.py` isolate non-UsdSkel implementations. `mesh.py` owns common radius sampling and the UsdSkel implementation.
- `mesh_geometry.py` contains topology helpers shared by procedural tube meshes.

## Preserved Design Decisions

- Vegetative and truss systems share a stage but keep separate authoring paths.
- Detachable tomatoes are moved into USD terminal-body space only during stage authoring; the biological hierarchy remains in branch metadata.
- Leaf blades remain separate visual meshes. Historical terminal leaf meshes are retained where generated because removing them would change the stage.
- The shared/global skeleton experiment is not part of the maintained backend set; the per-axis reference mode and segmented default remain available.
- Terminal visual variation is deterministic from branch identifiers.

## Verification

Source refactors should compare representative USDA serializations before and after in every backend/mode, then run the USD, skinning, GroIMP adapter, and optimizer tests. Isaac-dependent checks must use `~/isaacsim/python.sh`.
