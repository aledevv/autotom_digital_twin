# Follow-up fix notes: physics_cfg injection (see CHANGELOG.md §8 for full detail)

Quick reference for what changed in this follow-up pass, requested after
independent re-verification found `physics_cfg` was unreachable from real
entry points.

## Files touched
- `usd_exporter_builder.py` (+ synced `baseline_test/plant_model/usd_exporter_builder.py`)
- `main_builder.py` (+ synced `baseline_test/plant_model/main_builder.py`)
- `baseline_test/run_v2.py`
- `CHANGELOG.md` (new §8 appended)

## What changed
1. `build_plant_stage(snapshot, output_path, physics_cfg=None)` — new optional
   3rd parameter, threaded into `PlantBuilder(stage, stem_path, physics_cfg=physics_cfg)`.
2. `main_builder.py::main()` — `PlantPhysicsConfig(...)` now constructed
   inline near the top of `main()`, hand-editable, no CLI/argparse wiring
   (per explicit instruction). Passed to `build_plant_stage(...)`.
3. `baseline_test/run_v2.py` — now explicitly builds and passes a
   `PlantPhysicsConfig` through the new parameter.

## Verification results
- v1: byte-identical (md5 `e3761be5cf65f86003cc8c42b74c869d`) + `compare_usda.py`
  reports zero semantic differences. v1 code path untouched.
- v2: still 13 leaves / 1 lateral branch chain after the change.
- Standalone injection test (`/tmp/v2test_cfg_injection/test_cfg_injection.py`):
  - `PlantPhysicsConfig()` default → 13/13 petiole prims get `RigidBodyAPI`.
  - `PlantPhysicsConfig(leaf_petiole_dynamic=False)` passed via `physics_cfg=`
    → 0/13 petiole prims get `RigidBodyAPI`.
  - `build_plant_stage(snapshot, out_path)` with no `physics_cfg` arg at all
    → still defaults to 13/13 (backward compatible).

This proves the config is now actually reachable and functionally effective
from the real entry point, not just structurally present in `plant_builder.py`.
