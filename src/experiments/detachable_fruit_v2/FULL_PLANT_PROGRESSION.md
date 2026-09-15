# v2.3 vegetation with original main-derived trusses: experimental progression

User authorizes all necessary tests toward complete vegetation, retaining the
prior decision: trusses only on the main stem, no lateral trusses. Baseline is
PGS/CPU60 Hz, articulation32/4, fruit32/1, native force50,3 N from startup,
GUI real-time pacing. No PhysX source/binary edits or custom mouse controller.

Fresh day160 PlantState scaffolds use the existing exporter, canonical pose,
default vegetation policies and Gaussian leaf shapes (seed42). Intermediate
rank6-standard bodies are never simulated as the claimed original-rank scene.
`graft_original_trusses.py` replaces them with each rank's100 reference bodies
(20 per truss) from the previously verified five-truss fixture. Four lateral
standard trusses are removed entirely. Each direct root retains the v2.3 parent
and attachment location/axis; reference roll relative to gravity is preserved.
All local internal joint frames, drive gains, collider geometry, original fruit
masses and support properties are retained. Root parent frame is recomputed
from the unchanged child joint frame and world attachment. Loaded source COM
and inertia are made explicit, preserving their values. Physical fruit sizes
remain coherent at1000 kg/m3; truss support density20000 kg/m3 is artificial.

The normal exporter and existing preset are unchanged. These are experimental
post-export USD fixtures, not a finished public v2.3 preset. Source scaffold
manifests describe intermediates; final config.v23_graft describes actual
replacements. Existing scaffold overlap filters persist where their bodies
remain; removed-body references are pruned. Final overlap audit: direct154
filtered contacts/0 active; full220 filtered contacts/0 active. Vegetative
body/joint attributes compare exactly before/after graft. The current v2.3 stem
already uses fixed joints; it was not made fixed by this experiment.

Fresh direct stem+five trusses:60 s rest passed. Full vegetation+five direct
trusses:60 s rest passed,181 rigid bodies including40 fruits,131 leaflet visuals.
Remaining: native interaction checks, GUI review and measured FPS/recovery.
Body/pose-derived stability is distinguished from raw solver velocity advisories.

Evidence root: `artifacts/detachable_fruit_v2/full-plant-progression/`.
Builders: `build_v23_graft_scaffold.py`, `graft_original_trusses.py`.
Recorded input orientation helper: `rotate_native_recording.py` supports both
historical IDs and canonical GroIMP IDs, rejecting ambiguous rank mappings.

Preparation correction: the first full export request included allow_over_budget;
the exporter rejected it because that diagnostic flag requires physical
petiolules. It was removed; ordinary default petiolules remained unchanged and
the intermediate export succeeded within its normal budget (179 D6 joints).

## Completed headless progression

All8 requested60 s cases pass: direct rest/native, full rest, full native
on each rank6–10. Every native run has exactly one JOINT_BREAK on its selected
fruit, continuity passes; no other breaks/errors. Final full scene:81 vegetative
rigid bodies,60 truss support bodies,40 fruits. No truss root is attached to a
lateral branch. Gaussian leaf generation reports131 leaflets (mesh count includes
other leaf/support visuals and is not the leaflet count). Units verified1 m/unit,
Z up in source and destination.

`full_plant_results.json` contains completed reports/hashes. GUI and manual FPS
validation remain pending; user availability requested. The scene is promising
but not manually accepted yet.

```bash
UV_CACHE_DIR=/tmp/autotom-uv-cache uv run --no-sync python \
  src/experiments/detachable_fruit_v2/run_prepared_gui.py \
  artifacts/detachable_fruit_v2/full-plant-progression/full-rest --real-time
```
