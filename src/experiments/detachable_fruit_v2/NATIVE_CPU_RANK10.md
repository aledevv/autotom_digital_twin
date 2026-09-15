# Original rank-10 profile, TGS/CPU native interaction candidate

2026-09-15. User approved testing a native solver path that passed the known-weight
control. This case uses the main-derived rank-10 construction and its original
GroIMP/CSV fruit masses, not the repeated rank-6 profile. Complete stem and one
truss, eight fruits totalling 57.225134 g. This is not yet full-plant v2.3 validation.

Change only `physxScene:enableGPUDynamics` from true to false. TGS, 60 Hz,
articulation 32/4, fruits 32/1, 6 N from startup, corrected coherent sphere sizes,
inertias, support masses/drives/limits/colliders and native joint coefficient 10
are unchanged. Rendering remains GPU. Static full-stage comparisons confirm
only that scene attribute differs; loaded body checks pass.

| Trial | Outcome |
|---|---|
| Independent rest | 60 s, no breaks/errors; tail pose excursion 0.0498 mm |
| Recorded native joint gesture, after 30 s settling | 60 s, no gross instability, but target does not detach; failed interaction requirement |
| COM force ramp 0–12 N over 5 s, after 30 s settling | Target breaks at 32.55 s / 6.1200 N commanded; continuity passes, no other breaks through 60 s |

The force command is not a joint reaction-force sensor. The native replay is a
translated recorded input clip with independent raycast verification, not custom
mouse dynamics. Its failure means mouse usability remains unproven. The native
GUI is opened to obtain direct manual evidence; headless COM success is not
presented as native interaction acceptance. Solver-velocity advisories remain
separate from pose-derived motion.

GUI evidence: `artifacts/detachable_fruit_v2/native-cpu-rank10/gui-i9xbvvc_`.
No automatic replay or timeout in GUI; user supplies Shift+click. No custom force
controller or detachment recreation. Request 60 simulated seconds with at least
10 seconds of final recovery, and measure FPS separately from simulation speed.
Manual approval and FPS gate are pending.

```bash
UV_CACHE_DIR=/tmp/autotom-uv-cache uv run --no-sync python \
  src/experiments/detachable_fruit_v2/run_prepared_gui.py \
  artifacts/detachable_fruit_v2/native-cpu-rank10/rest
```

Preparation: `prepare_native_cpu_rank10.py --output NEW_ROOT` for rest/native;
`--case com --output ANOTHER_NEW_ROOT` for the controlled-force check. Execute
with `run_batch.py`. [Compact results](native_cpu_rank10_results.json) preserve
report hashes, outcomes and interaction metadata. Heavy evidence is local.
No default exporter/preset behavior has been changed.

## Completed manual joint-mode review

User: "Si muovono ma non si staccano". Window closed explicitly at 79.2333
simulated seconds, no breaks or monitor errors. Native observer records fruit
picks. Mean 134.30 FPS, after startup 134.01 FPS, real-time factor 2.2383.
The candidate fails detachment acceptance despite good cadence. No threshold
compensation is applied.

## Built-in native force mode comparison

Same CPU/TGS scene and recorded gesture, native force mode coefficient 50
(explicit experimental coefficient flag, not newtons). Target breaks at
30.2833 simulated seconds, continuity passes, no other breaks/errors through
60 s. This is PhysX native picking; no custom controller is used. Manual gesture
and recovery review remain required. Evidence: `native-cpu-rank10-force50/native`.

Native force GUI (manual review pending): `native-cpu-rank10-force50/gui-rnlo0ea2`.
Reopen with `run_prepared_gui.py artifacts/detachable_fruit_v2/native-cpu-rank10-force50/native`; the GUI launcher removes automatic replay.
