# Generic native truss search — 2026-09-15

User objective: a stable generic truss at least on the main stem, using original
fruit data at different ranks. Native mouse and physical detachment; no repeated
rank-6 mass substitution. Stop and ask the user when a very promising candidate
appears. Maximum two hours: goal created 09:52:47 UTC, deadline 11:52:47 UTC
(12:52:47 Europe/London). No automatic extension beyond that deadline.

Starting checkpoint: `3d525cb`. The original rank-10 TGS/CPU native force50 GUI
detaches, but user rejects the residual bend. A same-remaining-load fresh control
has a different final pose. The previous direct rank6-standard configuration is
preserved and is not treated as a generic solution.

First diagnostic: rank10, unchanged complete stem/truss/masses, 60 Hz, 6 N,
CPU, 32/4 and32/1. Rest30 s, ramp to2 N upward at the COM in1.25 s, hold until40 s,
release immediately, observe to60 s. No fruit should detach. Compare TGS,
TGS with force scheduling enabled, and PGS. Same load before and after allows
comparison of joint/pose recovery independent of fruit removal. These direct-force
diagnostics do not replace the required native manual review.

Evidence root: `artifacts/detachable_fruit_v2/generic-truss-search/`.
Preparer: `prepare_generic_truss_candidate.py`; each case records source hashes,
exact scene changes and expected runtime body properties. No masses, geometry,
drive gains or mouse code are changed by this preparer.

## Constant-load recovery result

All three independent 60 s cases pass without detachment or monitor errors.
Compare means at20–30 s (settled before pull) and50–60 s (after release):

| CPU solver | Maximum residual position | Maximum residual DOF angle |
|---|---:|---:|
| TGS default force scheduling | 29.18 mm | 2.407 degrees |
| TGS external forces every iteration | 0.985 mm | 0.157 degrees |
| PGS | 0.711 mm | 0.065 degrees |

No geometry, mass, drive, limit or threshold edits. Single runs on original rank10,
not proof of generic stability. The scheduled TGS candidate now needs native
detachment and manual recovery review. User asked to pause at promising results;
a review request has been issued before expanding to other ranks.

Compact evidence: `generic_hold_results.json`. The scheduling attribute exists
in the installed Isaac4.5 schema and is authored true in the candidate.

Scheduled TGS/CPU native force50 replay: 60 s passed; one break on the selected
fruit at30.3000 s, continuity passes, no other breaks or monitor errors.
Observer labels replay attribution unclassified; target identity and clip timing
are checked separately. This rapid replay does not validate the user's slow pull.
See `generic_native_results.json`. Await manual review before campaign expansion.

Reopen (manual input, automatic replay removed by launcher):
```bash
UV_CACHE_DIR=/tmp/autotom-uv-cache uv run --no-sync python \
  src/experiments/detachable_fruit_v2/run_prepared_gui.py \
  artifacts/detachable_fruit_v2/generic-truss-search/tgs-scheduled-native
```

## First scheduled-TGS manual session: additional failure

GUI `gui-wu5dikte` closes normally at67.2833 simulated seconds, mean141.22 FPS
after startup, real-time factor2.3566. Four selected breaks. However all four
detached fruits exceed100 m/s within0.10–0.133 s of break while mouse remains
held, reaching extreme finite velocities. Attached supports settle. Candidate
is not accepted pending diagnosis; no all-rank expansion. Human feedback pending.
The rapid headless replay releases automatically at break and therefore did not
cover continued native dragging of the free fruit. Existing gross-error gates
exclude broken bodies and missed this failure; passing status is insufficient.
Evidence: `generic_gui_results.json`, full native event and per-step trace locally.

User confirms: detachment works but tomato flies away. PhysX source modification
is an explicitly deferred last resort: stop and discuss before attempting it.

Replay instrumentation now has opt-in `native_replay_hold_after_break`: keep
sending original PhysX input events until recorded release, without custom forces.
A separate opt-in all-body1000 m/s diagnostic bound catches detached runaways
while allowing normal free fall during this60 s test. Regression tests33 passed.

First slow GUI gesture replayed from30 s, release at36.2833 s:
- Scheduled TGS/CPU force50 reproduces runaway:1786 m/s at33.5667 s.
- Default TGS/CPU force50 detaches at33.3167 s and completes60 s without runaway.
- Scheduled TGS/CPU joint10 completes60 s but does not detach.

Next bounded comparison: scheduled force10 andforce25, same full gesture.
No PhysX binary edits. No changes to body masses, drive stiffness or6 N threshold.

## Retained-native comparison completed

Scheduled TGS force10/25/35 and joint10/50 do not detach with this clip.
PGS/CPU force50 does detach at33.3667 s and completes60 s, with native hold
continued to36.2833 s. No divergent free-fruit acceleration; however target peak
speed during gesture is12.41 m/s, so gentle interaction is NOT established.
Tail support pose excursion0.0193 mm, but correct post-removal equilibrium
still needs manual review. This is a candidate, not a generic solution.
User review requested before further expansion.

Summary: `retained_native_results.json`; `prepare_retained_native.py` prepares
the eight comparisons from local source evidence without modifying scenes.
Native PhysX binary unchanged. Default replay still releases at break; retaining
is opt-in for diagnostic coverage. GUI native behavior unchanged.
