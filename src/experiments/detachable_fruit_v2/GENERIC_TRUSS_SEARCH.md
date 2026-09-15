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

## PGS manual feedback: accepted rank10 behavior, fast-time caveat

`gui-or0qhc9i`: user reports stable, fruit follows mouse after detachment;
fast-looking drop is acceptable for now but must be documented. Native force50,
PGS/CPU60 Hz; no custom grip/controller and no fruit mass changes. Six selected
breaks between338.8667 and419.8834 simulated seconds, followed by recovery.
Mean steady153.92 FPS, simulation/wall ratio2.56379. All six post-release
one-second velocity increments have vertical acceleration -9.809996 m/s2.
Thus fast displayed free fall is consistent with accelerated simulation time,
not increased fruit density. Attached-stage geometry is still main-derived;
this does not establish generic v2.3 integration or other-rank acceptance.

Run ended at447.8834 s because the fixed1000 m/s diagnostic gate falsely
flagged normal unbounded gravity fall (first fruit released345.9500 s; no
ground). This gate was adequate only for the60 s diagnostic horizon; it needs
a gravity/time allowance for unlimited GUI sessions. Do not call this a physics
crash or hide the raw failed report. Detailed analysis in
`pgs_manual_release_analysis.json`. Manual acceptance of this rank10 case does
not waive other-rank or full-scene checks.

## Optional real-time GUI pacing

User requests wall-clock1x review. `run_prepared_gui.py --real-time` enables
optional pacing before physics; unchanged dt/solver/native input/masses. Headless
is unpaced. Slow frames reset the wall anchor to avoid catch-up bursts; timeline
pauses reset it. Wait cost is reported separately from physics/rendering.
Regression checks:34 passed. Initial live GUI `gui-4u3l6ur_`:13.2000 simulated
seconds /13.1913 wall seconds, ratio1.00066, approximately60 FPS, no errors.
Manual review remains pending. This initial measure is not a full-run claim.

The optional all-body speed guard now allows |gravity| times elapsed simulation
time beyond its configured base bound, avoiding false positives from long
unbounded falls. It still catches the prior enormous divergent speeds; it is
a conservative gross-error diagnostic, not a realism criterion.

```bash
UV_CACHE_DIR=/tmp/autotom-uv-cache uv run --no-sync python \
  src/experiments/detachable_fruit_v2/run_prepared_gui.py \
  artifacts/detachable_fruit_v2/generic-truss-search/retained-pgs-force50 --real-time
```

Real-time GUI review completed: `gui-4u3l6ur_`,101.2500 simulated seconds,
101.2449 wall seconds, steady60.0002 FPS, real-time ratio1.00005, no monitor
errors. User: stable, but detachment gesture remains abrupt. This validates
pacing on this rank10 scene, not other ranks.

User asks whether lowering break threshold avoids spontaneous detachment.
Prepare4 N and3 N from the same PGS/CPU force50 original-rank10 scene. Each
threshold is active from startup, all eight fruit joints; no arming delay.
One60 s rest run and one60 s retained-native slow-gesture run per threshold.
Only authored breakForce changes; all body and drive properties retained.
`prepare_break_threshold.py` records/verifies the complete attribute difference.

Threshold comparison completed:4 N and3 N each pass60 s rest with zero breaks
and60 s native held-after-break gesture with exactly one intended break and
continuity passing. Runtime final joint thresholds verified on all eight fruits.
Peak selected-fruit speed during the identical gesture:6 N12.410 m/s,
4 N8.655 m/s,3 N6.956 m/s. Lower speed is not proof of a gentle gesture.
No mass, geometry, drive, solver or mouse-coefficient changes. Rank10 only;
no generalization to other ranks. Request manual real-time3 N GUI review.
Evidence: `threshold_results.json`.

## Real-time3 N feedback and native mouse coefficient

`gui-yyurc8k5`: user accepts basic detachment but objects to fruit rapidly
reaching pointer; wants a visible elastic pull with gravity after detachment.
Confirmed config: native force50, force_target=None, no retain_fruit_grip custom
controller. Recorded four breaks with continuity passing; no explicit reposition.
Do not label the gesture fully accepted.

Compare3 N force10/25 against existing force50, same recorded retained-native
clip. Threshold, PGS/CPU60 Hz and all scene properties unchanged.
`prepare_break_threshold.py --mouse-coefficient` can reproduce these variants
from the original6 N source. Force10 completes60 s but does not detach.

Force25 completes60 s with one intended break at33.2167 s, but peak target
speed8.473 m/s exceeds force50's6.956 m/s (break31.3667 s). Same input clip,
different detachment time/target displacement: lowering gain is not established
as a solution to perceived snapping. No new GUI candidate claimed.
See `three_newton_mouse_results.json`.
