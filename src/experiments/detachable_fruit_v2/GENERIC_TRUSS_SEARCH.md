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
