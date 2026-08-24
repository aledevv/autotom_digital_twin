# Phase J handoff — checks still required

Date: 2026-08-21

Phase J is implemented far enough to generate and audit canonical V2 stages,
but it is **not completed**. The mature flexible articulation fails the
mandatory Isaac Sim stability test, so the migration plan must not mark Phase
J as `COMPLETED` yet.

## 2026-08-24 conservative rebuild checkpoint

The current debugging path no longer treats the direct PlantState USD author
as the reference physics implementation. The migration is being repeated one
organ group at a time through the established V2 `BRANCHES -> build_stage()`
backend. The first implemented gate is `--debug-profile stem`.

The stem adapter emits one historical `trunk` branch with optional per-link
`link_specs`. Canonical mode preserves each GroIMP internode frame, length and
radius; legacy mode omits the frames. Both modes retain the segmented organic
mesh technique—there is no cylinder fallback. Day 10 currently audits as:

```text
5 main-stem internodes
5 rigid bodies
5 FixedJoints (root included)
10 invisible capsule colliders
5 OrganicVisual meshes
0 D6 joints
0 visual cylinders
```

The one-second headless Isaac run at 480 Hz passed with five finite bodies,
zero displacement, zero root/endpoint drift, no NaN/Inf and no PhysX error.
The stem is intentionally fixed and is not an interaction test.

The second implemented gate is `--debug-profile laterals`, tested on day 50.
It reconstructs four native order-one chains with four Internodes each and
attaches them to trunk links `2, 2, 3, 7`. The fixed stem remains the physical
support while all 16 lateral links use the established V2 D6 drives. No leaf
or truss geometry is included yet.

```text
26 rigid bodies
10 FixedJoints
16 D6 joints
52 invisible capsule colliders
26 OrganicVisual meshes / 5 visual axes
0 visual cylinders
0 active unfiltered initial overlaps
```

The day-50 one- and five-second flexible tests at 480 Hz pass. Because
`World.reset()` performs one internal physics step, gravity is suspended only
during that step and immediately restored. This separates joint-frame snapping
from real gravitational deformation: reset projection is below `8.8e-8 m`,
the laterals settle to `5.56%` maximum sag, stem drift is zero, and the final
tail speed is about `1.54e-4 m/s`. GUI telemetry confirms 16 interactive D6
bodies and mouse picking of invisible colliders. The user approved the visual
shape and Shift+drag interaction on 2026-08-24. This checkpoint is therefore
closed; leaf supports are the next incremental group.

## 2026-08-24 interactive-rate and performance checkpoint

The apparent day-50 regression to about 11 FPS was caused by a runtime
difference, not by PlantState topology. The old loader silently used Isaac's
60 Hz `World` default even though its USD declared 480 Hz. The diagnostic
loader applied 480 Hz and paid for eight PhysX substeps per 60 Hz render.

The runtime now uses 60 Hz for GUI inspection and retains 480 Hz for mandatory
headless validation. `--interactive-physics-hz 60|120|240|480` allows explicit
GUI experiments without changing the authored stage. Telemetry now separates
authored/runtime physics Hz, render Hz, render updates, physics steps and real
simulated time. A permanent legacy/candidate benchmark emits
`exporter_v2_performance_comparison/1.0` JSON.

Same-machine day-50 baseline:

```text
PlantState: 60 Hz 51.0 FPS, 120 Hz 34.7, 240 Hz 20.9, 480 Hz 12.0
480 Hz equal comparison: legacy 1.74 FPS, PlantState 11.89 FPS
60 Hz historical loop: legacy 11.5 FPS, PlantState 50.5 FPS
```

The earlier segmented-visual optimization is retained (81,036 -> 39,304 mesh
triangles) but is not presented as the root FPS fix. Phase J is still
`IN PROGRESS`; subsequent leaf/truss checkpoints must pass both interactive
60 Hz inspection and 480 Hz headless validation.

The permanent isolated benchmark subsequently confirmed 49.69, 34.30, 21.07
and 11.15 candidate FPS at 60, 120, 240 and 480 Hz. The matching legacy values
were 11.24, 6.34, 3.34 and 1.70 FPS, so the candidate remained 4.42-6.54 times
faster at equal cadence. Isaac Sim 4.5 requires its runtime `PhysicsScene`
registry to be synchronized after opening an existing USD and the selected
cadence to be reapplied after `World.reset()`; otherwise its getter silently
falls back to 60 Hz. A five-second day-50 `leaves` validation then completed at
an effective 480 Hz (2,400 steps) without non-finite bodies or articulation
failure.

The corresponding GUI run used an effective 60 Hz for 15.83 simulated seconds
and closed normally. It measured 39.39 rendered frames/s in the full
interactive application, with 43 interactive bodies, mouse grab and invisible
collider picking enabled, and no non-finite body. Shift+drag quality still
requires the user's visual confirmation; telemetry only proves availability.

## Implemented in this checkpoint

- Canonical `PlantState -> complete V2 visual view -> physical plan ->
  collision adaptation -> USD + manifest` path.
- No CSV fallback in the day-based V2 workflow.
- Serverless CLI: `uv run python -m exporterV2 --day N`.
- `run_mainV2.sh --day N` generation and Isaac loader; the no-day `BRANCHES`
  demo remains available.
- PlantBase rebasing to `(0,0,0)`, original GroIMP origin in metadata, and
  declared `GLOBAL_SCALE=2.0` adaptation.
- Canonical path/organ IDs on visuals and physical links.
- Exact duplicate suppression only when geometry, owner attributes and
  biological parent are identical.
- Joint thresholds 220/230 and physics-only aggregation entry point.
- Finite capsule-capsule, sphere-capsule and sphere-sphere checks using the
  collider dimensions actually authored to USD.
- Deterministic azimuth/tilt/shift attempts and explicit filtering/reporting
  for unresolved pairs.
- Deterministic `exporter_v2_manifest/1.0` with canonical/visual/physics
  counts, source/authored poses, collider settings, corrections, filters and
  canonical provenance.
- OpenUSD-only PhysX attribute authoring: serverless generation does not
  require Isaac Sim's `PhysxSchema` plugin.
- Isolated GroIMP duration override and a validated day-160 PlantState at
  `data/plant_states/plant_state_day_160.json`; the source RGG/GSZ is unchanged.
- Headless Isaac stability report `exporter_v2_stability/1.0`, with full-body
  finite-pose, explosion, root drift and long-run tail-speed checks.

Canonical counts observed:

```text
day 1:   20 visual axes,   0 fruits,   9 physical axes,   8 D6
day 25: 130 visual axes,   1 fruit,   51 physical axes,  50 D6
day 80: 347 visual axes,  72 fruits, 216 physical axes, 215 D6
day 160:347 visual axes,  72 fruits, 216 physical axes, 215 D6
```

Days 80 and 160 have the same structural counts, but they are not copies: at
day 160 the fruits have continued to mature and the minimum canonical fruit
radius increased from about 5.88 mm to 11.49 mm.

## Isaac results obtained

```text
day 25 locked, 5 s @ 480 Hz:     PASS
day 25 flexible, 5 s @ 480 Hz:   PASS
day 80 locked, 5 s @ 480 Hz:     PASS
```

The scalable locked baseline uses kinematic bodies. Fixed joints are retained
as disabled topology joints because a dynamic island of 215 FixedJoints took
several minutes merely to initialize in PhysX. This matches the intended
"static/rigid baseline" and is recorded in stage and manifest metadata. Until
flexible validation succeeds, both the module CLI and `run_mainV2.sh` default
to `locked` for safety.

The current blocker is:

```text
day 80 flexible, 5 s @ 480 Hz, stiffness 1x: FAIL before 1 s (NaN/Inf)
day 80 flexible, 5 s @ 960 Hz, stiffness 1x: FAIL before 1 s (NaN/Inf)
day 80 flexible, 5 s @ 960 Hz, stiffness 2x: FAIL before 1 s (NaN/Inf)
day 80 flexible, 5 s @ 960 Hz, stiffness 4x: FAIL before 1 s (NaN/Inf)
```

The 960 Hz fallbacks scale damping by `sqrt(stiffness_scale)`. All three fail
at essentially the same instant and with the same affected articulation, so
the failure is not explained by insufficient stiffness. The offline joint
frame reconstruction is consistent to approximately `1e-16`; computed masses
range from `6.66e-4` to `1.91e-2` kg and drive stiffness from `1.92e-3` to
`5.55e-1`, so no obvious non-finite or extreme source parameter was found.

## First diagnostic to run

Determine whether the terminal fruit bodies trigger the day-80 divergence.
Generate the flexible stage, make a temporary copy, then disable only terminal
rigid bodies, terminal joints and terminal collisions in that copy. Run one
simulated second. A temporary version was prepared under `/tmp` during this
session but is intentionally not committed.

If it still fails, repeat with all collision shapes disabled. This separates:

1. terminal-body/joint instability;
2. collision/contact instability;
3. the D6 articulation topology or drive formulation itself.

If the D6 articulation still fails without collisions, bisect by canonical
subtree (main stem, leaves, trusses/pedicels) while preserving the exact
authored joint frames. Record the first body that becomes non-finite and its
ancestor chain. Do not solve the issue by deleting canonical visuals.

Useful commands:

```bash
UV_CACHE_DIR=/tmp/uv-cache uv run python -m exporterV2 \
  --day 80 --physics-preset flexible --physics-hz 960 \
  --stiffness-scale 1 --output /tmp/tree_v2_day80.usda

~/isaacsim/python.sh src/exporterV2/isaac_app.py \
  --usd /tmp/tree_v2_day80.usda --headless --duration 1 \
  --physics-preset flexible --physics-hz 960 \
  --report /tmp/tree_v2_day80.stability.json
```

## Mandatory checks before completing Phase J

- Resolve the flexible day-80 NaN/Inf failure without hiding canonical visual
  organs.
- Repeat 5-second locked and flexible tests for days 1, 25, 80 and 160.
- Run the 30-second flexible stress test for day 80; because validation was
  explicitly extended, also run a 30-second day-160 stress test.
- Confirm no fruit detachment, articulation invalidation, persistent
  oscillation, PhysX crash, NaN/Inf or body explosion.
- Run a GUI smoke with `./run_mainV2.sh --day N` and verify the window remains
  interactive until closed.
- Re-run the complete offline Phase A-J suite with `uv run`.
- Re-run opt-in live GroIMP validation, including day 160 with
  `--model-duration-days 160`.
- Compare SHA-256 manifests for `model/project_bridge.gsz`, `model/input/`,
  `model/output/` and committed PlantStates before/after the final tests.
- Confirm no temporary GroIMP workbench remains open.
- Only then mark Phase J `COMPLETED` and select `flexible` as the default.

## Current source integrity

The isolated day-160 extraction did not edit the source model. Hashes observed
after extraction:

```text
d646a340eb3fd57f885d4dcea8f7f207b76a35596b0a87c90e63968d30acf4d9  model/project_bridge.gsz
e35f44c6edd68ad35370b80704a28db2a544b4b04e696f9d6f46a0f88f8cb1a9  model/param/parameters.rgg
```

No generated USD or stability log under `/tmp` is part of this checkpoint.
