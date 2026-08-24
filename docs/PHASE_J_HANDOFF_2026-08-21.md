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
The remaining immediate gate is a GUI review of the same USD. The stem is
intentionally fixed and is not an interaction test; Shift+drag begins with the
later `leaf-supports` checkpoint.

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
