# Checkpoint — 2026-09-16

Work paused at the user's request. Continue on
`experiment/v2-detachable-fruit-stability`; main and groPy are unchanged.

## Default local launch

After the user's subsequent positive feedback, `./run_mainV2.sh` (no arguments)
opens the last shown organic leaf collision scene. `--latest-simulation` is an
explicit alias. Scene and source-config hashes are pinned; each launch creates
fresh GUI output and runs until explicit closure. The fixture stays local and
must be restored separately on a fresh clone. `ISAACSIM_DIR` is respected.
Use `./run_mainV2.sh --exporter --day 160` for the existing exporter workflow,
or `./run_mainV2.sh --exporter` for the historical static demo. Existing explicit
`--day` invocations retain their behavior. This shortcut does not generalize
the experiment into the canonical exporter or enable all petiolule collisions.

## Saved state

- Reference: `realism-leaf-droop`, original checkpoint `3c2e6cf`.
- Collision campaign scripts and earlier evidence: checkpoint `8d12d86`.
- Current local candidate: `artifacts/branch_collisions/C-organic-leaf-pair-settle60`.
  It adds 17 convex organic-surface colliders across the selected main-stem leaf
  and lateral branch/leaf assembly, including rigid petiolules. It is not global.
- `prepare_organic_pair.py --include-lateral-leaf` builds the candidate from
  `C-screen`. Copy its config to a fresh case and set
  `diagnostic_reset_projection_limit_m=0.01` and `duration=60` for this trial.
  This tolerates measured initial separation (6.2794 mm), not divergence.
  The default diagnostic threshold remains 1 um, as does the fixed-root check.
- Loaded masses/inertias are preserved; no solver, stiffness, mouse controller,
  fruit threshold or authored-pose changes were made for this trial.
- Screens passed 20 and 60 seconds without reported errors or spontaneous breaks.
  Monitor regression suite: 18 passed. See `organic_settling_results.json`.

## Latest GUI and user feedback

Run: `artifacts/branch_collisions/gui-C-organic-leaf-pair-9d4zhb1d`.
Stopped with SIGINT for report persistence at the checkpoint. Recorded
293.800 simulated seconds, no reported validation errors. Reported overall GUI
FPS 38.80 and steady FPS 38.58; user observed approximately 33 FPS late in the
session. This is not a matched A/B performance benchmark. The report's raw
sim/wall ratio is not a validated pause-excluded active-time measurement.
See `organic_gui_results.json`.

The first organic pilot did not improve the user's crossing gesture: it excluded
the lateral leaf actually encountered. The expanded candidate includes
`LatLeaf_r3_o0_g421593` and both `Leaf_r5_o0_g421371_rachis` links, plus
`Branch_s2_o1_g421414_Link_04_Internode_g421675`.
The user has not explicitly confirmed final effectiveness of these new contacts.
Do not label this a fully accepted solution or a completed collision campaign.

Optional extension requested only as a code comment:
`LeafVisual_g421563_petiolule_left_02` and similar petiolules; exclude their own
support assembly but allow eligible third branches. Benchmark and verify initial
overlap before any rollout. This extension remains disabled.

## Remaining limits

D (truss contacts), matched paced/unpaced GUI benchmarks, observer overhead,
three independent candidate confirmations and final contact acceptance remain
unfinished. No runtime filter activation after settling was implemented.
Heavy USD, traces and logs remain local under ignored `artifacts/` subdirectories.

`docs/plan.md` is a separate leaf-physics experiment plan present in the workspace
and included in the requested save-all checkpoint; this session did not execute it.
