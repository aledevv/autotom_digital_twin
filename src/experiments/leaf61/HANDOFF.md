# Current status — 2026-09-18

User resumed. Shared-stem contact logging and moving-frame tests completed; dynamic and fixed-stem runs pass. Passive branches move 2.289/3.308 mm with the mobile stem and 0/0 mm with fixed stem. Full contact evidence confirms no direct hits on passive branches or stem. Details and commands are in BRANCH.md; comparison artifact is artifacts/leaf61/shared-stem-comparison-0918.json. Shared-stem GUI launched in artifacts/leaf61/shared-stem-gui-0918, visual review pending. No further campaign is running automatically. Below is the preserved previous handoff for history.

# Handoff — 2026-09-17

User paused work until tomorrow. Do not start additional simulations or GUI sessions until resumed.

Accepted visually: optimized single hinged branch with three skinned leaves and tomato; five independent branches with fifteen leaves also looked acceptable to the user. Previous results and limitations are recorded in BRANCH.md and STORM.md. Final independent-branch offscreen means: 2 branches 36.62 FPS, 5 branches 31.49 FPS, 10 branches 25.13 FPS. p95 work: 37.57 / 43.73 / 61.23 ms. Full-rate vectorized attachment checks preserve sampled trajectories exactly. Thirty FPS constant is not certified. Scaling artifacts are under artifacts/leaf61/branch-scale-*-batched-checks.

Current task: three branches connected to a common dynamic stem, one tomato on the upper branch, to test motion transfer to unstruck branches. Implemented a separate --shared-stem mode and run_leaf61_shared_stem.sh. --fixed-stem supplies a control. Shared stem is a synthetic 0.5 m rigid stem on an elastic X hinge (60 g, 3 Nm/rad, 0.25 Nms/rad), not distributed bending or the main plant. Branch attachment heights 0.20/0.32/0.44 m. Original branch and leaf parameters preserved. One articulation rooted at a fixed stem anchor. Cross-branch collision groups isolate the tomato from lower leaves; attachment checks now accept moving stem frame targets. Camera adjusted for the taller fixture.

First headless run started: artifacts/leaf61/shared-stem-smoke, process session 76572. No shared-stem GUI has been launched. Source snapshots and logs are in the run directory. Do not claim shared-stem acceptance until inspecting the report and validating the control.

Next steps on explicit resume:
1. Inspect the smoke result and logs; fix any failures without relaxing thresholds.
2. Strengthen contact evidence: branch_tomato currently logs only leaves under its own prefix. Log other leaf/actor contacts too, so the shared-stem test can explicitly prove no direct contact on passive branches or stem. Check target contact against the full intended prefix.
3. Add a pure test for moving stem attachment frames. Existing attachment_error test passes only with full suite module-import context: move model.rotation import to module scope inside branch_layout (currently function-local), or keep the test import path active. shared_stem has the same function-local pattern; test it independently.
4. Run dynamic-stem and fixed-stem control sequentially. Compare passive branch response and stem displacement, attachment error, deformation, recovery and performance; then launch GUI after checks.
5. Document all results in BRANCH.md and preserve failures. Existing tests: 29 passed before latest shared-stem changes; latest edits only formatted/linted, no full test run yet.

All work remains local; no commit or deployment. The user explicitly asked to retain numerical history for writing later.

## First shared-stem report at pause

```json
{
  "checks": {
    "completed": true,
    "finite": true,
    "attachment": true,
    "sampled_stretch": true,
    "sampled_area": true,
    "recovery": true,
    "residual_oscillation": true,
    "lamina_response": true,
    "branch_response": true,
    "shared_stem_response": true,
    "passive_branches_respond": true,
    "tomato_fell": true,
    "eight_seconds_after_contact": true,
    "target_contact": true,
    "contact_penetration": true
  },
  "runtime": "optimized",
  "gui_paced": false,
  "performance_window": "load start to trial end, excluding initialization and settling",
  "performance": {
    "frames": 390,
    "rendered_fps": 45.58209617151611,
    "work_p95_ms": 27.319731461466276,
    "components_ms_per_frame": {
      "physics": {
        "median": 8.447467494988814,
        "p95": 9.971924053388648
      },
      "read": {
        "median": 0.24056798429228365,
        "p95": 0.28704869910143316
      },
      "diagnostics": {
        "median": 4.201815987471491,
        "p95": 4.876977080130018
      },
      "skinning": {
        "median": 1.7830219876486808,
        "p95": 5.17271103162784
      },
      "render": {
        "median": 6.0256890137679875,
        "p95": 8.057230908889323
      }
    }
  },
  "branch_max_motion_from_equilibrium_m": 0.012256580404937267,
  "branch_motions_m": [
    0.002289394149556756,
    0.003307565813884139,
    0.012256580404937267
  ],
  "stem_motion_m": 0.0025437462609261274,
  "shared_stem": true,
  "fixed_stem": false,
  "branches": 3,
  "leaf_recovery_error_m": 0.00037996895844116807,
  "lamina_tip_motion_in_petiole_frame_m": [
    1.1311447451589629e-05,
    2.4594468413852155e-05,
    4.21439362980891e-05,
    9.760671673575416e-06,
    2.0937563022016548e-05,
    3.536694930517115e-05,
    0.00013112701708450913,
    0.00038093177136033773,
    0.03633938357234001
  ],
  "residual_oscillation_m": 0.0,
  "attachment_max_error_m": 1.8172777101048467e-07,
  "max_edge_extension": 0.04027596471083572,
  "min_area_ratio": 0.9690265621236531,
  "frame_work_p95_ms": 27.28145773871801,
  "geometry_sample_hz": 10,
  "fixed_branch": false,
  "visual_acceptance": "pending",
  "completed": true
}
```


## Full native plant integration — 2026-09-18

New separate launcher: `./run_leaf61_full_plant.sh` (131 flexible native blades, original dynamic branches, native Shift + left-drag). `--leaves 20` converts only the first 20 source blades while retaining the complete native plant. Production main/exporter unchanged.

See [FULL_PLANT.md](FULL_PLANT.md) for topology, mass redistribution, retained failed attempts, measured costs and limitations. Automatic load/recovery checks pass for 20 and 131 blades, but offscreen work rates are only 15.80 and 6.86 FPS respectively at unchanged 480 Hz physics. No 20 FPS performance acceptance; GUI acceptance pending. Final GUI artifact: `artifacts/leaf61/full-plant-131-gui-v2`. The previous GUI completed at 5.86 work FPS; PhysX support UI startup was subsequently corrected to run inside the Kit asyncio loop.

### Contact maximum, explicit restricted demo

`./run_leaf61_full_plant_contacts.sh` enables leaf/native and leaf/leaf contacts on all 131 flexible blades, excluding the 49 pairs measured deeply overlapping at startup. Raw `--leaf-contacts all` was rejected (11.46 mm initial overlap, 45.69 m native-body runaway). The restricted 20 s run completes without runaway and records actual branch/leaf contacts, but remains numerically FAILED: 10.01% mesh stretch and 1.439 mm settled penetration. Throughput 5.10 FPS offscreen; GUI `full-plant-contact-filtered-gui` is a stress demo, not acceptance. See FULL_PLANT.md and the per-run contacts/policy/comparison JSON files. Default full-plant launcher still uses contacts off.

### Softer leaf behavior, illustrative priority over FPS

The user found the old three-link leaves acted like rigid levers against branches. Added a separate `--leaf-profile soft` with seven segments, tapered bend/torsion drives, curved 3-D strip colliders, actual-sheet bend axes and a pivot at the real petiole endpoint (no broad rigid basal band). Native mesh and total biomass are preserved. Original profile/defaults remain unchanged.

Final controlled contact (`soft-leaf-base-hinge-contact`, 20 s): lamina bends 26.629 mm relative to petiole vs 16.331 mm originally; actual base moves 0.0155 mm vs 0.533 mm originally; recovery 0.0105 mm, contact penetration 0.00146 mm. All numeric checks pass in this controlled case; 35 tests pass.

Launch `./run_leaf61_soft_contact.sh` for a close-up repeatable capsule/leaf demonstration, or `./run_leaf61_soft_plant.sh` for all 131 soft blades with leaf/native contacts and 82 initial-overlap exclusions. GUI launched as `full-plant-soft-gui`. Whole-plant 3 s preflight remains numerically FAILED (78.56% maximum edge extension) despite finite attached bodies: illustration only, not a continuous sheet or accepted physical plant. See SOFT_LEAF.md for intermediate failures and final revision-3 evidence. FPS was explicitly deprioritized; no performance acceptance claimed.
