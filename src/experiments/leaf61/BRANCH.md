# Coupled branch and skinned leaves — 2026-09-17

## Fixture and scope

Separate synthetic fixture: one 280 mm rigid branch with an elastic revolute hinge about X, and three existing seed-42 skinned leaves at 60/140/220 mm along the branch. The branch rotates at its base; it does not have distributed curvature or the main plant's geometry. Leaf geometry, mass and elastic joint parameters are reused unchanged. This is not Surface Deformable.

An anchor body is fixed to the world. An internal revolute joint connects it to the branch. Fixed joints connect each petiole to the branch, and the existing three elastic lamina joints follow. There is one articulation rooted at the anchor's fixed joint. Internal self-collisions are disabled; the branch has a box collider. No tomato or water is present in this first coupling check.

Branch mass: 25 g; stiffness: 0.4 Nm/rad; damping: 0.04 Nms/rad; angular limits: ±40 degrees. These are engineering test parameters, not calibrated biology. Each petiole retains the previous 10 g fixture mass. Leaf stiffness remains 0.024 Nm/rad. CPU/PGS, 480 Hz physics, 32 position iterations, 30 Hz rendering, sleeping disabled.

Protocol: gravity enabled after 1 s, equilibrium captured at 9 s, downward load ramped to 0.03 N on the final rigid segment of each lamina over 1 s, held for 3 s and removed over 1 s. Observe recovery for 8 s. Forces act through PhysX; leaf and branch poses are not commanded. The load is applied at the final segment's origin, near the tip, not at a mesh vertex.

## Verified runs

| Quantity | Moving branch | Fixed branch control |
|---|---:|---:|
| Additional branch centre motion from gravity equilibrium | 3.949 mm | 0 mm |
| Lamina final-segment motion in petiole frame | 8.111–8.119 mm | 8.326 mm |
| Maximum attachment-frame error (including branch anchor) | 0.000058 mm | 0.000003 mm |
| Maximum edge extension | 0.1382% | 0.1427% |
| Recovery error, maximum segment position | 0.000481 mm | 0.000194 mm |
| Residual motion, last second | 0.0000075 mm | 0 mm |
| Frame work p95 | 37.01 ms | 32.23 ms |

Evidence: `artifacts/leaf61/branch-moving-smoke-v3` and `artifacts/leaf61/branch-fixed-control`. Each contains input mesh, source snapshot, launch command/hashes, scene, config, log, pose trace and report. New launches also record Isaac version, available PhysX extension versions and GPU/driver.

Both final runs pass finite data, attachment <0.2 mm, sampled stretch <5%, sampled area ratio >0.05, recovery <2 mm and residual motion <0.5 mm. A nonzero lamina response in the petiole frame is required; the moving case additionally requires nonzero branch response. Geometry is sampled at 10 Hz; poses and attachment errors are checked at physics frequency. Recovery is segment-position recovery, not a complete vertex-level biological validation. GUI appearance awaits user review.

Frame work includes physics, reads, skinning, rendering and online measurements over the whole trial, without process initialization or offline reporting. It is a single run per mode and is not a separated physics benchmark or a plant capacity claim. GUI timing includes pacing and must not be compared directly to these offscreen numbers.

## Failed intermediate attempts

`branch-moving-smoke`: measurement code passed a single quaternion to a batch rotation helper and stopped; corrected to a one-row batch.

`branch-moving-smoke-v2`: using the world-connected revolute joint as articulation root left the branch fixed. The initial checks passed lamina response but did not establish branch motion. This run is invalid as evidence for coupled motion. The final topology introduces the separate fixed anchor and internal hinge; the final report explicitly checks branch and relative lamina response.

## Commands

GUI:

```bash
./run_leaf61_branch.sh
```

Controls: **Load leaves / Repeat test**, **Release load**, **Finish and save**. The first test starts automatically after settling. Repeating allows eight seconds to settle before applying load. The GUI stays open after a completed trial; recording is bounded per trial.

Headless comparison:

```bash
./run_leaf61_branch.sh --headless
./run_leaf61_branch.sh --headless --fixed
```

Next milestones: GUI review, then a physical tomato/contact case on this fixture, followed by a branch with distributed bending and integration with the main plant. Rain and full plant coupling are not implemented by this fixture.

## User review

The user reported no apparent problems with the moving-branch / three-leaf controlled-load GUI ("mi pare che non dia problemi"). This is visual feedback for that fixture, not acceptance of subsequent tomato/contact variants.

## Dynamic tomato on the outer leaf — 2026-09-17

Implemented as optional `--tomato`; controlled-force mode is unchanged. A 20 g rigid sphere of radius 20 mm, with red body and green calyx, is dropped from rest with 60 mm nominal clearance above the outer leaf's middle segment after gravity settling. There is no applied leaf force in this mode. CCD is enabled. Contact impulses come from the PhysX contact stream; visual meshes are not used as collision evidence. A floor catches the tomato, and it is parked only after rolling off the floor and descending below z = -0.2 m, or on manual removal/restart. GUI recording is bounded per trial.

Final evidence: `artifacts/leaf61/branch-tomato-final`. All checks passed, including target contact, falling, collider penetration <1 mm and at least eight seconds since the last leaf contact. Contact data recorded at physics frequency: 54 positive-impulse contact samples on leaf L002, from t = 9.10833 to 9.21250 s. Maximum reported collider penetration: **0.03206 mm**. This does not certify visual-mesh nonintersection.

- Branch centre motion from gravity equilibrium: **12.1493 mm**.
- Outer lamina final-segment motion in petiole frame: **38.5293 mm**.
- Other lamina relative responses: **0.1532 / 0.4128 mm**.
- Maximum sampled edge extension: **3.7380%**; minimum triangle area ratio: **0.96618**.
- Maximum attachment error: **0.000058 mm**.
- Final maximum segment recovery error: **0.2528 mm**; sampled residual motion in the last second: **0 mm**.
- Offscreen whole-trial frame work p95: **38.01 ms**. This includes diagnostics and is not a separated physics benchmark or a GUI FPS claim.

The earlier `branch-tomato-smoke` also passed, but allowed the tomato to keep falling after rolling off the small floor, yielding a meaningless 712 m final fall-distance maximum. The final version bounds that out-of-scene behavior; its branch and leaf response metrics are unchanged. The final run has no logged Error/Traceback. Lint, diff checks and the existing 28 unit tests passed; the new contact behavior was verified by the Isaac headless runs, not those unit tests.

Launch:

```bash
./run_leaf61_branch.sh --tomato
```

The first drop occurs automatically after nine simulation seconds. **Drop tomato / Repeat test** removes the old tomato, waits eight simulation seconds for settling and drops again. **Remove tomato** clears it. **Finish and save** closes the run. Visual acceptance for the tomato variant remains pending.

## Runtime optimization — 2026-09-17

User observed approximately 23 FPS in the original tomato GUI. That observation is distinct from the controlled offscreen measurements below. The old pacing slept after every physics step and added rendering time on top; optimized pacing waits only at visual frames against the simulated-time deadline, avoiding catch-up bursts after stalls.

Default runtime is now `optimized`. `--runtime baseline` retains the prior synchronization/pacing path for comparisons. Physics stays CPU/PGS at 480 Hz, with the same masses, geometry, joints, iterations and sleeping settings. Optimized runtime disables per-step USD transform/velocity publishing, explicitly publishes visible transforms at 30 Hz, caches UsdSkel animation channels, combines the tomato pose read with the leaf/branch batch and vectorizes attachment checks. Full-rate finite/attachment checks and contact reporting remain active.

Sequential offscreen evidence: `branch-perf-baseline`, `branch-perf-optimized`, and `branch-perf-optimized/comparison.json`. The comparison excludes initialization and gravity settling, measuring 390 frames over 13 simulated seconds (load start through recovery):

| Metric | Baseline | Optimized |
|---|---:|---:|
| Rendered wall-clock throughput | 30.65 FPS | 59.00 FPS |
| Frame work p95, excluding intentional pacing | 40.80 ms | 20.01 ms |
| Simulation step time, median per 16-step frame | 21.52 ms | 6.87 ms |
| Pose reads, median per frame | 0.223 ms | 0.197 ms |
| Diagnostics, median per frame | 2.902 ms | 2.695 ms |
| Skinning and explicit visual synchronization, median per frame | 0.407 ms | 0.974 ms |
| Rendering, median per frame | 6.139 ms | 5.546 ms |

The simulation-step bucket includes framework/synchronization overhead, not only the PhysX solver. Sum of component medians need not equal median total frame work. This is one run per mode, not three-repeat performance acceptance. At 30 renders per simulated second, optimized offscreen throughput corresponds to approximately 1.97× realtime; GUI is intentionally paced to 1× and remains capped at 30 Hz.

All physical checks pass. The complete recorded 10 Hz pose arrays (220 samples × 14 bodies × 7 coordinates) and tomato traces are exactly equal between the two runs; max sampled position difference is 0 m. This is sampled trajectory equivalence, not a claim about every physics substep. Existing 28 unit tests, lint and diff checks pass. No two-branch or full-plant performance extrapolation is established.

## Branch scaling — 2026-09-17

The user accepted the optimized single-branch GUI at approximately 30 FPS and requested scaling. Implemented `--branches 1|2|5|10`, with three skinned leaves and one simultaneously released tomato per branch. Each branch is an independent copy of the accepted hinged fixture, anchored to the world. There is no common dynamic stem. Cross-replica collisions are explicitly filtered; own tomato/leaf/floor contacts remain enabled. Layout positions are deterministic subsets of a four-column grid (300 mm X, 380 mm Y spacing). Camera framing expands with count; resolution and render settings stay the same.

Initial offscreen measurements before batching attachment diagnostics were 33.99 / 24.56 / 18.02 FPS for 2 / 5 / 10 branches. At ten branches the diagnostic median was 18.29 ms per frame. Batching rotations and attachment-frame checks across branches reduced it to 3.94 ms, while retaining full-rate finite and attachment checks. Dormant tomato checks reuse the parent pose batch. The ten-branch pose traces before/after this optimization are exactly equal at the recorded 10 Hz sample rate.

Final implementation, one sequential offscreen run per count, 390 measured frames over 13 simulated seconds; initialization, gravity settling and offline reporting excluded:

| Branches | Leaves / tomatoes | Offscreen mean FPS | Frame work p95 ms | Max trajectory difference from single branch, mm |
|---:|---:|---:|---:|---:|
| 2 | 6 / 2 | 36.62 | 37.57 | 0.000154 |
| 5 | 15 / 5 | 31.49 | 43.73 | 0.000185 |
| 10 | 30 / 10 | 25.13 | 61.23 | 0.000198 |

All physical/contact/recovery checks passed for every count, and each impacted lamina and branch responded. Position comparison is sampled at 10 Hz after subtracting each replica translation; all are within the unchanged 0.1 mm comparison tolerance. Summary: `artifacts/leaf61/branch-scaling-comparison.json`. Individual evidence: `branch-scale-{2,5,10}-batched-checks`.

These are preliminary single-run throughput measurements, not three-repeat performance acceptance. None meets the strict p95 <=33.33 ms gate for constant 30 FPS in this campaign; ten branches also exceeds 50 ms at p95, so its 25.1 FPS average does not establish constant >=20 FPS. The count of ten is physically checked but not promoted as a constant-30-FPS configuration. GUI remains paced to at most 30 Hz; offscreen FPS above 30 means simulation faster than realtime, not a higher GUI setting. No extrapolation to a full coupled plant is established.

A five-branch GUI is the next visual trial. Start it using:

```bash
./run_leaf61_branch_scale.sh --branches 5
```

Use `--branches 2` or `--branches 10` to inspect the other counts; the scale launcher defaults to 2. Buttons release/remove all tomatoes together. Each trial records all branches and contact evidence per tomato. The comparison is reproducible with `branch_compare.py --reference artifacts/leaf61/branch-perf-optimized --output PATH RUN_DIR...`.
