# Day-160 detachable fruit investigation, 2026-09-09

Historical first-phase report. On 2026-09-10 the user changed the final review
to one minute and requested 60 Hz / FPS comparisons; see [the follow-up](ITERATIONS_60HZ.md).

This is the first instrumented reproduction and bounded ablation comparison.
It does not certify a stable, interactively detachable plant. GUI acceptance is
pending and still requires three minutes including the user's Shift+click.

## Reproduction contract

- Checkpoint: annotated GitHub tag
  `checkpoint/v2-before-detachable-fruit-2026-09-09`, peeled remote commit
  `ac8793ea683998eab8d7318d6e30182535ce5543`.
- Experiment branch: `experiment/v2-detachable-fruit-stability`.
- Input: `data/plant_states/plant_state_day_160.json`, SHA-256
  `d6c8b490f190b793f6bec6dea7f955c506f7ff02fd2a87b5c5337d43196a87bc`.
- Shared full source USD SHA-256:
  `94fe77b9378ff6cacb4e7bde4723bcb1bc51b7281c95d61f670d536f651f332c`.
- Isaac Sim `4.5.0-rc.36+release.19112.f59b3005.gl`, PhysX extension
  `106.5.7`, RTX 4080 Laptop GPU. Four task/physics workers, 480 Hz.
- Flexible source physics, deterministic leaf seed 42; native external
  FixedJoints break at 6 N from the first measured step. Each case starts in a
  fresh process from the same authored pose. No mass or geometry tuning.
- Local archive: `artifacts/detachable_fruit_v2/2026-09-09/`. Each case includes
  source/scene/code hashes, audit, effective runtime properties, logs and
  compressed per-step traces. Original execution paths in those reports point
  to `/tmp/autotom-fruit-v2/`; the archive preserves those provenance records.

## Sanity findings and corrections

The adapter hardcoded exclusion and 6 N, bypassing `TrussPhysicsConfig`.
It now respects the configuration; existing defaults produce the same settings.

Isaac `World` overwrote authored PGS with TGS before initialization. The
experimental loader now restores the requested solver before reset and checks
the loaded solver, GPU mode and frequency. The first purported PGS run is
excluded; the corrected rerun really uses PGS.

The full source has 288 bodies: 216 articulated supports and 72 external
fruits. The root is fixed, with 412 articulation DOFs. All authored joint rest
frames agree within approximately 3 nanometers; the audit found no missing or
duplicate incoming connections. Duplicate *names* in the native articulation
metadata are warnings, not duplicate joint paths; diagnostics use full paths.

All 72 fruits use the same sphere prim for visualization and collision. Their
attachment is 2 mm inside that sphere's surface. Authored fruit/pedicel mass
ratios are **6.240–10.445**, not the initial geometric estimate 50–84. The
current global geometry scale is reflected in support mass, while the fruit's
explicit source mass is retained. That policy deserves a separate physical
calibration review; it has not been changed or established as the instability's
cause. Effective runtime masses and positive inertia eigenvalues were checked;
turning collision off preserved the cooked masses and inertias.

The old `feat/tomato-runtime-detachment` report describes a full-plant crash
after deleting an articulation link and rebuilding with `World.reset()`.
Its compound-body/proxy replacement remained WIP. Neither approach is imported
here; these experiments keep articulation topology fixed and use native break
events.

## Baseline ablations

Screens request 20 simulated seconds and stop at spontaneous breakage. Speeds
below are final-ten-second maxima only for completed screens; a dash means the
case stopped during startup. These are measured solver velocities, not claims
about visible jitter.

| Case | Bodies / fruits | First spontaneous break | Tail linear m/s | Tail angular rad/s |
|---|---:|---:|---:|---:|
| Complete, no fruit | 216 / 0 | None in 20 s | 0.01091 | 0.44337 |
| Complete, collision on, 6 N | 288 / 72 | 0.07708 s, four joints | — | — |
| Complete, collision off, 6 N | 288 / 72 | 0.07708 s, four joints | — | — |
| Complete, collision on, unbreakable | 288 / 72 | Disabled | 4.5892 | 219.22 |
| Complete, collision off, unbreakable | 288 / 72 | Disabled | 5.6550 | 247.49 |
| One external fruit, anchored articulated pedicel | 3 / 1 | None in 20 s | 0.00799 | 0.08606 |
| Selected entire truss | 24 / 8 | 0.07292 s | — | — |
| All trusses, vegetation removed | 208 / 72 | 0.06250 s | — | — |
| One internal fruit, unbreakable control | 3 / 1 | Disabled | 0.00387 | 0.13431 |

The unbreakable complete plant keeps moving strongly during seconds 10–20:
maximum attachment separation is 9.56 mm with collision and 11.16 mm without.
Contact is therefore **not necessary** for this failure, and disabling breakage
does not fix it. The eight-fruit truss is a much smaller reproduction.

The single-fruit controls have essentially fixed poses late in the run despite
nonzero reported solver velocities. The no-fruit baseline also fails the strict
numerical filter. For this reason the latest monitor adds independent COM and
angular finite differences, without silently relaxing the agreed thresholds.
No control is promoted merely from small displacement.

The final matched truss control disables breakage in **both** configurations:

| Truss attachment | Tail solver v / w | Tail pose-derived COM v / w | Tail attachment separation |
|---|---:|---:|---:|
| External, unbreakable | 11.995 mm/s / 0.2093 rad/s | 2.338 mm/s / 0.0201 rad/s | 7.85 micrometers |
| Internal, unbreakable | 10.441 mm/s / 0.1797 rad/s | 0.162 mm/s / 0.00537 rad/s | 0.029 micrometers |

Both complete 20 seconds and fail the raw solver-velocity filter. Unlike the
complete unbreakable plant, the reduced external truss settles in actual pose;
its 6 N failure is an early transient. This limits the reduction: it reproduces
the startup break but not the full plant's persistent unbreakable oscillation.
The internal control tightens the frame constraint, but cannot satisfy the
detachable requirement. The single external fruit was also rerun unbreakable
to match its internal control; its results agree with the 6 N single-fruit case.

## Bounded parameter comparison on the selected truss

The default articulated solver counts are 32 position / 4 velocity; external
fruit counts are 32 / 1. The selected fruit is
`/World/TerminalBodies/Truss_r6_o0_g421531_tomato_01`, the maximum authored ratio.

| Change from default | First spontaneous break |
|---|---:|
| Fruit position iterations 64, velocity unchanged | 0.00625 s |
| Fruit 64 / 4, existing stabilized preset | 0.00625 s |
| TGS, both velocity counts zero | 0.07292 s |
| TGS CPU, original counts | 0.16458 s |
| TGS CPU, both velocity counts zero | 0.16458 s |
| Truss damping ratio 1 | 0.05833 s |
| Truss damping ratio 2 | 0.06042 s |
| Truss damping ratio 7 | 0.08750 s |
| Damping multiplier 2 (ratio 8) | 0.07500 s |
| Damping multiplier 4 (ratio 16) | 0.05000 s |
| Damping multiplier 7 (ratio 28) | 0.02500 s |
| Truss stiffness 0.5, damping ratio retained at 4 | 0.11042 s |
| Truss stiffness 0.25, damping ratio retained at 4 | 0.06250 s |
| **Actual PGS, both velocity counts zero** | **None in 20 s** |

PGS substantially improves the reduced scene. Its final-ten-second maxima are
5.296 mm/s solver linear velocity, 0.0441 rad/s angular velocity, 0.537 mm
position excursion, 0.0676 mm attachment separation and 0.166 degrees attachment
rotation. Corrected independent COM finite differences reach 5.295 mm/s. The strict
20-second filter therefore still fails, although motion decays: during second
19, the maximum solver linear velocity is approximately 0.073 mm/s.
This merits checking the complete plant,
not declaring success or combining parameters immediately.

The solver comparisons are motivated by NVIDIA's documented Isaac 4.5
[physics limitations](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/physics/physics_resources.html).
These results establish behavior for this scene; they do not identify a unique
PhysX defect from that list.

### PGS carried back to the complete plant

`full-pgs-actual` completed 20 seconds with all 72 external fruits attached,
collision enabled and 6 N active. It still **fails** the agreed numerical gate:

| Final-ten-second quantity | Measured | Limit |
|---|---:|---:|
| Solver linear velocity | 6.002 mm/s | 5 mm/s |
| Solver angular velocity | 0.05189 rad/s | 0.05 rad/s |
| Independent COM velocity, recomputed from traces | 6.002 mm/s | 5 mm/s |
| Position excursion | 0.981 mm | 1 mm |
| Attachment separation | 0.402 mm | 0.5 mm |
| Attachment rotation | 2.463 degrees | 0.5 degrees |

The dynamic motion decays, but the attachment orientation error persists at
the final sample. The largest is 2.447 degrees for
`/World/TerminalBodies/Truss_r5_o0_g421786_tomato_04`; fruit `02` on the same truss
has 2.438 degrees. This is not the truss selected by maximum mass ratio. The
next focused diagnosis should include this actual worst residual constraint,
using PGS as the promising solver control. No stiffness/damping combination,
mass edit or architecture replacement has been promoted from this result.

## Evidence limitations

- `baseline-no-fruit` and `baseline-full` were interrupted during development
  of the monitor. They are not physics results. The latter exposed a worker/GIL
  deadlock in event delivery; events are now pumped on the Python simulation
  thread after each step.
- `truss-pgs-zero` actually loaded TGS and is invalid as a PGS comparison.
  `truss-pgs-actual` is the corrected run.
- The early `full-no-collisions` report's position excursion includes an old
  broken-fruit masking error; do not use that excursion. Its native break times
  and velocities remain valid. Subsequent monitoring removes broken fruit
  consistently from the entire settling window.
- Earlier physical gate failures could exit zero because Isaac used fast
  shutdown. Reports are authoritative for those cases. The runner now uses
  graceful shutdown and records exit status separately; extension teardown
  warnings are not counted as physics instability.
- Isaac 4.5 returns COM tensors shaped `(N, 1, 3)`. An early independent-velocity
  calculation assumed `(N, 3)` and broadcast across bodies. It now normalizes
  that shape, with a two-body regression test. Pre-fix **linear kinematic**
  fields in runtime reports are invalid. The corrected values above come from
  `recheck_pose_velocities.py` and each case's `pose-velocity-check.json`, using
  the original saved poses. Native velocities, quaternion angular differences,
  attachment errors and break events are unaffected. Two monitor regression
  runs also exposed a NumPy boolean JSON conversion and this extra dimension
  in failure reporting; those incomplete reports are excluded from results.
- This checkpoint has not passed the 120-second, repeated-start, force-ramp,
  or three-minute GUI gates. The GUI and force-ramp paths are implemented but
  remain unvalidated until a suitable full-plant candidate is selected.

## Implementation validation

- Focused numerical, topology, geometry, configuration and serialization suite:
  **15 passed, 1 skipped**. The skipped runtime schema suite requires Isaac's
  PhysX schemas, which the project `uv` environment does not provide.
- Existing Isaac loader/timing argument checks: **6 passed**. Earlier focused
  truss and PlantState regressions: **56 passed, 7 deselected**.
- The final monitor rerun (`monitor-final-regression-fixed`) reproduces the
  same native break at 0.0729167 s on truss fruit `05`. It successfully writes
  the event, pre/post-break state, support chain, effective configuration and
  failed physical gate, and returns exit code 1.
- `git diff --check` passes. Generated scenes, logs and figures are ignored by
  Git and archived locally. Normal fruit-free launch defaults are unchanged.
