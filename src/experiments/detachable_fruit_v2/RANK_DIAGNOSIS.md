# Why rank 10 breaks while rank 6 survives

2026-09-14. Diagnostic follow-up to MULTIPLE_TRUSSES.md. These are rest screenings,
not manual acceptance or a new production physics configuration.

## Controlled results

All cases use the original full stem, eight fruits, native interaction unchanged,
6 N break threshold, original support density/drives, TGS/GPU 32/4 and 32/1.
Each starts fresh; maximum requested duration is 20 simulated seconds. Unless
specified, use 60 Hz and corrected original fruit dimensions. Loaded body mass,
inertia and COM are checked against expectations, including when contacts are
removed. Explicit diagonal inertias prevent collider removal from changing inertia.

| Rank / intervention | Result |
| --- | --- |
| 10 original, repeated with explicit loaded inertias | Same lat_3_R break at 0.20000001 s |
| 10, all collisions disabled | Same break at 0.20000001 s |
| 10, verified zero gravity | Pass 20 s; tail maximum linear speed 2.43e-7 m/s |
| 10, rank-6 fruit masses AND inertias | Functional pass 20 s |
| 10, rank-6 fruit masses ONLY | Functional pass 20 s |
| 10, rank-6 fruit inertias ONLY | Same break at 0.20000001 s |
| 6, rank-10 fruit masses ONLY | Corresponding lat_3_R break at 0.20000001 s |
| 10 original, 120 Hz ONLY | Functional pass 20 s |

Report times above are the completed-step event attribution, at 60 Hz resolution.
The pre-step console callback timestamp can read 0.183333; do not interpret this
as a different trial or finer force measurement. Screens stop at the first break;
subsequent collapse is not tested. Functional pass is not a strict all-velocity
numerical pass: raw tensor velocity advisories remain, including the 120 Hz case.
No GUI interaction or FPS measurement was conducted in this diagnostic follow-up.

## What the audit establishes

All corresponding drive stiffnesses, damping values and joint limits match between
ranks 6 and 10. Rachis link masses are 10.0531 g each and pedicels 7.5398 g each
in both. Both have four rachis links and eight pedicels/fruits. Stem links are
connected by FixedJoints. Static frame audits report no errors; maximum reset
projection is approximately 0.11 micrometres for rank 10.

Fruit mass profiles in pedicel order 0L, 0R, 1L, 1R, 2L, 2R, 3L, 3R (grams):

- Rank 6: 10.6318, 10.4638, 10.1265, 9.6885, 9.3472, 8.7760, 8.1546, 7.5201.
- Rank 10: 6.4163, 6.3519, 6.4311, 6.7043, 6.9793, 7.4933, 8.0924, 8.7565.

Rank 10 has LESS total fruit mass (57.2251 versus 74.7085 g) and LESS initial
fruit-only gravity moment about the rachis base (0.06489 versus 0.07806 Nm).
Thus total weight alone does not explain failure. The loading distribution differs.
The mass-only diagnostic deliberately decouples mass from geometric density and
inertia; it isolates sensitivity, and is not a biologically consistent replacement.

## Causal interpretation and limits

The failure follows the fruit mass profile in both transfer directions, with
geometry, frames, axes, location and inertia left unchanged in each recipient.
Other trusses and contacts are unnecessary. Gravity initiates the response;
no appreciable motion is observed in its verified absence. The original case
survives the screening when only the timestep is halved.

This supports a load-dependent startup response that is sensitive to temporal
resolution in this solver/drive configuration. It does not demonstrate erroneous
biological masses, prove a specific PhysX defect, or establish that changing one
particular fruit alone suffices. Joint reaction-force histories were not measured;
JOINT_BREAK proves the break event, not the continuous force waveform or its exact
source within the coupled constraint solution. Structural frame bugs were not
found in this audit; a universal absence of construction issues is not claimed.

The useful next candidate preserves original masses and geometry: verify the
five-truss scene at 120 Hz, then manual native dragging and measured GUI FPS. If
60 Hz remains necessary, study solver/drive sensitivity separately rather than
silently replacing fruit masses. No production mass/drive change is made here.

## Instrumentation correction and evidence

The first no-gravity attempt was INVALID: World/PhysicsContext replaced authored
zero gravity with 9.81 m/s2. Kept its artifacts and marked it excluded. Added an
opt-in `diagnostic_gravity_magnitude` runtime setting before reset and a loaded
value assertion in the monitor. Ordinary runs without that setting are unchanged.
The repeated 60 Hz control reproduces the original failure after this change.

`prepare_rank_diagnosis.py` writes fresh named scenes, source hashes, exact
attribute changes, static audits and expected loaded properties. Detailed compact
results are in `rank_diagnosis_results.json`; heavy evidence remains local under
`artifacts/detachable_fruit_v2/rank-diagnosis/`. No failing GUI was opened.
