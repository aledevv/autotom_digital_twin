# Rachis startup oscillation controls

Scope: corrected input-size fruit, support density2000 kg/m³, original native joint
mouse, breakForce6N, TGS/GPU60Hz, articulation32/4, fruit32/1. All ten articulation
DOFs remain available. No geometry, mass, solver, limits or pedicel drive changes.

The user reports that rigidity is already reasonable; excessive oscillation is
the primary concern. Independent optional multipliers target only rachis entry
and internal joint drives. No production default is changed.

Each fresh case is screened for20 simulated seconds; stop on spontaneous break.
The existing corrected-fruit baseline is reused with the same measurement method.
Actual traced body positions (not reported PhysX velocities) measure motion of
rachis link origins. Initial drop is relative to the first measured step, and
vertical overshoot is below the final5-second mean. These are proxies for startup
motion, not an identification of biological damping or natural frequency.

| Stiffness multiplier | Damping multiplier | Outcome | Initial drop cm | Vertical overshoot cm | Travel in first5s cm |
|---:|---:|---|---:|---:|---:|
|1|1|Pass20s|8.83|2.70|53.76|
|1|2|Spontaneous break0.217s|—|—|—|
|1|4|Spontaneous break0.200s|—|—|—|
|2|1|Pass20s|6.83|2.65|52.78|
|4|1|Pass20s|4.43|1.97|39.16|
|2|2|Pass20s|8.73|2.67|52.83|
|4|2|Pass20s|6.75|2.61|51.64|

Increasing drive damping alone is not an effective remedy in this runtime.
Combined changes demonstrate interaction: failures of damping-only cases do not
invalidate stiffness effects, nor prove damping is generally useless. The unusual
response requires further diagnosis before attributing it to a specific solver
mechanism. No biological validity is claimed for these parameters.

Best measured startup candidate: stiffness4/damping1. Compared with baseline,
initial drop is50% smaller, max speed falls from0.570 to0.348m/s, total initial
travel falls27%, vertical overshoot falls27%. This is a partial improvement from
stiffening, not evidence of increased damping; the user must assess whether this
tradeoff is acceptable. GUI trial `gui-freeze-k8id8gmy` opened for comparison.
Native interactive detachment/recovery of this candidate remains to be reviewed.

```bash
UV_CACHE_DIR=/tmp/autotom-uv-cache uv run --no-sync python \
  src/experiments/detachable_fruit_v2/run_gui_freeze.py \
  --coherent-fruit --support-density 2000 --rachis-stiffness-scale 4
```

`--rachis-damping-scale` controls damping independently. Defaults for both
multipliers remain1. Eleven unit tests pass; scalar controls are checked to leave
pedicel, geometry, mass, limits and other properties unchanged. Full measurements,
scene hashes and report hashes are in `rachis_motion_results.json`; compressed
per-step traces and Isaac logs remain in local artifacts.

## Manual decision: reject stiffness increase

The user reports: fewer rebounds, but rachis feels like a stick; retain the previous
configuration. The stiffness4/damping1 candidate is **rejected** despite improved
startup metrics. Restore stiffness1/damping1 with input-size fruit and support
density2000. Do not promote any drive variant. Further angular-body damping was
mentioned as a possible diagnostic but was not implemented or tested after the
user chose to retain the previous behavior. The oscillation issue remains open.

Baseline command:

```bash
UV_CACHE_DIR=/tmp/autotom-uv-cache uv run --no-sync python \
  src/experiments/detachable_fruit_v2/run_gui_freeze.py \
  --coherent-fruit --support-density 2000
```

## Intermediate stiffness and3N request

The user subsequently requested an intermediate stiffness and3N detachment. Chosen
intermediate: stiffness2/damping1, preserving all other settings. Added optional
`--fruit-break-force`, changing only fruit joint breakForce and recorded config.
The existing stiffness2/6N case passed20s; fresh `rachis-k2-3n-screen` loaded3N
(confirmed in effective attachments) and failed at0.1333 simulated seconds with
spontaneous JOINT_BREAK during startup. No manual GUI is opened for this failed
combination. No automatic threshold, gravity ramp or constraint deletion was
introduced. The working6N reference remains unchanged;3N is not promoted.
Twelve targeted tests pass, including isolation of fruit breakForce changes.

Prepared candidate flags (failed at3N; for reproducing the diagnosis):
`--coherent-fruit --support-density 2000 --rachis-stiffness-scale 2 --fruit-break-force 3`.
To inspect the already screened intermediate stiffness at6N, omit the last option.
