# Rank-10 breakup: iteration-dependent external-joint break behavior

2026-09-15. Follow-up to [mass distribution](MASS_DISTRIBUTION.md).
No production preset, mass profile, mouse implementation or break threshold was
changed. Native joint-break events remain authoritative for actual detachment.

## Spatial localization

All four fresh controls preserve the original rank-10 total mass and geometry,
inertias, joint properties, TGS/GPU 60 Hz and 6 N. Only named mass pairs swap:

| Mass swap | Rest-screen outcome |
|---|---|
| First/last on left only | Right distal fruit breaks at 0.200 s |
| First/last on right only | 20 s functional pass |
| First/last on both sides | 20 s functional pass |
| Left/right distal fruit only | Left distal fruit breaks at 0.1833 s |

The failure follows the larger distal mass under these controls. This alone
would not distinguish mechanical load sensitivity from solver break evaluation.
[Results](mass_distribution_localized_results.json) retain all events and hashes.

## Quantitative relationship in the truss traces

For each fruit compute the frame-average nongravitational resultant estimate
`F_est = m * (delta_v / dt - gravity)` from native COM velocities, using the
zero initial velocity. This is NOT a substep force sensor. For TGS/GPU controls,
compare `position_iterations * |F_est|` against the authored 6 N threshold.

Across twelve completed original/profile/local-swap/64-iteration/128-iteration
runs, this predicts the first break's exact sampled step and fruit set, including
all cases with no break. Original rank 10 reaches ~0.18849 N at step 12:
32 * 0.18849 = 6.0318 N. Rank 6's original mass control stays below the scaled
threshold. Increased position iterations make even smaller resultants cross it,
explaining why adding iterations previously made breakup occur earlier.

This was a retrospective observation, then tested prospectively on an analytic
hanging-mass fixture. The script retains mismatches as well as matches; the
relationship is not asserted for every solver or attachment type.
[Retrospective evidence](break_scaling_retrospective.json).

## Prospective hanging-mass test

Source: `support-ablation/main-rigid`. Two fixed articulation links (zero DOFs),
one external fruit FixedJoint, no contacts, no input, gravity 9.81 m/s². Same
6 N threshold. Geometry and inertia held fixed while mass is set explicitly;
this is a numerical probe, not a biological replacement.

| Solver/backend | Position iterations | Mass / actual weight | Result |
|---|---:|---:|---|
| TGS/GPU | 32 | 10 g / 0.0981 N | Pass 5 s |
| TGS/GPU | 32 | 20 g / 0.1962 N | Break first step |
| TGS/GPU | 64 | 10 g / 0.0981 N | Break first step |
| PGS/GPU | 32 | 20 g / 0.1962 N | Pass 5 s |
| TGS/CPU | 32 | 20 g / 0.1962 N | Pass 5 s |
| TGS/GPU, independent kinematic support | 32 | 20 g / 0.1962 N | Pass 5 s |

The weight is far below 6 N. Before the TGS/GPU 20 g break the fruit COM velocity
is only about 0.00008 m/s; reset projection ~0.19 micrometres. Loaded mass and
6 N are verified. An oscillating rachis, distal distribution, or flexible branch
is not necessary for the anomalous break. The counterexample on CPU narrows the
behavior to the GPU path tested here, not TGS universally.
[Analytic probe results](break_scaling_probe_results.json).

For these GPU external-articulation fixtures, the observed threshold behaves
like approximately 6/N N: 0.1875 N at N=32, 0.09375 N at N=64. This is an empirical
statement about these controls, not a recommendation to multiply breakForce by
N or to change biological masses. A source-level diagnosis of the installed
PhysX binary has not been made.

## Native force-scheduling flag: tested, not a remedy

NVIDIA documents TGS substeps and the scheduling of gravity/external forces in
[Simulation](https://nvidia-omniverse.github.io/PhysX/physx/5.7.0/docs/Simulation.html#tgs-force-application).
The installed Isaac 4.5 schema exposes
`physxScene:enableExternalForcesEveryIteration`, default false.
This provides a plausible mechanism to test, not evidence by itself.

Changed ONLY this flag to true on the hanging 20 g case and original rank 10.
Full attribute/relationship comparison confirms isolation. Both post-reset
scene reports contain true, but both still break at the original times (first
step / 0.200 s). Thus this flag does not cure these cases in the tested runtime.
Do not promote it based on the newer documentation or describe the scheduling
mechanism as confirmed. A historical NVIDIA force-reporting discussion likewise
does not identify the installed binary's exact defect.

## Reproduction and validation scope

- `prepare_mass_distribution.py --matrix localized --output NEW_ROOT`
- `prepare_break_scaling_probe.py --output NEW_ROOT`
- `prepare_force_scheduling_control.py --output NEW_ROOT`
- `prepare_break_scope_probe.py --output NEW_ROOT`
- Execute prepared directories with `run_batch.py`; one Isaac process at a time.
- `analyze_break_scaling.py ROOT... --output FILE` saves event predictions and
  COM estimates; it is diagnostic analysis, not a force measurement API.

Local roots: `mass-distribution-localized`, `break-scaling-probe`,
`force-scheduling-control`, `break-scope-probe-v2`, `break-scope-probe-v3`, all
under `artifacts/detachable_fruit_v2/`. The first prepared world-anchored variant
was not run because the current attachment monitor requires a body0. It was
replaced by a stationary kinematic support outside the articulation. Its first
run exited before physics sampling because entity metadata was missing; that
run is excluded, and the preparation metadata was corrected for a fresh retry.

Python compilation and whitespace checks pass. Actual loaded-body checks run
in every physical trial. Rest-screen passes do not certify mouse detachment,
full-plant recovery, realistic tissue properties, GUI FPS or a general solution.

## Conclusion and exact scope

The independent kinematic-support retry completes 5 s with zero breaks or
monitor errors. Thus the same mass, external joint threshold and TGS/GPU setting
survive when the parent is an immobile body OUTSIDE the articulation. The original
fixed articulation remains in the scene, but no longer supports the fruit.
Together with the CPU and PGS controls, this localizes the observed scaling to
the tested TGS/GPU external-articulation constraint path. It is not a generic
maximum fruit mass or an intrinsic defect in the GroIMP profile.

The causal explanation supported here is: moving masses along the truss changes
individual attachment loads during startup; in this runtime path the break
criterion behaves as if those frame-average loads were amplified by the position
iteration count. Rank 10 crosses that anomalously low effective threshold, while
rank 6, uniform, reversed and the right-distal swap remain below it in these
screens. The same threshold behavior reproduces without truss flexion at all.
The exact native source line and the internal impulse accumulation/conversion
mechanism remain unverified. This diagnosis applies to the FIRST spontaneous
break; it does not by itself explain every later instability observed in v2.3.

[Scope controls and force-scheduling results](break_scaling_scope_results.json).
The flag-only USD comparison is local `force-scheduling-control/static-isolation.json`.
Kinematic preparation attempts v2/v3 failed diagnostic setup/topology gates and
are excluded from physical outcomes. Version v4 passes. The monitor now permits
an explicitly listed diagnostic kinematic support only after checking its role,
kinematic flag, unique identity and absence from the native articulation links;
ordinary expected topology checks are unchanged. Eighteen diagnostics tests pass.

No empirical multiplication of the 6 N threshold, no mouse workaround, no
standardized-mass promotion and no default solver change is made. Next engineering
validation would carry the original GroIMP masses into a matched CPU/TGS or
GPU/PGS truss trial and then native GUI detachment. Those are not validated by
this numerical diagnosis and have not been launched as a new campaign here.
