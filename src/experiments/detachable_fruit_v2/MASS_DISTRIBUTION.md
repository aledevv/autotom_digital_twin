# Why the fruit mass profile changes startup breakage

2026-09-14. New diagnostic objective after the lateral search was stopped.
The direct-stem standard preset is unchanged; no lateral configurations are
reopened. User requested stopping work by 16:00 local and resuming tomorrow.

## Controlled question

Previous reciprocal mass swaps established sensitivity to the eight-fruit profile,
but changed total mass as well as spatial distribution. This matrix separates them
on ONE rank-10 geometry, full original fixed stem, same joints/drives, inertias,
colliders, TGS/GPU 60 Hz, articulation 32/4, fruit 32/1, 6 N from startup.
No mouse input; 20-second rest screens stop on the first unwanted break.

All six scenes have the same attributes except the eight fruit `physics:mass`
values. The common preparer authors the previously loaded inertias explicitly.
Runtime expected mass/inertia/COM checks remain enabled. Mass-only controls are
causal probes, deliberately not geometrically consistent biological replacements.
The original 57.2251 g and rank-6 74.7085 g profiles are the two total-mass levels.

Preparation and execution use fresh local folders:

```bash
UV_CACHE_DIR=/tmp/autotom-uv-cache uv run --no-sync python \
  src/experiments/detachable_fruit_v2/prepare_mass_distribution.py \
  --output artifacts/detachable_fruit_v2/mass-distribution-v1
```

`run_batch.py` receives each of the six prepared directories sequentially.
`summarize_mass_distribution.py ROOT --output FILE` regenerates the compact
results from reports, effective properties and body traces. Heavy files remain
under `artifacts/detachable_fruit_v2/mass-distribution-v1/`.

## Completed results

| Spatial profile | Total fruit mass | Result |
|---|---:|---|
| Original rank 10 | 57.2251 g | Break at 0.200 s |
| Rank 6 | 74.7085 g | 20 s functional pass |
| Rank 6 normalized to rank-10 total | 57.2251 g | 20 s functional pass |
| Rank 10 normalized to rank-6 total | 74.7085 g | Break at 0.150 s |
| Rank-10 masses in reverse order | 57.2251 g | 20 s functional pass |
| Eight equal masses | 57.2251 g | 20 s functional pass |

All six processes finished. Both failures are actual JOINT_BREAK events and
screening stops, not claimed GUI crashes. Six full-stage comparisons verify
identical prims/types/relationships/attributes except fruit mass. Loaded-property
checks report no errors. Python compilation and diff checks passed.
[Compact results](mass_distribution_results.json) include hashes, events, tail
advisories and step-average resultant estimates. Static isolation audit is local
at `mass-distribution-v1/static-isolation.json`.

The two-by-two profile/total comparison establishes that the benefit of rank 6
is not just its larger total mass: at BOTH totals, profile 6 passes and profile
10 fails. Reversing the SAME set of masses cures the rest-screen failure without
changing total or the range of individual masses. Uniform mass at the lighter
total also passes. Thus neither total weight nor a global minimum/maximum mass
ratio alone explains this result. Spatial placement and coupled dynamics matter.
This does not yet distinguish a local distal mass/support sensitivity from a
whole-truss dynamic mode or numerical constraint-force issue. In particular,
these data do not establish resonance, a universal safe profile, or a physical
6 N transient independently of PhysX's break decision.

## Interpretation boundary

A break event proves PhysX broke the constraint, not that the biological fruit
would experience the same force. The fruit-only resultant estimated from
`m * (delta_velocity / dt - gravity)` across whole recorded steps is NOT the
constraint's internal force sensor or a bound on substep/solver impulses.
In the old original rank-10 trace, that estimate for the first breaking fruit
peaks near 0.1885 N over the first 0.2 s, far below the authored 6 N threshold.
The corresponding rank-6-mass control is about 0.1670 N. The estimate cannot
exclude internal oscillating/cancelling impulses, and is not proof of a false
break. It motivates inspecting solver reaction reporting separately from motion.

NVIDIA documents that the constraints are resolved iteratively and discusses
mass/inertia conditioning in its [joint documentation](https://nvidia-omniverse.github.io/PhysX/physx/5.3.1/docs/Joints.html).
A [2023 NVIDIA discussion](https://github.com/NVIDIA-Omniverse/PhysX/discussions/101)
reports a TGS/nonzero-velocity-iterations force-reporting issue. This is historical
context, NOT proof that the installed Isaac 4.5 build has that defect or that it
causes these breaks. Previous iteration-zero controls did not cure other fixtures.

The loaded structure has unchanged force-drive stiffness/damping while mass is
moved among its nodes. That changes coupled inertia and modal response, even at
constant total weight. Calling this resonance, a particular mass-ratio limit,
or a unique implementation bug would require evidence we do not yet have.

## Next diagnostic after discussion

After the matrix, distinguish local distal-fruit sensitivity from distributed
support coupling: targeted mass swaps at fixed total, and a consistent comparison
of pre-break poses/velocities/DOFs. If examining reaction forces, establish what
is actually exposed for the EXTERNAL fruit FixedJoint; articulation incoming-force
APIs alone do not measure that joint. A matched timestep/solver comparison on this
same fixture can distinguish robustness of the result from other earlier scenes.
No further physical configurations are launched in this session after the matrix.
