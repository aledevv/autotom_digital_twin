# Native v2.3 stability: construction audit and factorial controls

Checkpoint: `524e29b`, experimental branch only. Custom checkpoint `e1908c6`
and main remain unchanged. Local evidence root:
`artifacts/detachable_fruit_v2/2026-09-11/native-stability/`.

## Construction audit

Fresh original-builder snapshots and day160 inputs are recorded under
`../2026-09-10/native-comparison/{main,v23}/provenance.json` and `direct/build.json`.
Both cases retain the complete stem, original stem anchoring, direct rank6
truss and eight corresponding fruits. Removed vegetation contributes no mass.
Main attaches to stem link7, v2.3 to link6; these are corresponding constructions,
not identical input geometry. This comparison cannot establish biological accuracy.

| Property | Main | v2.3 | Classification |
|---|---:|---:|---|
| Stem links | 10 | 10 | Construction |
| Rachis links | 4 | 7 | Construction; unchanged |
| Total bodies | 30 | 33 | Construction |
| Rachis link length/radius, m | .04 / .002 | .024 / .003 | Construction; unchanged |
| Pedicel length/radius, m | .03 / .002 | .018 / .003 | Construction; unchanged |
| Total fruit mass, kg | .074708533 | .074708533 | Equal |
| Stem mass, kg | .102768382 | .109999486 | Construction; unchanged |
| Rachis mass, kg | .040212385 | .009500177 | Construction; unchanged |
| All pedicels mass, kg | .060318578 | .008143008 | Construction; unchanged |
| Filtered initial overlaps | 39 | 56 | Connected-body collision filtering |
| Active conservative overlaps | 0 | 0 | No detected initial contact conflict |
| Maximum reset translation, m | 6.02e-8 | 4.15e-8 | Negligible projection |
| Maximum independent K/D relative error | 5.36e-8 | 4.74e-8 | Float rounding |

Static frame/connection/mass checks found no errors. Loaded mass/inertia checks
found no errors; eigenvalues are positive. Capsule/sphere overlap checks are
geometrically exact, cylinder checks conservative. No demonstrated structural
bug has yet been identified. This does not establish dynamic correctness.

Joint drives are force drives about rotX/rotY with zero targets; translation and
rotZ are locked. Independent calculations reproduce EI/L, series half-link
attachment compliance, local pivot inertia, damping, gain scaling and the
angular degree conversion. Rachis elastic modulus is 3e9 Pa in main and 20e9 Pa
in v2.3; pedicel modulus is 3e9 versus 4e9 Pa. Geometry also enters stiffness
through radius to the fourth power. The large coefficient differences are
explained by the formulas, not a demonstrated unit-conversion bug.

| Drive role | Main K | v2.3 K | Main D | v2.3 D |
|---|---:|---:|---:|---:|
| Rachis attachment | .0277600 | .0731567 | .000714221 | .000146775 |
| Rachis internal | .0164493 | .9252754 | .000549790 | .000521989 |
| Pedicel attachment | .001879925 | .07791793 | .000120809 | .000098829 |

These are authored USD angular coefficients (degree convention). Damping uses
local link pivot inertia rather than the full moving subtree. Attachment gains
use nominal parent dimensions. These are modelling choices/suspects, not proven
bugs. `audit_native_pair.py` writes the per-joint arithmetic, effective settings,
inertia checks and initialization evidence to `construction-audit.json`.

## Controlled factors

All cases restart from the same v2.3 source, TGS/GPU60 Hz, articulation32/4,
fruits32/1, break force6 N, native joint mouse coefficient10; no input during
screening. The new baseline USD is byte-identical to the earlier failed baseline.

- A: main stiffness matched by joint role and axis; original damping retained.
- B: main damping matched by joint role and axis; original stiffness retained.
- C: zero velocity iterations for the articulation and all eight external fruits;
  TGS/GPU and position counts32 remain unchanged.

C is motivated by the [Isaac Sim 4.5 documented D6 drive limitations with TGS and
velocity iterations](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/physics/physics_resources.html).
It is a diagnostic intervention, not an attribution of this failure to that issue.
Each case saves all old/new attributes, source/reference/implementation hashes,
effective loaded configuration, per-step metrics and first failure. No axes,
limits, targets, masses, geometry, segmentation or collision filters are changed.

| Factors | First spontaneous break, simulated seconds | Fruit | Screening |
|---|---:|---|---|
| None | .116667 | 07 | Failed |
| A | .050000 | 01 | Failed |
| B | .116667 | 07 | Failed |
| A+B | .066667 | 01 | Failed |
| C | .116667 | 07 | Failed |
| A+C | .050000 | 01 | Failed |
| B+C | .116667 | 07 | Failed |
| A+B+C | .066667 | 01 | Failed |

The monitor stops at the first spontaneous break; these runs do not characterize
the later collapse. Earlier break under A does not prove A has no contribution
to a combined fix. None of these four configurations is a stable candidate.
No manual trial or GUI FPS claim follows from a failed headless screening.

C reduces the baseline support angular-speed envelope at step3 from6.70 to
5.20 rad/s, while the first break remains at the same step7. With A alone,
step3 angular speed instead changes from15.66 to16.40 rad/s under C. This is
evidence of configuration-dependent partial effects, not a stable improvement.
All eight cases fail, so no candidate is advanced to GUI, repeated60-second
validation, lateral support or the full plant. No fourth factor or architectural
change has been implemented. Discuss the next structural diagnostic first.

Loaded-stage iteration properties were verified after reset. These are authored
PhysX settings, not instrumentation of the internal solver's per-island iteration
execution. Mass, inertia and body velocities come from runtime physics views.

`native_factorial_results.json` records compact results and hashes of the full
local manifests/reports. The full `matrix-summary.json`, per-step traces and USDs
remain local. `summarize_native_matrix.py` regenerates both summaries.
Validation:26 focused tests passed (factor isolation, diagnostics, native observer).
The audit independently checks positive inertia eigenvalues and triangle
inequalities for every loaded body in both original constructions.

## Reproduction

Use a fresh output directory for each invocation. Example A+B+C:

```bash
UV_CACHE_DIR=/tmp/autotom-uv-cache uv run --no-sync python src/experiments/detachable_fruit_v2/prepare_native_comparison.py \
  --build-dir artifacts/detachable_fruit_v2/2026-09-10/native-comparison/v23/direct \
  --run-dir artifacts/detachable_fruit_v2/2026-09-11/native-stability/main-drives-zero-velocity \
  --variant main-drives --zero-velocity-iterations \
  --reference-usd artifacts/detachable_fruit_v2/2026-09-10/native-comparison/main/tgs-6n-preflight/scene.usda
UV_CACHE_DIR=/tmp/autotom-uv-cache uv run --no-sync python src/experiments/detachable_fruit_v2/run_batch.py \
  artifacts/detachable_fruit_v2/2026-09-11/native-stability/main-drives-zero-velocity
```

The first passing candidate goes to manual native GUI review before completing
the remaining matrix. Acceptance, three independent60-second runs, lateral
support and full-plant validation remain pending until a candidate passes.

The subsequently approved support-removal diagnosis is recorded in
[SUPPORT_ABLATION.md](SUPPORT_ABLATION.md). It reproduces v2.3 failure with only
one fruit once the stem/rachis support chain is present.
