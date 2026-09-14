# Lateral support diagnosis — standard truss integration

The direct-stem fixture remains preserved. No exporter preset defaults were changed.
The standard truss on `Branch_s2_o0_g421408` adds a moving support chain absent
from the positive direct-stem fixture: four spherical articulation joints, two
angular DOFs each, versus fixed stem joints. The branch weighs 48.88 g and the
whole standardized truss (including fruit) weighs 175.24 g.

## Controlled results

All cases restart from the same authored pose. GPU, 60 Hz, articulation 32/4,
fruit 32/1, collision geometry, masses, drives and 6 N break force are preserved
except for the explicitly named factor. Native mouse is not exercised headless.

| Variant | Result |
| --- | --- |
| Flexible branch, contacts on | Same fruit breaks at 0.333 s |
| All four branch joints fixed, contacts on | 20 s functional pass |
| Flexible branch, contacts off | Same fruit breaks at 0.333 s |
| All four branch joints fixed, contacts off | 20 s functional pass |
| Flexible branch, actor origins at branch COMs | Same fruit breaks at 0.333 s |
| Flexible branch, TGS velocity iterations 0/0 | Same fruit breaks at 0.333 s |
| Flexible branch, PGS (only solver changed) | 20 s and independent 60 s functional passes |

The contact-enabled and disabled flexible cases have exactly equal recorded
float32 body states through failure. All 34 effective body mass/inertia/COM
checks pass; collision removal is performed after cooking and rechecked after
the first step. Thus contacts do not explain this particular early failure.

The original branch joints remain within their +/-30 degree limits before
failure: the smallest sampled margin is 3.80 degrees. Branch joint anchors
initially coincide within numerical precision (largest gap below 3e-17 m),
with maximum rest-frame angular discrepancy around 0.0049 degrees. Runtime
inertia tensors are finite and positive; authored zero inertia entries request
automatic computation and are not evidence of zero physical inertia.

The support falls before the fruit breaks. Its attachment drops about 13 cm
by 0.267 s and 21 cm by the final 0.333 s sample in TGS. PGS still produces
substantial initial sag (about 24 cm by 0.35 s). At 20 s it has recovered to
attachment Z ~0.081 m versus initial ~0.260 m. Do not describe PGS as fixing
support stiffness or validating the biological response. It prevents the
spontaneous break in these two runs; manual motion and detachment remain gates.

Event times in the compact report are assigned at the completed physics-step
sample; the raw callback log can show the previous step's time (one 60 Hz step
earlier). Compare sample indices consistently rather than claiming substep timing.

## NVIDIA compatibility checks

[Isaac Sim 4.5 limitations](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/physics/physics_resources.html)
documents spherical articulation drive/limit issues with nonidentity COM frames,
and TGS D6 drive issues, especially with velocity iterations. Runtime joint
metadata confirms the branch joints are spherical. The COM-origin comparison
preserves collider/visual world transforms and joint world frames, and checks
loaded inertias and the changed local COM coordinates. It did not cure this case.
TGS with zero velocity iterations also did not cure it. PGS is a useful empirical
candidate, but these results do not establish a specific NVIDIA bug as the cause.

The current evidence points to solver sensitivity during the loaded flexible
chain's initial motion. It does not justify removing lateral trusses, globally
fixing branches, or changing biological masses. The substantial sag remains a
separate behavior for user review. No combined factor sweep was performed.

## Reproduce

Local heavy evidence is under
`artifacts/detachable_fruit_v2/standard-integration/lateral-diagnosis-v2/`.
The compact report is `lateral_diagnosis_results.json`. Each variant has a
configuration, USD hash, authored-property diff, effective properties, per-step
body traces and DOF traces including loaded limits. The source is the immutable
`../lateral-screen` directory. Use a new output directory for each run:

```bash
UV_CACHE_DIR=/tmp/autotom-uv-cache uv run --no-sync python \
  src/experiments/detachable_fruit_v2/prepare_lateral_diagnosis.py \
  artifacts/detachable_fruit_v2/standard-integration/lateral-screen \
  /tmp/lateral-pgs-test --variant pgs --duration 60
UV_CACHE_DIR=/tmp/autotom-uv-cache uv run --no-sync python \
  src/experiments/detachable_fruit_v2/run_batch.py /tmp/lateral-pgs-test
```

Launch the prepared candidate manually (fresh evidence directory, no automatic close):

```bash
UV_CACHE_DIR=/tmp/autotom-uv-cache uv run --no-sync python \
  src/experiments/detachable_fruit_v2/run_prepared_gui.py \
  artifacts/detachable_fruit_v2/standard-integration/lateral-diagnosis-v2/pgs-minute
```

GUI candidate uses native joint Shift+click coefficient 10, no custom controller,
with 6 N break force from startup. Require 60 simulated seconds including manual
pull/detachment and final recovery; GUI mean >=20 FPS at 1280x720 and user approval.
The functional headless passes do not imply the strict numerical tail gate passes.

Validation: nine standard-truss tests and eighteen diagnostics tests passed.
The seven screening cases and the independent PGS minute were executed in Isaac.
