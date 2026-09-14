# Full-truss transition from main to the latest experimental setup

Reference: main rank6, complete stem, four rachis links, eight pedicels/eight fruits,
30 rigid bodies and24 articulation DOFs. Same day160 input and original anchors.
Every case uses TGS/GPU60Hz,32/4 articulation and32/1 fruit iterations, native
joint mouse coefficient10 and6N target breakForce. Cases are fresh stage copies;
no unrelated parameter search, rebuild of articulations or branch edits to main.

Factors:
- D: support density20000 ->2000 kg/m³, scaling support masses and inertias by0.1.
- G: corrected input-size fruit spheres, unchanged fruit mass, coherent inertia
  and preserved world attachment; includes updated fruit center/attachment offset.
- K: rachis drive stiffness multiplied by2; damping and pedicel drives unchanged.
- A: unbreakable initial fruit attachments, arming6N once supports stay quiet.

The source main reference passes again. Four one-factor cases are followed by
progressive combinations and the latest setup with only D removed. G+K is also
checked to separate the startup gate's effect at the original support density.
Outcomes refer only to20-second headless screening, not manual acceptance or
long-duration stability. Failed tests stop on their first fatal anomaly, so they
do not reveal the entire later detachment cascade.

| Case / changes from main | Result | First break or termination (simulated s) |
|---|---|---:|
| baseline | Pass20s | 20.000 |
| density | Spontaneous JOINT_BREAK | 0.067 |
| geometry | Pass20s | 20.000 |
| stiffness | Spontaneous JOINT_BREAK | 0.150 |
| arming | Pass20s | 20.000 |
| density-geometry | Spontaneous JOINT_BREAK | 0.083 |
| density-geometry-stiffness | Spontaneous JOINT_BREAK | 0.050 |
| latest | Structural divergence; never armed | 5.517 |
| geometry-stiffness | Spontaneous JOINT_BREAK | 0.150 |
| latest-heavy-supports | Pass20s | 20.000 |

Reproducible preparation (choose a fresh run root):

```bash
UV_CACHE_DIR=/tmp/autotom-uv-cache uv run --no-sync python \
  src/experiments/detachable_fruit_v2/prepare_main_transition.py \
  --run-root artifacts/detachable_fruit_v2/full-truss-return-main-new \
  baseline density geometry stiffness arming geometry-stiffness \
  latest-heavy-supports density-geometry density-geometry-stiffness latest
```

Run prepared directories with `run_batch.py`, then summarize with
`summarize_main_transition.py --run-root ... --output ...`.
Each directory records exact modified properties, source/scene/code hashes,
loaded body properties, first break, arming time and per-step traces. Summary
adds support excursion timing based on measured poses, not reported velocities.
The0.5m excursion marker is diagnostic, not a new pass/fail gate.

GUI commands, only for screened cases:

```bash
# Main reference, original fruit dimensions and support mass
UV_CACHE_DIR=/tmp/autotom-uv-cache uv run --no-sync python \
  src/experiments/detachable_fruit_v2/run_gui_freeze.py \
  --full-truss --support-density 20000

# Latest setup except for the lighter support mass
UV_CACHE_DIR=/tmp/autotom-uv-cache uv run --no-sync python \
  src/experiments/detachable_fruit_v2/run_gui_freeze.py \
  --full-truss --coherent-fruit --support-density 20000 \
  --rachis-stiffness-scale 2 --fruit-break-force 6 --arm-after-settle
```

## Interpretation

1. The main reference reproduces a full20s pass. All cases have the same30 bodies,
   24 DOFs and loaded solver iteration settings; body count does not explain the
   differences within this comparison.
2. D alone causes a break at0.066667s. D+G also fails, and D+G+K fails. Thus support
   mass/inertia reduction is sufficient to trigger failure under these settings.
3. K alone and G+K both break at0.15s. Stiffness doubling is a second sufficient
   startup trigger; density is not the only factor.
4. G alone and A alone pass. G+K+A at original support density passes and arms at
   4.783333s. The initial gate mitigates the startup failure in that heavy-support
   context; it is not a general structural stabilization method.
5. The latest D+G+K+A case reproduces the previous failure at5.516667s, with no
   broken joints and no arming. Compared with G+K+A, changing only support density
   converts the passing case into a divergence. Compared with D+G+K, adding the
   gate replaces early detachment with later gross structural divergence. This
   delayed failure is not an improvement in stability.

This isolates sufficient triggers and context-dependent mitigation, not a complete
four-factor decomposition or a proven microscopic PhysX cause. Support density
changes both mass and inertia; these were not varied independently. We have not
proven whether mass ratios, drive/solver response or another dynamic interaction
is the underlying mechanism.20s passing runs are screening evidence only. No GUI
acceptance or long-run robustness is claimed for these newly compared full cases.
Original main support density20000 kg/m³ remains an artificial numerical reference,
not an endorsed biological tissue density. No default configuration was changed.

All ten preparation/static audits and native body-property/DOF checks completed;
physics outcomes are retained as failures where appropriate. Source artifacts and
large traces remain local; compact results and reproducible tooling are tracked.
