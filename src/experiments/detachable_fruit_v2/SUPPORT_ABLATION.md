# Native support ablation after the failed factorial matrix

Experimental checkpoint: `5b11f5c`. Main/groPy/custom code is unchanged.
All scenes are private copies of the audited freshly rebuilt rank6 fixtures,
not physical changes to the exporter. Local artifacts:
`artifacts/detachable_fruit_v2/2026-09-11/support-ablation/`.

Purpose: identify which support structure is necessary for the spontaneous
failure. Keep original construction-specific body geometry, mass, COM, collider,
inertia, drive gains and the fruit FixedJoint. Common TGS/GPU60 Hz,
articulation32/4, fruit32/1,6 N break force, native joint mouse coefficient10.
No custom controller or graphics.

Selected corresponding fruit: number07, the baseline v2.3 first-break fruit.
Main name: `Truss_r6_o0_rachis_pedicel_lat_3_L_tomato`.
V2.3 name: `Truss_r6_o0_g421531_tomato_07`.
Both builder definitions give mass0.008154648458009497 kg and radius
0.012486447 m before the builders' common geometry scale2. Main pairs are
numbered by alternating L/R at nodes0..3; v2.3 uses numbers01..08. This mapping
is verified against the original `direct/build.json` definitions.

| Level | Retained structure | Changed constraints |
|---|---|---|
| rigid | Original anchored root link + pedicel07 + fruit07 | Pedicel incoming joint reanchored to root, preserving global frames; rotX/Y/Z locked |
| articulated | Same three bodies | Same reanchoring, original pedicel angular limits/drives active |
| rachis | Complete original stem + whole rachis + pedicel07 + fruit07 | Original joints/anchors unchanged; other seven pedicels/fruits removed |
| original | Complete stem + rachis + all eight pedicels/fruits | Existing reference, no ablation |

The locked case retains an articulation with fixed links. It does not make the
fruit kinematic or delete its breakable constraint. Reanchoring changes support
compliance intentionally; removed bodies no longer contribute load. No claim
that this isolates collision count alone. Retained collision filters remain,
references to removed objects are cleaned. `ablation.json` records every removed
body/property and every modified joint/filter property. Automated assertions
and tests verify retained body/collider properties and fruit joint preservation.

## Results

| Case | Headless duration | Result |
|---|---:|---|
| main rigid |20 s | Passed, no break |
| v2.3 rigid |20 s | Passed, no break |
| main articulated |20 s | Functional pass, no break; numerical advisories |
| v2.3 articulated |20 s | Functional pass, no break; numerical advisories |
| main rachis, one fruit |20 s | Functional pass, no break |
| v2.3 rachis, one fruit |4.900000 s | Failed: spontaneous fruit07 break |
| main original, eight fruits |20 s | Earlier reference passed |
| v2.3 original, eight fruits |.116667 s | Earlier reference failed: fruit07 break |

For both rigid cases, every retained body's runtime mass, inertia matrix and COM
are exactly equal to that body's values in the original unablated fixture.
Final10-second position excursion is zero for both. Fruit joint position error:
main4.42e-8 m, v2.3 1.10e-7 m. No manual acceptance or GUI FPS claim follows.

This establishes that the original external fruit joint plus a fixed pedicel
is not sufficient to reproduce the v2.3 failure at rest. It does not establish
that the joint is innocent when its support moves or under mouse interaction.

All six new runs preserve every retained body's runtime mass, inertia, COM and
COM orientation exactly against its original source. No body-property changes
explain the ablation results. The articulated three-body cases have almost
stationary tail poses (excursion main3.46e-6 m, v2.3 5.94e-7 m), despite larger
reported physics velocity peaks (main .02254 m/s/.66275 rad/s, v2.3
.05100 m/s/.33341 rad/s). Finite differences of poses are recorded separately
and much smaller. Both cases exceed the strict velocity diagnostic thresholds;
v2.3 also has about1.28 degrees fruit attachment error. Functional pass means
no spontaneous break or gross divergence, not perfect settling or GUI acceptance.

Reintroducing the original stem plus rachis reproduces v2.3 spontaneous failure
with just one fruit, while main completes20 s. The other seven pedicels/fruits
are not necessary for this failure, but their presence accelerates it in the
full rank6 fixture. Removing them changes load, contact opportunities and solver
coupling together; this does not isolate a mass-only cause. Likewise the return
of the stem/rachis structure does not by itself establish which constraint is at
fault. Correction after inspecting every joint: the stems are fixed chains in
both fixtures; the earlier attribution to returned stem compliance was incorrect.
The failed run stops at the first break and does not characterize a
subsequent collapse.

The approved follow-up locks the rachis entry joint, internal rachis joints,
or both, and compares diagnostic support densities2000/20000 kg/m^3.
Locking the stem would be redundant. The original main builder deliberately
inflates truss density to20000; v2.3 uses2000. These controls are not production
fixes. The user also accepts considering a stiffer, simpler remake.

Manual availability was requested as soon as the first paired headless checks
were underway. `v23-rigid-gui` is prepared with native input for60 s; no new GUI
session has been launched and no manual feedback or FPS has been measured in
this ablation phase yet. GUI launch awaits the user's availability response.

Validation:23 tests passed (support preservation, diagnostics and native observer).
`support_ablation_results.json` stores compact results and local evidence hashes.

## Commands

Use a fresh output directory. `--mode rigid|articulated|rachis` selects the level;
`--source-case` selects main or v23. Add `--gui --duration 60` for manual preparation.

```bash
UV_CACHE_DIR=/tmp/autotom-uv-cache uv run --no-sync python src/experiments/detachable_fruit_v2/prepare_support_ablation.py \
  --source-case artifacts/detachable_fruit_v2/2026-09-10/native-comparison/v23/tgs-6n-preflight \
  --run-dir artifacts/detachable_fruit_v2/2026-09-11/support-ablation/v23-rigid-gui \
  --mode rigid --gui --duration 60
/home/alessandro/isaacsim/python.sh src/exporterV2/isaac_app.py \
  --usd artifacts/detachable_fruit_v2/2026-09-11/support-ablation/v23-rigid-gui/scene.usda \
  --physics-preset flexible --interactive-physics-hz 60 --duration 60 \
  --fruit-experiment artifacts/detachable_fruit_v2/2026-09-11/support-ablation/v23-rigid-gui/config.json
```

The GUI preparation already exists locally; rerun only the Isaac launch command
for that case (use a fresh preparation directory to preserve prior GUI reports).
