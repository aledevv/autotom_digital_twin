# Main reference comparison — 2026-09-10

The user rejected the bounded manual drag after a complete 60-second run:
grabbing worked but felt abrupt, the native pick marker/arrow was missing,
and no tomato detached. They observed no instability and suggested comparing
the working truss in `main`. The minimum requirement remains unmet.

The corrected GUI run (`bounded-gui-review-fixed`) measured 33.16 FPS overall
and after settling, at 1280×720. Whole-cycle frame times p50/p95/p99 were
29.09/34.39/39.05 ms; real-time factor was 0.553 (60 simulated seconds in
108.57 wall seconds). Three grabs on two trusses peaked at 2.324 N; no native
break occurred. This is performance evidence, not detachment acceptance.

## Read-only comparison

Both local and remote `main` resolve to
`60246d57bc41785d828e6a071448aa130ab9785b`. The archived reference is
`src/data/usd_models/tree_v2_day_160.usda` from that commit. It has not yet
been rerun or regenerated for this comparison. Raw USD, hash and the extracted
properties remain local in `artifacts/detachable_fruit_v2/2026-09-10/main-comparison/`.

| Authored property | Main archived day 160 | Current candidate |
|---|---:|---:|
| Rigid bodies | 265 | 288 |
| Physical tomatoes | 40 | 72 |
| Tomato mass range | 6.35–10.63 g | 6.35–10.63 g |
| Immediate pedicel body mass | 7.540 g | 1.018 g |
| Fruit/pedicel mass ratio range | 0.84–1.41 | 6.24–10.45 |
| Tomato joint | FixedJoint, external | FixedJoint, external |
| Break force | 6 N | 6 N |
| Trunk internal joints | 10 fixed | 10 fixed |
| Solver / GPU dynamics | TGS / enabled | PGS / disabled |
| Authored physics rate | 480 Hz | 60 Hz |
| Articulation iterations | 32/4 | 32/0 |

The two attachment frames coincide in both archived rest poses (maximum
translation residual below 2 nm). Main's fruit joint already uses
`excludeFromArticulation`; that flag does not distinguish the models.

Main's source config uses truss density 20,000 kg/m³ and damping ratio 7,
against 2,000 kg/m³ and ratio 4 in the current exported source. Young's modulus
and the pedicel drive scale also differ. These are several simultaneous
differences, not proof that any one is causal. Do not copy main's inflated
density into the candidate: the agreed requirement preserves its masses.

Main's regular launcher uses native joint dragging (`forceGrab=False`), not
the native **force** mode used in the previous 20/60 cm replay comparisons.
Its native picker also draws the marker/arrow missing from the experimental
direct-force controller. A native-joint replay is therefore a useful control
before replacing more of the interaction architecture.

The old launcher constructs `World()` and resets without reapplying the
authored rate; 480 Hz in its USD is not a measured runtime rate. Measure the
effective configuration before describing a runtime comparison as equivalent.

There is also a separate `src/experiments/test_stable_truss.py` in main, with
two fruits and different overrides (including a 200 N break threshold). It
must not be conflated with the regular whole-plant 6 N configuration. The
user has been asked which reference they remember.

## Additional native-joint controls

Two 60-second headless runs used native joint dragging, coefficient 10,
30 seconds settling, a 60 cm downward ray-target trajectory over 5 seconds,
then release and observation. Both used CPU PGS60, articulation32/0 and
fruit255/0; masses, geometry and joint drives were retained from each source.
The main copy only received diagnostic body labels and the explicit solver
normalization. Its first loader attempt failed before physics because those
labels were missing; the corrected case is `main-native-joint-normalized-v2`.

| Scene / selected fruit | Native breaks | Tail attachment position error | Tail angle error |
|---|---:|---:|---:|
| Current / r5 g421786 fruit08 | 0 | 1.065 mm | 5.22° |
| Main archive / r8 lateral pair1 right | 0 | 0.0071 mm | 0.0116° |

Both completed without nonfinite states or persistent gross divergence but
failed the required detachment. The normalized main structure converges much
more closely. These controls differ in geometry, mass distribution, drives,
fruit count and selected attachment, so they do not isolate one causal factor.
They also do not reproduce main's original TGS/GPU runtime. Reproducing the
user's known successful launcher/configuration remains the next reference
check; the broader task is still open and final manual acceptance is absent.
