# Native main/v2.3 comparison

The custom controller is preserved at `e1908c6`. The user reports that holding
roughly 2 N on a fruit carried by a lateral branch can progressively drag its
support and produce rotation/instability in both branch and truss. This is an
observed custom-controller defect; load distribution is a hypothesis, not a
measured explanation. Native input is investigated separately.

## Rebuilt direct-truss pair

Local code snapshots: main `60246d57bc41785d828e6a071448aa130ab9785b`, v2.3
`e1908c6`. The original day160 CSV and PlantState inputs are preserved in their
respective snapshots. `build_native_reference.py` runs each original adapter,
filters its branch definitions before USD construction, then uses its original
builder. No leaves are authored or aggregated. V2.3 retains its historical truss
visual and initial overlap-filter authoring. Main uses its normal resolution cap.
This compares the two native constructions and inputs, not identical geometry.

The first common direct rank is 6, with eight fruits in each method:
main `Truss_r6_o0_rachis`, v2.3 `Truss_r6_o0_g421531_rachis`.
The full ten-link stem and original root/attachment joints remain. Main attaches
to stem link7, v2.3 to link6. No joints are reanchored. Counts are 30 versus33:
main has four rachis bodies, v2.3 seven. Total fruit mass is identical, 74.709 g.
The eight pedicels total60.319 g in main versus8.143 g in v2.3; rachis totals
40.212 g versus9.500 g. Source geometry, drive coefficients and masses remain
method-specific; runtime overrides only set backend, solver iterations, frequency
and the common break threshold. Damping multiplier is identity for both methods
(the reference main source ratio remains7, v2.3 remains4).

## Initial checks and threshold correction

The original agreed 2.5 N comparison failed at its first physical step in both
methods: two spontaneous main breaks, one v2.3 break. The user clarified that
main's6 N or higher threshold was intentional to prevent spontaneous detachment.
Native comparisons therefore resume at6 N; the saved custom2.5 N variant stays
unchanged. Reports for the failed2.5 N cases remain local. The first main attempt
failed before simulation because legacy USD lacked entity-kind metadata; adding
only diagnostic labels corrected that harness problem.

With TGS/GPU60 Hz, articulation32/4, fruits32/1, and6 N:
- Main completed20 s, exit0, no spontaneous break, functional gate passed.
- V2.3 spontaneously broke fruit07 at approximately0.1167 s. The monitor stopped
  at that failure; this is not a passed20-second test.

Thus the complete plant is not required for this v2.3 failure. The causal builder
property remains unidentified. No stiffness/mass tuning or new parameter search
has been performed. Manual main review is next; v2.3 must be presented as an
already-failed diagnostic case before any GUI demonstration.

`prepare_native_comparison.py` prepares fresh cases with `--runtime main|candidate`,
`--break-force` (default6), and scenario `stem-truss`. It validates retained body
count, zero aggregated leaf mass, original anchors and joint frames. The ordinary
`truss` ablation still keeps its original reanchoring behavior. The native GUI
observer forwards original viewport commands exactly once and records rays/hits;
it applies no force and adds no custom graphics. Native force remains unknown.
Both GUI cases use the same camera and1280x720 rendering; user zoom remains free.

Local evidence: `artifacts/detachable_fruit_v2/2026-09-10/native-comparison/`, with
snapshot commit provenance, input hashes, build definitions, USD/audits, executed
code hashes, simulator versions, and `construction-comparison.json`.
The initial monitor/interaction run passed30 tests. After adding the retained-stem
case and2 native-observer tests, the monitor+observer run passed20 tests.

## First manual main review

`tgs-6n-gui`: the user reports "Sì, il comportamento nativo è buono".
Two selected fruits physically broke with continuous motion. The GUI session
ended at18.9833 simulated seconds (20.0273 wall seconds), so the report correctly
fails duration completion. Measured mean FPS56.8965, steady57.5378, steady p05
53.5399, real-time factor0.94787. This supports the gesture comparison, not a
completed60-second acceptance test. The native observer logged four starts and
four releases, without logging errors; one query hit a support, one missed.

The second GUI is explicitly diagnostic: `--diagnostic-gui` allows observation
after spontaneous breaks, but retains them as report errors and never converts
the run into a pass. Nonfinite states and gross divergence still stop the run.
No physical parameter changes accompany this monitor option.

## V2.3 manual diagnostic: spontaneous release followed by rachis collapse

The user explicitly corrects the observation sequence: all fruit detached on
their own, then the rachis started oscillating violently until it crumpled and
the simulation closed. This was not a mouse-induced instability or merely a
fruit detachment problem. The native observer recorded zero grabs. All eight
physical JOINT_BREAK events occurred between0.1167 and0.2500 s. The monitor
subsequently detected a nonfinite state on pedicel01 at2.9333 s and stopped.
The report records a numerical simulation failure; it does not establish a
separate Isaac process crash. There was no opportunity for a manual grip test.

The v2.3 reduced construction fails without the complete plant or any mouse
input, while main's reduced construction stays usable with the same TGS/GPU
runtime and6 N threshold. This localizes a reproducible defect to the reduced
v2.3 stem/truss system. The next investigation should compare rachis/pedicel
construction, joint/drive properties and physical parameters with main; it
must not assume that the rest of the plant or the custom mouse is required.
The causal property and whether the break cascade causes or follows the
underlying solver instability remain unproven. No further runtime matrix,
lateral scene, stiffness tuning or custom-controller changes have been made
following this first-pair result; discuss this evidence before proceeding.
