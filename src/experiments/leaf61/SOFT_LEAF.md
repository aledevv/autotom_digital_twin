# Illustrative compliant leaf — 2026-09-18

The user prioritizes leaf-like contact behavior over FPS. This is a separate `--leaf-profile soft`; the original three-link profile remains unchanged. This is an illustrative articulated approximation, not a biomechanically calibrated sheet.

- Seven longitudinal segments, bending and torsion at every joint. Revision 3 retains revision 2's correction: it derives the bending plane from the area-weighted native mesh normal and longitudinal axis; the cylindrical petiolule roll is not treated as the leaf plane.
- Bending stiffness tapers geometrically from 0.003 to 0.0004 Nm/rad (original: 0.024 throughout). Torsion is 35% of bending stiffness. Damping ratio 1.2; angular limits ±55 degrees.
- Collision hulls follow clipped native **3-D curved** mesh strips, thickened by 0.5 mm; they are not flat plates. Convexification remains an approximation.
- Original visible contour, initial curvature, native leaf mass and plant mass are preserved. Revision 3 anchors the lamina at the actual petiole endpoint (fixed_length=0), replacing the broad fixed band inherited from the rectangular fixture. The basal visible points follow the first segment; the true base remains on the joint pivot. This removes the artificial fixed-to-rotating skin transition across broad basal wings. Segment masses are proportional to clipped surface area, normalized to the original leaf biomass. Initial point-lumped mass is removed from the native host as in the previous integration.
- CPU/PGS at 480 Hz; skin and rendering at 30 Hz. No FPS target for this illustration.

## Controlled branch-shaped contact

```bash
./run_leaf61_soft_contact.sh
```

A 3 mm radius, 75 mm long kinematic capsule approaches the first leaf from underneath at 65% of free length. It starts 20 mm clear of the pre-contact surface, approaches over 1.5 s, holds 0.5 s and withdraws over 2 s. There are eight seconds of recovery by t=20 s. Only this leaf collides with the probe; other original plant bodies are filtered from the probe. The native host remains dynamic, allowing measurement of how much motion reaches the support.

GUI starts with a close camera. **Repeat branch contact** restarts the stroke after two seconds; **Focus selected leaf** reframes the leaf; native **Shift + left-drag** remains available. The first 20 seconds of poses are recorded by default, so later manual repeats are visual exploration rather than fully recorded trials.

Headless runs: `original-leaf-contact-control` and `soft-leaf-base-hinge-contact` (revision 3), earlier comparison `artifacts/leaf61/soft-leaf-final-comparison.json`.

| Measure | Original 3-link | Soft 7-link |
|---|---:|---:|
| Maximum lamina movement relative to petiole | 16.331 mm | 26.629 mm |
| Maximum actual blade-base movement during contact | 0.533 mm | 0.01550 mm |
| Accumulated contact impulse magnitude | 0.05776 Ns | 0.001311 Ns |
| Maximum edge extension | 1.283% | 1.188% |
| Recovery error | 0.265 mm | 0.0105 mm |
| Maximum post-settling collider penetration | 0.957 mm | 0.00146 mm |

Both contact trials remain finite and pass the existing numeric checks. The soft profile transmits about 34x less support motion and yields locally. The comparison changes segment count, collider geometry, stiffness, torsion and mass distribution together; it is a profile comparison, not an isolated stiffness experiment. Each probe stroke is positioned relative to its model's own pre-contact equilibrium.

The first attempt `soft-leaf-contact-v1` built/settled but stopped at contact start due to a pose callback argument mismatch. The corrected v2 completed 20 simulated seconds. Tests check 3-D strip area partition without flattening and preservation of native curved bind geometry; all 34 leaf61 tests pass.

## Full-plant contact boundary

All 131 soft blades add 917 bodies (1098 total including the original 181). The raw all-contact initial trial `full-plant-soft-initial` stayed finite with native-body displacement 53.2 mm, but was **rejected as an initial-contact setup**: initial overlap reached 12.34 mm and visual edge extension reached 290%. There are 102 deeply overlapping first-step pairs. This is why old three-segment exclusion evidence must not be reused: the collider geometry/topology changed. The launcher checks the leaf profile as well as source SHA and leaf count when importing overlap exclusions.

This model can bend along its length and twist; each transverse strip is still rigid. It does not simulate independent deformation across the full blade width, venation, wrinkling or a continuous membrane. GUI appearance remains for the user to judge.


The intermediate seven-link model retained the petiolule transverse axes. On a complete plant, that exposed an authoring assumption: arbitrary cylindrical petiole roll does not specify the leaf plane. `full-plant-soft-initial` and `full-plant-soft-filtered` are therefore retained as failed revision-1 trials; the latter still reached 270% extension despite 102 pair exclusions. Revision 2 aligns the bend axis to the actual sheet and is recorded in runtime metadata. Imported exclusion evidence must match the revision, preventing use of stale collision pairs.

For the current controlled revision-2 run: 20 simulated seconds completed; root error 0.000035 mm, recovery 0.027 mm, residual motion 0.00121 mm; all existing contact/geometry checks passed. This is a controlled capsule/leaf demonstration, not an acceptance of full-plant self-contact.


## Final revision 3 and launchers

`soft-leaf-base-hinge-contact` completed 20 s: finite data, root error 0.000033 mm, edge extension 1.188%, recovery 0.0105 mm, residual 0.00056 mm, contact penetration 0.00146 mm. The fresh final comparison uses the actual native blade-base point in both models, so it does not confuse the old fixed-band marker with the new pivot. Thirty-five leaf61 tests pass; one explicitly rotates the whole lamina and verifies that its base stays exactly on the hinge without a skin discontinuity.

```bash
# Close-up, repeatable branch-shaped contact with one soft lamina:
./run_leaf61_soft_contact.sh
# Full native plant, soft leaves against native plant colliders:
./run_leaf61_soft_plant.sh
```

The full soft launcher uses **plant** contact mode (leaf/native bodies, including fruits; no leaf/leaf contacts). This deliberately focuses the new illustration on the requested leaf/branch behavior. It uses only revision-3 overlap evidence (`full-plant-soft-base-hinge-initial`), omitting its 82 initial overlapping pairs. Original full-plant/stress launchers remain unchanged. This full native layout is still a stress illustration, not a validated sheet model or collision layout.

Final full-plant revision-3 preflight `full-plant-soft-base-hinge-filtered` completed 3 simulated seconds with 82 explicit startup exclusions and plant-only contacts. It stayed finite and attached (root error 0.000067 mm, native-body motion 54.44 mm), observed 65 impulsive leaf/plant pairs, and reduced residual first-step penetration to 0.0356 mm. **It still fails the stretch criterion: maximum edge extension 78.56%**, so it is not promoted as a validated whole-plant model. There was no post-8s recovery or performance window in this short preflight. It is retained solely for the user's requested illustration. Whole-plant GUI: `artifacts/leaf61/full-plant-soft-gui`, automatically focused near leaf 0; manual visual acceptance pending.
