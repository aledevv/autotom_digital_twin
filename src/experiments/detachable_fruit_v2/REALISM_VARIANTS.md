# Experimental realism variants

All changes live in this experiment directory. Exporter core, native mouse,
fruit radii, masses, colliders and detachment thresholds remain unchanged.
The reference is the verified full day160 fixture, with direct trusses only.

The user's photos guide silhouette, taper and calyx appearance. They do not
provide dimensions or measured stiffness. GroIMP fruit radii are preserved at
the user's explicit request. Calyces/pedicels received positive visual feedback;
the preceding slender-only GUI also received positive stability feedback.

Preparation sequence (project Python, each destination must be fresh):

```bash
uv run --no-sync python src/experiments/detachable_fruit_v2/slender_truss_visuals.py SOURCE SLENDER
uv run --no-sync python src/experiments/detachable_fruit_v2/natural_truss_visuals.py SLENDER CALYX
uv run --no-sync python src/experiments/detachable_fruit_v2/prepare_realism_candidate.py CALYX CANDIDATE
uv run --no-sync python src/experiments/detachable_fruit_v2/run_prepared_gui.py CANDIDATE --real-time
```

`SOURCE` is the existing full-rest prepared fixture, not an arbitrary plant USD.
Scripts intentionally check the expected 5 trusses/40 fruits and 55 leaf-support
bodies. They are reproducible day160 experiments, not a generic exporter feature.

- Calyces: five tapered sepals;216 vertices/fruit; no collision or rigid body.
- Pedicels: curved centerlines unchanged; visual diameters2.4mm to1.92mm.
- Rachides: visual diameter3mm at base, tapering to1.95mm at tip;0.5mm gentle
  bow within each segment. Endpoints stay put; no additional physics links.
- Leaf support flexibility: only rotX/rotY drive stiffness on petiole and
  leaf_rachis bodies multiplied by0.8. Damping, limits and masses stay unchanged.
  These are experimental aesthetic/dynamic choices, not biological calibration.

For the stiffness comparison, use `--leaf-stiffness 1.0 --leaf-probe` and
`--leaf-stiffness 0.8 --leaf-probe` on two fresh destinations, then run
`run_batch.py --leaf-probe BASELINE SOFTER`. The separate headless wrapper applies
2mN downwards at one deterministic leaf rachis body's center of mass from30–35s,
with a1s ramp, and records its pose and commanded force every physics step.
The GUI does not load this wrapper and has no automated leaf forces.

## Measured leaf comparison

The0.8 stiffness candidate passed60 simulated seconds at rest without breaks or
nonfinite values. A paired0.05N probe (`--probe-force 0.05`) on the same selected
leaf rachis gave0.7367mm displacement for the baseline and0.9273mm for the softer
candidate, measured from the mean28–30s pose to the mean34–35s pose. Both recovered
to within0.001mm of their own pre-force poses in the last5s. This establishes a
roughly26% compliance increase on that body, not biological stiffness calibration.
The smaller2mN paired probe gave the same trend; all four physical reports passed.

Two initial preparation attempts assumed a cylindrical leaf collider, but the
vegetation uses capsule chains. They failed before simulation; selection now
uses the sum of capsule heights. Their partial directories are retained locally.
The first wrapper runs produced completed physical reports but exit1 on shutdown;
the wrapper now uses the same explicit process-exit mechanism as isaac_app.
The0.05N paired runs both exited0. Core code was not modified.

Do not interpret a resting pass alone as improved realism: compare displacement
under equal force, recovery after release and the user's GUI feedback.
