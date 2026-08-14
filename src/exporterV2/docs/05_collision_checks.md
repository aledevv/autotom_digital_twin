# Collision Checks

V2 uses two collision-control layers: geometric checks while parsing/generating branch layout, and USD collision filters while authoring PhysX bodies.

## Geometric Checks

The CSV adapter validates lateral branch spacing to avoid obvious same-rank and adjacent-rank overlaps before simulation. The tomato profile keeps a conservative minimum angular separation and retries jittered orientations when branches are too close.

## USD Filtered Pairs

During USD generation V2 authors `FilteredPairs` relationships for dense articulated structures:

- parent-child link pairs
- branch attachments that start close to the parent surface
- pedicel/truss connections
- detachable tomato to pedicel
- detachable tomato to truss rachis

The detachable tomato filters are important because the tomato is outside the articulation root. Without them, contact resolution near the breakable FixedJoint can dominate the intended detachment behavior.

## Validation

Pure geometry checks can run in the regular Python environment. PhysX schema checks and runtime inspection require Isaac Sim or an environment with `pxr`/`PhysxSchema` available.
