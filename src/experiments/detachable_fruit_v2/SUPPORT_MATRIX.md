# Rachis constraints and diagnostic density: paired support matrix

Starting checkpoint `eb1ea90`, experimental branch. User-approved diagnostic
mass changes only; main/groPy/custom exporter settings remain unchanged.
Local evidence: `artifacts/detachable_fruit_v2/2026-09-11/support-matrix/`.

## Corrected starting facts

Both source stems have ten fixed links: RootFixedJoint plus nine internal
PhysicsFixedJoints. They contribute zero DOFs. The prior suggestion to lock
stem flexibility was redundant and its interpretation was incorrect.

Main's original TrussPhysicsConfig explicitly uses inflated support density
20000 kg/m^3; v2.3 uses2000. The source comment's asserted solver mass-ratio
threshold is an old tuning rationale, not a verified universal PhysX limit.
The loaded support masses confirm the different densities. Other construction
and drive differences remain, so main is a positive behavioral reference,
not automatically a physically correct target.

## Independent controls

Source: the existing complete-stem/complete-rachis/one-pedicel/fruit07 scenes.
All16 factorial cases keep this geometry, native mouse and 6 N break force,
TGS/GPU60 Hz, articulation32/4 and fruit32/1. No drive retuning.

- `--lock-entry`: lock rotX/rotY of the rachis incoming joint; rotZ already locked.
- `--lock-internal`: lock rotX/rotY of every internal rachis joint.
- `--density 2000|20000`: scale rachis and pedicel mass and inertia by the density
  ratio; COM, principal frame, geometry, fruit mass and every drive gain unchanged.
- `--without-fruit`: extra control at each construction's original density;
  remove only the fruit and its joint, retaining its pedicel and all support load.

`prepare_support_matrix.py` writes the full old/new property manifest plus
expected masses/inertias and articulation DOFs. Cooked original support inertias
are scaled in their recorded COM frames and authored before simulation. No
runtime reconstruction or mass adjustment while stepping is used.
`fruit_diagnostics.py` checks loaded body properties to relative tolerance1e-5
and articulation DOFs exactly. Main DOFs:10/8/4/2 for no/entry/internal/both locks;
v2.3:16/14/4/2. The remaining2 are the pedicel bending DOFs.

Each screening runs20 simulated seconds from rest, stopping on the existing
fatal conditions. Functional passes retain all stricter numerical advisories;
measured physics velocities and finite differences of poses remain separate.
Full support traces, first failure and all break events are retained locally.
The summary does not promote a headless pass to GUI acceptance or measured FPS.

Execution order: original references, no-fruit controls, reciprocal density
controls, locks at original density, locks at swapped density. Sequential Isaac
processes. Availability for an early60-second native GUI was requested before
results; GUI takes priority as soon as the user is available.

## Results

Results are generated in `support_matrix_results.json`. Tests and final
interpretation are recorded after the bounded matrix finishes.

## Remake feasibility (proposal only)

The user accepts a stiffer truss. An isolated replacement can aggregate rachis
and pedicels into one compound rigid body while keeping all eight fruit bodies
external with real breakable FixedJoints. Keep the original support mass, compute
its combined COM and inertia using rotated component tensors and the parallel-axis
theorem, and re-express fruit joint anchors in the aggregate frame. Keep native
mouse input. Never rebuild the articulation at break time.

For the complete rank6 fixture (not the full plant):

| Construction | Current total bodies | Proposed total bodies | Current DOFs | Proposed DOFs, mobile entry |
|---|---:|---:|---:|---:|
| main |30 |19 |24 |2 |
| v2.3 |33 |19 |30 |2 |

For v2.3, all15 original support bodies sum to0.0176431843 kg; the candidate
would preserve that mass, not copy main's0.1005309634 kg support mass. The combined
inertia eigenvalues computed in the original rest pose are positive:
1.61432e-6,4.40904e-5,4.45019e-5 kg m^2. Full calculations are in local
`remake-feasibility.json`. A fixed entry alternative has zero support DOFs but
loses overall truss swing as well as internal rachis/pedicel bending.

Collision preservation requires special care: current fruit filters exclude its
own pedicel and the rachis, not necessarily every other pedicel. Mapping every
old body filter to the aggregate would suppress additional contacts. A prototype
must preserve the intended external collider-pair filtering and document that
collisions between constituents of the same rigid body no longer exist.
Visual hierarchy and any skinning bindings need explicit remapping, without
nested active rigid bodies or altered collider world poses.

This is feasible as a reduced mechanical representation, not verified stable
or faster. Collider/render cost remains even when body/DOF count falls. The
prototype would require native grasp/release, real JOINT_BREAK, one-fruit then
eight-fruit screening and manual checks before full-plant integration. Isaac4.5
[documented drive limitations](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/physics/physics_resources.html)
remain relevant to any mobile-entry design. No replacement builder is implemented
before discussing the matrix results and this approximation with the user.
