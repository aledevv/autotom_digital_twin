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

All18 headless runs completed or stopped at their first spontaneous break.
Every runtime body-property and DOF check passed.30 focused tests passed.
`support_matrix_results.json` contains the full compact matrix and evidence hashes.

Main passes20 s in all eight combinations, including density2000 with all
original drives and mobile joints. Both no-fruit controls pass20 s.

| v2.3 density | Rachis entry | Rachis internal joints | Result |
|---|---|---|---|
|2000 | Mobile | Mobile | Fruit07 breaks at4.9000 s |
|2000 | Locked | Mobile | Fruit07 breaks at7.8667 s |
|2000 | Mobile | Locked |20 s, no break |
|2000 | Locked | Locked |20 s, no break |
|20000 | Mobile | Mobile |20 s, no break |
|20000 | Locked | Mobile |20 s, no break |
|20000 | Mobile | Locked |20 s, no break |
|20000 | Locked | Locked |20 s, no break |

These are functional screening outcomes, not strict numerical or GUI acceptance.
At original density with only internal joints locked, v2.3 tail position excursion
is0.2102 mm, pose-derived maximum speed0.0311 mm/s and angular speed0.001176 rad/s.
The fruit attachment angle still reaches about1.26 degrees, and reported PhysX
velocities exceed the strict thresholds despite much smaller pose motion.
Increasing density alone instead leaves1.6397 mm tail excursion and pose-derived
speed17.34 mm/s: it avoids early break but is not cleanly settled.

Offline frame-gap analysis of all support joints (from the per-step physics
poses) found only floating-point-scale separations, including before both failed
runs. The dynamic failure is not accompanied by a demonstrated support-anchor
position mismatch. Mobile-joint relative angles include intended bending and
are not themselves counted as joint errors.

Interpretation: loaded internal rachis mobility contributes to this v2.3 failure;
entry mobility alone does not explain it. Increased support mass/inertia can mask
the failure in this construction, but is not necessary for the main one-fruit
fixture to survive20 s. Geometry, inertia distribution, segmentation and drive
differences can still interact. This neither proves one broken builder formula
nor establishes that a mass ratio or joint count alone is the cause.

The preferred manual diagnostic is `v23-internal-locked-gui`: original mass,
mobile entry and pedicel, internal rachis locked. A second diagnostic,
`v23-d20000-gui`, retains all mobile joints with inflated support mass. Both are
prepared for60 s with native input. Availability was requested early; no reply
has arrived during this matrix, so no GUI was launched and no FPS or acceptance
has been claimed. The installed SimulationApp default rendering resolution is
1280x720; the monitor records actual GUI settings/resolution on launch.

Launch the already prepared preferred case once (the runner refuses to overwrite
existing reports):

```bash
cd /home/alessandro/isaacsim/autotom_digital_twin
/home/alessandro/isaacsim/python.sh src/exporterV2/isaac_app.py \
  --usd artifacts/detachable_fruit_v2/2026-09-11/support-matrix/v23-internal-locked-gui/scene.usda \
  --physics-preset flexible --interactive-physics-hz 60 --duration 60 \
  --fruit-experiment artifacts/detachable_fruit_v2/2026-09-11/support-matrix/v23-internal-locked-gui/config.json
```

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

Recommendation for discussion: first manually validate the internal-lock control
at original mass. A limited builder change could retain separate links but expose
internal locking separately from the entry joint: the current branch `joint_type`
fixed setting locks both and would not reproduce the preferred control. The
compound-body remake removes more bodies/joints but also removes pedicel flexion
and requires explicit collision/skinning remapping. Neither implementation has
been substituted into the exporter during this diagnostic phase.

## Requested manual main review: startup blocked before scene load

The user emphasizes preserving useful truss mobility and requests main at the
reduced density. Prepared `main-d2000-native-gui`: the same one-fruit fixture
that passed20 s, density2000, entry/internal joints mobile, native joint grab,
6 N break force and60 s requested duration. No production model was changed.

The GUI launch failed on2026-09-11 around11:25 UTC during SimulationApp startup,
before loading USD or stepping plant physics. Log:
`artifacts/detachable_fruit_v2/2026-09-11/support-matrix/main-d2000-native-gui/gui.log`.
First error: CUDA error3 / initialization failure, followed by GPU device creation
failure, X BadMatch and segmentation fault. `nvidia-smi` still lists the RTX4080
and driver575.57.08. An independent `/usr/bin/python3` call to
`libcuda.so.1:cuInit(0)` also returns3 / `CUDA_ERROR_NOT_INITIALIZED`. This is not
an observed truss failure, and no manual feedback or FPS was collected.

The prepared scene remains available. After recovering CUDA, launch:

```bash
cd /home/alessandro/isaacsim/autotom_digital_twin
/home/alessandro/isaacsim/python.sh src/exporterV2/isaac_app.py \
  --usd artifacts/detachable_fruit_v2/2026-09-11/support-matrix/main-d2000-native-gui/scene.usda \
  --physics-preset flexible --interactive-physics-hz 60 --duration 60 \
  --fruit-experiment artifacts/detachable_fruit_v2/2026-09-11/support-matrix/main-d2000-native-gui/config.json
```
