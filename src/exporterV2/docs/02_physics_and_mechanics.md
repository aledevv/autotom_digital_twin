# Physics and Mechanics

Exporter V2 authors PhysX-ready articulated plants for Isaac Sim. Stems, laterals, leaves, and trusses are built as rigid links connected by joints whose drives are derived from beam-style stiffness and damping approximations.

## Runtime Defaults

Current V2 runtime defaults are in `PhysicsRuntimeConfig`:

- `PHYSICS_HZ = 480`
- articulation solver iterations `32 / 4`
- terminal tomato rigid-body solver iterations `32 / 1`
- GPU dynamics enabled
- CCD and broadphase settings remain part of the V2 PhysX setup

These defaults are a performance/stability compromise after detachable-truss tuning. Experiments that declare their own solver settings keep their local values.

## Branch Mechanics

Regular flexible links use D6 joints with angular limits and spring-damper drives. Stiffness is computed from radius, length, Young's modulus, and second moment of area; damping is computed from damping ratio and rotational inertia. The main trunk can be fixed through `PhysicsRuntimeConfig.RIGID_TRUNK`.

## Truss Mechanics

Trusses use a dedicated `TrussPhysicsConfig` because rachis and pedicels are much thinner than stems:

- truss Young's modulus: `80.0e8 Pa`
- truss damping ratio: `5.0`
- truss density: `20000 kg/m^3`
- pedicel bend limit: `25 deg`
- pedicel drive stiffness scale: `50.0`

The inflated truss density is deliberate: it improves the solver mass ratio between pedicels and tomatoes without changing visual geometry.

## Tomato Detachment

Tomato detachment is enabled through breakable FixedJoints:

- tomatoes live under `/World/TerminalBodies`
- each tomato is excluded from the main articulation
- each tomato receives `PhysxRigidBodyAPI`
- each tomato FixedJoint has `breakForce = 6.0 N`
- tomato solver iterations are authored from terminal-body runtime config

This keeps the truss articulation stable while allowing tomatoes to detach during interaction. The break force is currently a diagnostic/tuning value, high enough to avoid immediate gravity-only drop in the current generated scenes.

## Collision Filtering

The stage applies collision filters for immediate parent-child pairs and for dense organ attachments. Detachable tomatoes additionally filter collisions against their pedicel and related truss rachis so the breakable joint is not destabilized by initial overlap or near-contact.
