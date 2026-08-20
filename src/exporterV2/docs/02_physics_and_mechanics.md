# Physics And Mechanics

Exporter V2 represents plant axes as rigid links connected by Fixed, D6, or revolute joints. The current numerical values and feature switches live in `core/tree_config.py`; documentation intentionally does not copy them.

## Branch Mechanics

Branch-chain authoring is centralized in `core/usd/branch_chains.py`. Link mass is derived from geometry and configured density. Flexible drive stiffness follows beam-style flexural rigidity and attachment compliance; damping follows the configured damping ratio and inertia model.

The vegetative backend changes how stems and leaves are rendered, not how their rigid links and joints are simulated. Organic meshes, leaf blades, terminal fork dressing, and overlap tongues are visual-only.

## Trusses And Pedicels

Truss rachides and pedicels retain the legacy articulated-chain path in hybrid scenes. Their mechanics use the truss profile selected by branch metadata and `TrussPhysicsConfig`.

The visible gravity-curved pedicel is a child mesh of the straight physical proxy. Its tip is also used to place the visual tomato and fixed-joint frames consistently, but the proxy topology and collision shape remain unchanged.

## Tomato Detachment

Detachment is controlled by `TrussPhysicsConfig` and optional per-body overrides. When enabled, a tomato:

- is authored as a rigid body under the configured terminal-body parent;
- can be excluded from the main articulation;
- is connected to its pedicel by a FixedJoint with the configured break force;
- receives terminal-body solver settings from `PhysicsRuntimeConfig`.

The stage does not replace the configured break force with an artificial holding value. Detachment remains available with the values selected in `tree_config.py`.

## Collision Filtering

Normal parent-child and sibling filtering remains part of branch authoring. Terminal-body handling adds two narrowly scoped policies:

- detached tomatoes are filtered against their own pedicel and related rachis links;
- tomato-tomato pairs are filtered only when they overlap in the authored rest pose and the corresponding config switch is enabled.

Non-overlapping tomato pairs are not globally filtered, and the overlap filter does not disable detachment.
