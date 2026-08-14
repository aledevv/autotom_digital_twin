# Implementation Notes

## Refactor Decisions

- The CSV pipeline reads the input graph once per full parse and passes a shared DataFrame to the loaders.
- Loader APIs remain standalone-compatible through an internal keyword-only `_dataframe` parameter.
- Leaf construction uses shared helper logic for trunk and lateral leaves while preserving order and warnings.
- `tree_config` loading in leaf and truss builders is cached and still supports package imports and standalone execution.
- Stage construction reuses one `branch_id -> branch` map for parent lookup, terminal bodies, and collision filtering.
- Terminal tomato body authoring is isolated in a private helper so detachment logic stays auditable.

These notes consolidate the useful information that used to be scattered across
intermediate restructuring and refactoring reports. The old reports described
the transition from a monolithic exporter to the current `core/adapters/profiles`
layout, the preservation of standalone loader compatibility, and the split
between production tests and visual demos. Those decisions are now part of the
canonical V2 documentation rather than separate status summaries.

## Truss and Detachment Notes

The stable truss path keeps V2 structural compatibility: pedicels remain one-link CSV-derived branches, and tomatoes are detached as terminal rigid bodies only at USD authoring time. The test USDA from early experiments is obsolete; current behavior is defined by Python tests and generated V2 output.

Tomato detachment is intentionally authored late in stage construction. The
branch JSON still represents the biological hierarchy, while the USD stage moves
detachable tomato rigid bodies under `/World/TerminalBodies`, creates breakable
FixedJoints to the pedicel terminal link, marks the tomato as
`excludeFromArticulation`, and filters collisions against the related pedicel
and rachis links.

The recursive-tree experiments also produced one important transform lesson:
sub-branch placement must inherit the full parent orientation, not only the
parent attachment position. That lesson remains relevant when comparing V1
visual baselines, recursive experiments, and V2 branch authoring.

## Optimizer Notes

The active optimizer is a joint-budget system rather than a visual simplifier.
It counts physical joints, applies selected techniques under an explicit budget,
and leaves disabled controls disabled unless a test or experiment opts into
them. Current maintained techniques include petiole lock, lateral branch
reduction, leaf branch reduction, stem collapse, and truss static handling.

Visual optimizer comparisons and before/after assets have moved to
`src/exporterV2/demos/optimization_visual_validation/`; the task-by-task
implementation summaries are no longer the source of truth.

## Experimental Mechanics Notes

The cantilever and three-point experiments are useful calibration references,
but their old planning summaries should not be treated as current V2 behavior.
The stable lessons are:

- D6 angular drives should be interpreted through beam stiffness `EI` and the
  local link/control length, not as arbitrary per-joint constants.
- When only discretization changes, total branch length, mass, radius, density,
  boundary conditions, force, and solver settings must stay fixed.
- Three-point bending uses the classical small-deflection relation
  `delta = F L^3 / (48 E I)` with simple supports and load at midspan.
- The span-to-diameter ratio should remain high enough for Euler-Bernoulli
  assumptions; use only the initial linear force-deflection region for fitting.
- Solver convergence, timestep, damping, and mass ratios must be recorded when
  comparing simulated stiffness to analytical or physical measurements.

For detachable tomatoes, small motion at the FixedJoint can be a solver artifact
rather than biological behavior. The current checklist is: verify the two joint
frames coincide in world space, filter local tomato-pedicel-rachis collisions,
author tomato solver position iterations explicitly, and keep the tomato to
pedicel-tip mass ratio within a solver-friendly range.

## Performance Notes

Current runtime is `480 Hz` with articulation iterations `32 / 4` and tomato-body iterations `32 / 1`. This is smoother than lower-rate tests but can cost frame time in Isaac Sim. Truss density and damping are intentionally high for stability.

## Cleanup Notes

Legacy crash repros, single-function root tests, duplicate archived repo files, and Python caches are no longer tracked. Visual validation was moved to `src/exporterV2/demos/` so ordinary pytest runs do not collect manual Isaac scripts.
