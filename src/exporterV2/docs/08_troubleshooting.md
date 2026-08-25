# Troubleshooting

## Import Errors

Run ordinary tests from the project root so `src/` imports resolve:

```bash
uv run pytest src/exporterV2/adapters/groimp_csv/tests \
  src/exporterV2/core/optimizations/tests src/exporterV2/tests -v
```

Do not collect the entire `src/exporterV2` directory with ordinary Python:
`core/usd/tests` contains standalone Isaac-oriented tests with historical
top-level imports and has its own command below.

If a test needs `PhysxSchema`, run it with Isaac Sim Python:

```bash
~/isaacsim/python.sh -m pytest src/exporterV2/core/usd/tests -v
```

## Interactive FPS vs validation rate

ExporterV2 authors and validates physics at `480 Hz`, but the interactive GUI
defaults to `60 Hz`, matching the effective runtime of the legacy V2 loader.
The rates are intentionally separate:

```bash
# Fluid interactive inspection (default: 60 Hz)
./run_mainV2.sh --day 50 --interactive-physics-hz 60

# High-rate finite validation (always uses --physics-hz)
./run_mainV2.sh --day 50 --headless --duration 5 --physics-hz 480
```

Interactive alternatives are `120`, `240`, and `480 Hz`. Increasing the GUI
rate adds respectively 2, 4, or 8 PhysX substeps to every 60 Hz render update.
Do not diagnose this cost from joint count alone: compare stages at the same
runtime timestep.

The repeatable comparison tool reports USD complexity, render updates/s and
raw PhysX steps/s for both stages:

```bash
uv run python -m exporterV2.performance_benchmark \
  --baseline legacy=/tmp/tree_v2_day_50_legacy.usda \
  --candidate plantstate=/tmp/tree_v2_day_50_leaves.usda \
  --physics-hz 60,120,240,480 \
  --output /tmp/v2_day50_performance.json
```

On Isaac Sim 4.5, constructing `World` after opening an existing stage can
leave `SimulationManager` unaware of its already-authored `PhysicsScene`; its
rate getter then silently returns 60 Hz. The loader synchronizes that runtime
registry explicitly and reapplies the selected cadence after `World.reset()`.
Do not remove this compatibility shim merely because the USD attribute reads
480 Hz: authored and effective runtime rates are separate checks.

Solver iterations remain centralized in `PhysicsRuntimeConfig`; changing them
affects generated USD metadata and must be documented with the output.

## Leaf movement vs joint cost

PlantState leaf checkpoints expose two policies without changing geometry,
poses, masses or material parameters:

```bash
# Default: D6 at petiole and rachis, distributed leaf-support bending
./run_debugV2.sh --day 50 --organ leaves --leaf-joint-policy distributed

# Lower-cost fallback: D6 petiole, fixed rachis
./run_debugV2.sh --day 50 --organ leaves --leaf-joint-policy optimized
```

Petiolules and blades are rigid visuals in both modes. Do not make them rigid
bodies merely to restore rachis bending: the historical day-50 asset spent 91
additional D6 joints on petiolules. Compare policies at the same interactive
physics rate before changing stiffness, damping or solver iterations.

If PlantState branches look cylindrical, check for `--visual-quality
performance`. The default `realistic` mode restores the original V2 profile:
14 radial samples, 5 mm axial spacing and 9 radius-transition samples on every
segmented vegetative axis. The performance profile uses 12/12 mm/5. Neither
profile authors visual cylinders.

Leaf supports retain their EI-derived stiffness but use the historical V2
damping ratio (`0.3`). Leaf dry biomass from PlantState (mg converted to kg) is
aggregated into the host support mass and center of mass; petiolules and blades
remain free of rigid bodies, colliders and joints.

## Appendage Angles or Truss Visuals Differ from V2

PlantState profiles default to `--appendage-pose-mode v2-aesthetic`. This keeps
the native GroIMP organ topology and dimensions while applying the historical
V2 local angles to lateral petiolules and pedicels. Use
`--appendage-pose-mode canonical` to inspect the raw GroIMP pose. The manifest
always stores both `source_pose` and `authored_pose`, and mesh, collider, joint,
leaf blade and tomato all follow the authored pose together.

Leaf blades still use the historical longitudinal fold, centre arch and tip
sag. Truss rachides are physically segmented so they can bend, but visually
use one continuous skinned/segmented organic axis with the historical truss
material. A generated truss containing `HistoricalTrussRachisVisual` cylinders
or a non-zero `visual_cylinders` audit count is a regression.

## Day-160 Truss Stiffness and Damping

Use the in-memory calibration presets before changing active values in
`core/tree_config.py`:

```bash
./run_debugV2.sh --day 160 --organ truss-supports \
  --lateral-joint-policy fixed --truss-calibration-preset balanced
```

`compliant`, `balanced`, `firm` and `current` select independent rachis and
pedicel values. `--truss-damping-override 2|4|7` changes damping only after a
preset has been selected; use it for persistent oscillation, never to correct
static sag. The exporter applies these values to an in-memory BRANCHES copy,
so repeated candidates do not rewrite `tree_config.py`.

After visual approval, copy the selected values to `TrussPhysicsConfig`:

- `RACHIS_YOUNG_MODULUS` and `RACHIS_DAMPING_RATIO` for the main truss axis;
- `PEDICEL_YOUNG_MODULUS` and `PEDICEL_DAMPING_RATIO` for pedicels;
- `PEDICEL_DRIVE_STIFFNESS_SCALE` for the additional pedicel drive scale.

The active day-160 selection is rachis `20 GPa`, pedicel `4 GPa`, damping
ratio `4` for both and pedicel drive scale `0.2`.

Truss density is deliberately limited to `2000 kg/m3`; ordinary plant tissue
remains `1000 kg/m3`. Do not raise density to hide an unstable drive. In one
temporary day-50, 480 Hz test
with the new V2-authored pedicel pose, damping ratios 7 and 4 caused spontaneous
fruit detachment, while ratio 2 completed five simulated seconds. This is a
diagnostic observation, not a selected default: mass density, the 6 N break
force, overlap filters and authored pose also influence the result.

Pedicel length is a geometry control, not a physics parameter. Adjust
`TrussGeometryConfig.PLANT_STATE_PEDICEL_LENGTH_SCALE` (currently `3.0`) instead
of the legacy `PEDICEL_LENGTH`: PlantState supplies its own native length. The
source length remains in the manifest while mesh, collider, endpoint and fruit
move together to the scaled authored length.

For the fruit-bearing stage, first try
`--terminal-solver-preset stabilized`, which changes only terminal bodies from
32/1 to 64/4 solver iterations. If that is insufficient, test
`--truss-armature-multiplier 1` and then `4`; the value is multiplied by each
truss link's local inertia and authored only on truss D6 joints. Both controls
are explicit fallbacks and remain disabled in the isolated support baseline.

## Tomatoes Fall Immediately

Physical tomatoes in the canonical GroIMP PlantState pipeline are unsupported
on mature plants. The normal day-based workflow intentionally uses
`truss-supports`, so no tomato rigid body, collider or terminal joint is
authored. Use `fruit-visual` when static tomato geometry is sufficient.

The historical experimental path is guarded:

```bash
./run_debugV2.sh --day 160 --organ full \
  --allow-experimental-fruit-physics
```

It authors external tomato bodies with `excludeFromArticulation=true` and
immediate break force `6 N`. There is no soft-start, gravity ramp, deferred
arming or automatic equilibrium-pose capture in the production runtime.

Do not infer stability merely from finite coordinates. The headless monitor
fails an unforced terminal body that travels more than `0.25 m` and reports its
path and joint ancestry. During the 2026-08-25 canonical checkpoint, day 80
showed exactly this failure while `truss-supports` without fruit remained
stable. A stricter day-160 A/B test then disabled all 504 colliders in memory,
raised the break force to `1e9 N` and preserved exclusion from the articulation.
It still failed to settle after 12 seconds. Residual collision contacts are
therefore not the primary cause in that configuration; the 72 external
terminal bodies and FixedJoints remain the unsupported boundary. Exact
measurements and the comparison with `main` are recorded in
[`../../groimp_bridge/BRANCH_REPORT_2026-08-25.md`](../../groimp_bridge/BRANCH_REPORT_2026-08-25.md).

## Tomatoes Do Not Detach

This section applies only to the explicitly enabled experimental `full`
profile or to the legacy BRANCHES path.

Verify that detachment is enabled and serialized:

- `TOMATO_DETACHMENT_ENABLED = True`
- tomato body under `/World/TerminalBodies`
- FixedJoint has a finite `breakForce`
- `physics:excludeFromArticulation = True`
- the interacting object actually applies enough impulse to exceed the break force

## Collision Instability Near Tomatoes

Inspect `FilteredPairs` on the tomato body. Detachable tomatoes should filter collisions toward their pedicel and related truss rachis. Missing filters can make contact resolution fight the FixedJoint.

## A Different Plant Day Reports Initial Overlaps

Canonical PlantState profiles default to `--initial-overlap-policy filter`.
The exporter measures the actual authored capsule, cylinder, and sphere
colliders, then filters only each rigid-body pair that overlaps at rest. The
manifest records collider paths, roles, canonical IDs, contact count, and
maximum penetration depth.

Sphere checks must be order-independent. A sphere is encoded internally as a
zero-length swept segment; both point-segment orders have regression coverage.
If a mature day behaves differently after changes to the narrow phase, run the
reversed sphere-capsule and sphere-cylinder tests before adding more filters.

This prevents startup impulses but permanently disables contact between that
specific pair. It does not alter the GroIMP pose and it does not filter the
whole stem. Use `--initial-overlap-policy error` when investigating whether a
new overlap is a modelling problem rather than an intentional dense canopy.

## Physical Petiolules Are Slow or Exceed the Budget

`--physical-petiolules` is intentionally disabled by default. It adds one
rigid body, two capsule proxies, and one D6 joint for every positive
petiolule/terminal rachis axis. At day 50 the complete canonical plant rises
from 123 to 254 D6 joints because PlantState retains more native leaf and truss
segments than the old CSV grouping.

Normal exports stop above 230 D6 joints. `--allow-over-budget` is accepted only
with `--physical-petiolules` and is a diagnostic comparison switch, not a
recommended production configuration.

## Demo Paths

Visual and Isaac demo scripts now live under `src/exporterV2/demos/`. Root runner scripts are intentionally limited to `run_mainV1.sh` and `run_mainV2.sh`.
