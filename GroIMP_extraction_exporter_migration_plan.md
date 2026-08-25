# GroIMP Canonical Plant Extraction and Exporter Migration Plan

## 1. Objective

The goal of this phase is to replace the current partially inferred GroIMP-to-USD pipeline with a **single, validated, canonical plant extraction pipeline**.

The new pipeline must correctly recover the topology, organ parameters, turtle-based orientation, and spatial geometry of the plant from GroIMP. The resulting representation must then become the common input for both ExporterV1 and ExporterV2.

The final architecture should support two independent workflows:

```text
LIVE / BRIDGE MODE

GroIMP
  ↓ GroPy / GroLink
Canonical Extractor
  ↓
PlantState
  ↓
ExporterV2
  ↓
Isaac Sim
```

and:

```text
OFFLINE / SERVERLESS MODE

plant_state_day_N.json
  ↓
PlantState Loader
  ↓
ExporterV1 or ExporterV2
  ↓
USD
```

There must **not** be a separate parser for the bridge and another parser for offline export. Both workflows must converge on the same `PlantState` representation and the same downstream code.

---

## Implementation status — 2026-08-21

### Phase A — GroIMP Ground Truth Inspector: `COMPLETED`

Implemented under `src/groimp_bridge/`:

* lifecycle-safe GroPy client with guaranteed workbench cleanup;
* isolated project runtime that copies the GSZ and external inputs under `/tmp`,
  so `Dynamic_Model` cannot overwrite the source model outputs;
* public `inspect_project(...)` and `inspect_workbench(...)` APIs;
* `uv run python -m groimp_bridge.inspector` CLI;
* versioned `groimp_inspection/1.0` diagnostic JSON report;
* complete ProjectGraph node/edge retention, successor/branch classification,
  typed organ/turtle attributes, `location()`/`direction()` world anchors,
  console output, type counts, and explicit diagnostics;
* enrichment for `Root`, `Internode`, `Leaf`, `Truss`, `Fruits`, `Meristem`,
  `PlantBase`, `RH`, `RL`, `RU`, and `Translate`.

Validation completed:

```text
Offline inspector suite: 14 passed
Live GroIMP day-1 test: 1 passed
Live GroIMP day-25 slow test: 1 passed
```

The day-1 live run returned 51 nodes, 50 edges, 3 internodes, and 5 leaves.
The three extracted internode lengths matched the existing day-1 reference.
The live test also verified that the SHA-256 hash of the original day-1 CSV was
unchanged after extraction. The day-25 run verified direct `Truss`, `Fruits`,
and nullable reproductive-array extraction.

Commands:

```bash
uv run pytest src/groimp_bridge/tests/test_offline_inspector.py -q

RUN_GROIMP_TESTS=1 \
uv run pytest src/groimp_bridge/tests/test_live_inspector.py \
  -m "groimp and not slow" -q

RUN_GROIMP_TESTS=1 RUN_GROIMP_SLOW_TESTS=1 \
uv run pytest src/groimp_bridge/tests/test_live_inspector.py \
  -m "groimp and slow" -q
```

At the end of Phase A, local transform matrices, the shared turtle resolver,
push/pop resolution, and endpoint validation were intentionally left open.
Those items are completed by Phase B below.

### Phase B — GroIMP Turtle Resolver: `COMPLETED`

Implemented under `src/groimp_bridge/`:

* public `TurtleFrame`, `ResolvedNodePose`, `TurtleResolution`,
  `TurtleResolutionError`, and `resolve_turtle(...)` API;
* lifecycle-safe disposable `newRGG` workbenches with source update,
  compilation, and guaranteed cleanup;
* source-controlled live fixtures for one/two internodes, `RH`, `RL`, `RU`,
  `RG`, local `Translate`, nested branch scopes, organ expansion, a leaf, and a
  lateral branch;
* deterministic local-to-world 4x4 frames, incoming/outgoing node poses,
  internode endpoints, traversal order, and explicit topology diagnostics;
* strict rejection of cycles, missing structural nodes, and multiple
  structural parents, while unknown nodes and edge codes remain diagnostic.

The controlled GroIMP oracle established these conventions:

```text
column vectors; local-to-world matrices
column 0 = local X / left
column 1 = local Y / up
column 2 = local Z / head
column 3 = world position
composition = world @ local
angles = degrees
RH = right-handed rotation around local head
RL = right-handed rotation around local left
RU = right-handed rotation around local up
Translate = displacement along the current local axes
RG = minimal alignment of head to world negative Z
```

Successor and branch children inherit the same parent outgoing frame. Branches
are resolved independently; therefore nested `[...]` scopes restore the parent
state without relying on ProjectGraph iteration order. An `Internode` advances
along local head because the model's `Organ` inherits from turtle `M`.

The mature live model exposed an important cache boundary: after
`calcDimensions()`, the public biological `Internode.length` may already hold
the new value while GroIMP's rendered M-step still uses the preceding effective
advance. When native anchors are present, the resolver therefore calibrates the
effective axial step from the internode and successor anchors and reports the
largest declared/effective difference. Snapshot fixtures without anchors use
the declared `length` fallback.

Validation completed:

```text
Offline Phase A+B suite: 32 passed
Controlled live turtle fixture: 1 passed
Live GroIMP day-1 resolver validation: 1 passed
Live GroIMP day-25 resolver validation: 1 passed
Live GroIMP day-80 mature-plant validation: 1 passed
Combined live-enabled bridge suite: 36 passed
Open temporary workbenches after tests: 0
```

The day-80 validation covered 301 nodes, 300 edges, and 121 branch edges,
including 26 internodes, 27 leaves, 9 trusses, 9 fruit modules, and 16 `RG`
operations. The resolver reached all 301 nodes and compared all 285 enriched
world anchors. Maximum observed errors were `4.04e-08` for position and
`1.22e-07` for head direction, below the declared `1e-6` live tolerance. No
unknown edge codes were present.

Commands:

```bash
uv run pytest src/groimp_bridge/tests -q

RUN_GROIMP_TESTS=1 \
uv run pytest src/groimp_bridge/tests/test_live_turtle_fixture.py -q

RUN_GROIMP_TESTS=1 \
uv run pytest src/groimp_bridge/tests/test_live_inspector.py \
  -m "groimp and not slow" -q

RUN_GROIMP_TESTS=1 RUN_GROIMP_SLOW_TESTS=1 \
uv run pytest src/groimp_bridge/tests/test_live_inspector.py \
  -m "groimp and slow" -q

RUN_GROIMP_TESTS=1 RUN_GROIMP_SLOW_TESTS=1 \
uv run pytest \
  src/groimp_bridge/tests/test_live_inspector.py::test_live_day_80_resolves_the_full_mature_plant_without_source_changes \
  -q
```

The day-1 live test fingerprints the source GSZ and every file under
`model/input/` and `model/output/` before and after isolated inspection; all
SHA-256 hashes remained unchanged. `groimp_inspection/1.0` remains the raw
diagnostic report: resolved matrices are returned separately by
`resolve_turtle(...)` and are not embedded into that schema.

### Phase C — Validate against GroIMP: `COMPLETED`

Implemented on 2026-08-21:

* public `build_rendered_geometry(...)`, `validate_rendered_geometry(...)`,
  `ReconstructedGeometry`, `AxisPrimitive`, `SpherePrimitive`, `ObjMesh`, and
  `GeometryValidationReport` APIs;
* exact RGG-production reconstruction for main/lateral internodes, petioles,
  leaf rachides, left/right petiolules, truss rachides, pedicels, and fruit
  spheres;
* GroIMP OBJ subscene export, parser, and documented axis conversion
  `(x,y,z) OBJ -> (x,z,y) GroIMP`;
* spatial endpoint/radius matching that does not depend on absent OBJ object
  names or shared face indexes;
* deterministic debug OBJ containing starts, endpoints, local axes, and
  parent-axis connections;
* versioned `groimp_geometry_validation/1.0` report with checks, tolerances,
  measurements, ambiguity reasons, and diagnostics.

Declared tolerances are `1e-6` for anchor positions, `1e-5` for directions,
and `max(1e-5, 1e-3 * dimension)` for rendered endpoints and dimensions.
Controlled live cylinder/sphere fixtures passed. Real-plant results were:

```text
day 1:  9 passed, 0 ambiguous, 0 failed, 0 not-recoverable
day 25: 11 passed, 1 overlap-ambiguous, 0 failed, 0 not-recoverable
day 80: 32 passed, 0 ambiguous, 0 failed, 0 not-recoverable
```

The validation exposed a GroIMP cache boundary analogous to Phase B: a mature
organ's ProjectGraph anchor can lead its rendered subscene by one update. The
report preserves the raw offset and applies only that per-organ translation
before validating local direction, length, radius, and endpoints; it never
rotates or rescales the prediction. The largest observed offset was
`7.704e-4` GroIMP units on the young day-25 fruit module. Dense subscenes can
also contain an axis with indistinguishable overlapping shells; this is
reported as `ambiguous`, never silently accepted as a match.

### Phase D — Compare new extraction with the current CSV: `COMPLETED`

Implemented on 2026-08-21:

* public `compare_representations(...)`, `MigrationComparisonReport`, and
  `validate_project(...)` APIs;
* `uv run python -m groimp_bridge.migration_validation` CLI;
* versioned `groimp_migration_comparison/1.0` JSON plus deterministic Markdown
  summary;
* one isolated run feeding native GroIMP data, graph CSV wire data, an actual
  V1 USDA stage, and V2 `BRANCHES`/terminal bodies without Isaac Sim;
* deterministic duplicate matching, organ counts, biological parent topology,
  scalar/array length and radius fields, local angles, V1 world orientation and
  endpoint errors, and V2 geometry/physics adaptations;
* explicit `EXPECTED_IMPROVEMENT`, `EXPECTED_SIMPLIFICATION`,
  `PHYSICS_ADAPTATION`, `UNKNOWN_DIFFERENCE`, and `LIKELY_BUG` classifications.

The checked GSZ exposes `Dynamic_Model` but not the newer checkout helper
`exportPlantGraph`. The validator first redirects `PATH_OUTPUT` under `/tmp`;
when that helper is absent it projects the same live native snapshot into the
legacy CSV wire shape and records
`same_run_native_projection_for_legacy_gsz`. This validation-only projection
is not the future Phase G compatibility adapter.

Days 1, 25, and 80 all generated V1 USD and V2 configuration successfully.
Native and CSV biological counts/topology matched, all compared fields stayed
within tolerance, and the final reports contained zero `UNKNOWN_DIFFERENCE`
and zero `LIKELY_BUG`. Expected differences document V1 world-Z heuristics and
same-rank USD path collisions, CSV omission of world transforms, and V2
averaging/clamping/resampling for physics.

Validation commands and final results:

```bash
uv run pytest src/groimp_bridge/tests -m "not groimp" -q
# 39 passed

RUN_GROIMP_TESTS=1 RUN_GROIMP_SLOW_TESTS=1 \
uv run pytest src/groimp_bridge/tests -m groimp -q
# 8 passed
```

The day-1 acceptance retained 51 nodes, 50 edges, 3 internodes, 5 leaves, and
the three known lengths. Day 80 retained 301 nodes, 300 edges, 26 internodes,
27 leaves, 9 trusses, and 9 fruit modules. Live tests fingerprint the GSZ and
all files under `model/input/` and `model/output/`; SHA-256 manifests were
identical before and after, and no workbench remained open.

Remaining limits are the renderer-cache translation recorded above, ambiguous
geometric separation in overlapping subscenes, and validation of leaflet
surface assets beyond their supporting axes. `groimp_inspection/1.0` remains
unchanged.

### Phase E — Build the canonical extractor: `COMPLETED`

Implemented on 2026-08-21:

* exporter-independent `plant_state` package with `PlantState`, native graph
  nodes/edges, typed organ records, turtle operations, local/world poses,
  canonical axis/sphere primitives, provenance, units, and diagnostics;
* public `extract_plant_state(...)`, `extract_workbench_state(...)`, and
  lifecycle-safe `extract_project_state(...)` APIs;
* deterministic single-plant subtree selection based on biological descendants
  rather than `plant_number` alone;
* explicit exclusion of the zero-organ `PlantBase`/sphere marker observed at
  day 80, while retaining the true plant's resolved world placement;
* reuse of the Phase B resolver and Phase C RGG geometry reconstruction without
  duplicated turtle or production logic;
* strict validation for references, topology, cycles, parents, homogeneous
  frames, local/world consistency, dimensions, organ arrays, and primitive
  coverage.

The canonical layer contains no GroIMP client, USD, PhysX, exporter, or Isaac
Sim concepts. Native IDs are guaranteed only within one GroIMP workbench and
that scope is recorded in metadata/diagnostics. `Internode` records retain both
the declared biological length and the anchor-calibrated effective length.
Leaves and fruit modules retain all validated supporting axes and fruit spheres;
leaflet surface assets remain explicitly outside schema 1.0.

### Phase F — JSON persistence: `COMPLETED`

Implemented on 2026-08-21:

* strict `plant_state/1.0` JSON schema and public `save_plant_state(...)`,
  `load_plant_state(...)`, `validate_plant_state(...)`, and
  `plant_states_equivalent(...)` APIs;
* deterministic sorted serialization, final newline, `NaN`/infinity rejection,
  exact schema-version handling, and rejection of unknown structural fields;
* `uv run python -m groimp_bridge.extractor` CLI with explicit output path and
  isolated source-project execution;
* serverless JSON reload and exact round-trip coverage.

Final validation results:

```text
Offline Phase A-F bridge suite: 45 passed
Live Phase A-F bridge suite:    11 passed

day 1:  26 canonical nodes, 25 edges, 3 Internode, 5 Leaf, 20 axes
day 25: 152 canonical nodes, 151 edges, 15 Internode, 17 Leaf,
        1 Truss, 1 Fruits, 130 axes, 1 fruit sphere
day 80: 251 canonical nodes, 250 edges, 26 Internode, 27 Leaf,
        9 Truss, 9 Fruits, 347 axes, 72 fruit spheres
```

All three live states passed native-frame/primitive equivalence and
`GroIMP -> PlantState -> JSON -> PlantState` equality. The day-1 three known
internode lengths remained unchanged. GSZ, model inputs, and existing outputs
had identical SHA-256 manifests before and after live extraction; lifecycle
contexts left no temporary workbench open. No Isaac Sim process was used.

Remaining limits are leaflet surface assets, renderer-cache offsets retained
only in Phase C diagnostics, and lack of cross-run identity guarantees for
native GroIMP node IDs. ExporterV1/V2 and the legacy CSV path remain unchanged.

**Migration decision (2026-08-21):** Phase G is `SKIPPED BY DESIGN`. Historical
CSV compatibility is not a product requirement; Git retains the working CSV
pipeline for regression archaeology and rollback. The exact checkpoint is
commit `d7f5038` (`checkpoint: legacy CSV pipeline before PlantState
migration`).

### Phase H — ExporterV1 migration: `COMPLETED`

Implemented on 2026-08-21:

* canonical `PlantState -> V1-compatible rendering view -> V1 visuals` path;
* removal of the legacy CSV loader, inferred CSV hierarchy, and CSV fallback
  from normal ExporterV1 operation;
* unique USD paths derived from canonical node IDs and a topology prim for
  every canonical node, with parent ID, incoming edge kind and GroIMP node ID;
* one organ prim per canonical organ; internode, compound-leaf and fruit-module
  visuals retain the established V1 style;
* `PlantBase`, `Truss` and `Meristem` remain explicit metadata/topology prims
  without invented geometry;
* conservative exact-overlap handling: all canonical organ prims remain, but
  a second visual with identical class, exact world pose and exact visual
  geometry records `autotom:geometryDuplicateOf` and does not duplicate the
  geometry. Near overlaps and intersections are never collapsed;
* `uv run python -m exporterV1 --day N` serverless CLI and `run_mainV1.sh` with
  `--isaacsim`, `--plant-id`, `--input`, `--output`, `--generate-only`, and
  `--headless`;
* static Isaac loader that stays interactive in GUI mode and performs a finite
  stage-open smoke test in headless mode.

Default serverless inputs are committed under `data/plant_states/` for days 1,
25 and 80. The exporter does not contact GroIMP and fails before Isaac startup
when the requested JSON is absent or its day/plant metadata do not match.

### Phase I — V1 before/after validation: `COMPLETED`

The deterministic `exporter_v1_manifest/1.0` sidecar audits PlantState organ
counts, USD organ prims, expected/created visuals, topology parentage, node
coverage, unique paths, non-visual organs and exact-overlap decisions.

Real-plant results:

```text
day 1:  3 Internode, 5 Leaf; all counts passed
day 25: 15 Internode, 17 Leaf, 1 Truss, 1 Fruits, 1 fruit sphere; all passed
day 80: 26 Internode, 27 Leaf, 9 Truss, 9 Fruits, 72 fruit spheres; all passed

path collisions:                  0
missing supported organ prims:    0
unexplained count differences:    0
exact coincident visual pairs:    0 on days 1/25/80
```

Legacy/new V1 world bounding-box heights were `0.053735/0.053735 m` (day 1),
`0.188952/0.183726 m` (day 25), and `0.296614/0.290814 m` (day 80). The mature
height differences are approximately 2–3%. ExporterV1 rebases `PlantBase` to
the stage origin for convenient static inspection and stores the original
GroIMP offset as `autotom:sourceWorldOrigin`; the canonical PlantState retains
the original world placement. This comparison validates overall scale, not
graphical parity with GroIMP: leaf, truss and fruit styling intentionally
remains V1.

Validation commands and results:

```bash
uv run pytest src/groimp_bridge/tests src/exporterV1/tests -m "not groimp" -q
# 49 passed, 11 deselected

RUN_GROIMP_TESTS=1 RUN_GROIMP_SLOW_TESTS=1 \
uv run pytest src/groimp_bridge/tests src/exporterV1/tests -q
# 60 passed

./run_mainV1.sh --day 1 --isaacsim --headless
# static stage opened in Isaac Sim; exit code 0
```

The live tests fingerprint the source project, inputs and original outputs;
the model tree remained byte-for-byte unchanged and no source-model Git diff
was produced. Workbench lifecycle tests passed. Phase J is the next official
task. V2 must keep complete visual coverage while budgeting physical joints:
up to 220 no filtering, 220–230 warning/explicit aggregation review, and above
230 explicit filtering or merging with canonical organ references retained on
every aggregate physical link.

### Phase J — ExporterV2 migration: `IN PROGRESS — FLEXIBLE VALIDATION BLOCKED`

Checkpoint recorded on 2026-08-21. The canonical day-based V2 adapter,
serverless CLI, launcher, complete visual mapping, 220/230 budget policy,
finite-shape collision planning, source/authored pose provenance, OpenUSD
authoring and `exporter_v2_manifest/1.0` are implemented. Inputs for days 1,
25, 80 and the newly extracted day 160 produce 20/130/347/347 visual axes;
days 80 and 160 produce 72 fruits and 216 physical axes (215 D6), so no
physical aggregation is required by the budget.

Isaac Sim passed 5-second day-25 locked/flexible runs and the scalable
kinematic locked baseline at day 80. The dynamic flexible day-80 articulation
diverges before one simulated second at 480 Hz, and also at 960 Hz with
stiffness scales 1x, 2x and 4x (damping scaled by the square root). Phase J is
therefore deliberately **not** marked completed. The safe user default remains
`locked` until the flexible blocker is resolved.

The exact diagnostic sequence, completed results, day-160 extension and
remaining acceptance checks are in
`docs/PHASE_J_HANDOFF_2026-08-21.md`. The next official task is to resolve this
Phase-J flexible articulation blocker; the live in-memory bridge remains out
of scope until Phase J passes.

#### Conservative V2 stem checkpoint — 2026-08-24

The flexible diagnosis was reset to the last proven V2 authoring architecture
instead of continuing to duplicate its physics. The `stem` diagnostic profile
now follows `PlantState -> BRANCHES adapter -> build_stage()` and uses the
established `skinned` backend in `segmented` mode. It therefore retains the V2
organic per-rigid-link mesh, invisible compound capsule colliders, FixedJoint
topology and Isaac lifecycle; no visual cylinders are introduced.

`BRANCHES` accepts optional per-link `link_specs`. Canonical PlantState inputs
use individual internode length, radius and rebased GroIMP local-to-world frame.
`--pose-mode legacy` omits those frames and retains the procedural rest-pose
fallback. Configurations without `link_specs` remain backward compatible.
Link paths keep the historical hierarchy while appending readable GroIMP IDs,
for example `trunk/trunk_Link_01_Internode_g421092`.

The day-10 serverless checkpoint produces five order-zero internodes, five
rigid bodies, five FixedJoints including the world anchor, ten capsule
colliders, five `OrganicVisual` meshes, zero D6 joints and zero visual
cylinders. Offline adapter/backend regression tests pass. The one-second
flexible-preset Isaac smoke at 480 Hz also passes with five finite bodies,
exactly zero displacement/drift and no reported errors. Phase J remains
`IN PROGRESS`.

#### Conservative V2 lateral-branch checkpoint — 2026-08-24

The `laterals` diagnostic profile now follows the same established
`PlantState -> BRANCHES -> build_stage()` route. It contains only the fixed
stem and native lateral Internodes: leaves, leaf supports, trusses and fruits
are deliberately absent. The adapter walks canonical `parent_id` links through
turtle pass-through nodes and builds maximal native Internode chains; it does
not reuse CSV ranks, geometric nearest-neighbour matching or phyllotaxis.

For day 50 this resolves 10 stem Internodes and four lateral chains of four
Internodes. Their native attachments are trunk links `2, 2, 3, 7`, with
readable paths such as `Branch_s2_o0_g421238/..._Internode_g421238`. Canonical
mode preserves every GroIMP frame, length and radius. The legacy procedural
45-degree pose remains available only as a diagnostic comparison and reports
its initial overlaps without making them canonical acceptance failures.

The flexible canonical stage audits as:

```text
26 rigid bodies
10 FixedJoints (fixed stem)
16 D6 joints (four complete lateral chains)
52 invisible capsule colliders
26 segmented OrganicVisual meshes across 5 visual axes
0 visual cylinders
10 intentional/filtered initial capsule overlaps
0 active unfiltered overlaps
```

The established V2 parent/child, attachment-neighbour and sibling collision
filters are retained. The two branch roots on trunk link 2 are explicitly
filtered as siblings. Source and authored poses, physical gains, collider
radii, attachment mapping and overlap classifications are recorded in
`exporter_v2_laterals_checkpoint/1.0`.

Offline ExporterV2 regression completed with `88 passed, 2 skipped`. Isaac at
480 Hz passed the one- and five-second day-50 flexible runs. `World.reset()`
performs one mandatory internal physics step, so the runtime temporarily
suspends gravity only for that step and restores it before explicit
simulation. The measured reset projection is below `8.8e-8 m`; subsequent
gravity sag reaches `5.56%`, stem/root drift remains zero, and the five-second
tail speed is approximately `1.54e-4 m/s`. The GUI monitor reports 16
interactive D6 bodies, enabled mouse grab including invisible colliders, and
no non-finite body. The user approved the initial canonical shape and
Shift+drag behavior on 2026-08-24. The lateral checkpoint is complete, but
Phase J remains `IN PROGRESS`; leaf supports are the next incremental
implementation.

#### Phase J interactive performance checkpoint — 2026-08-24

The day-50 `leaves` checkpoint exposed an apparent regression to roughly
11 FPS. A same-process comparison showed that the PlantState stage is not the
cause: at equal settings it is substantially faster than the checkpoint USD
from commit `d7f5038`. The difference was the loader. The historical V2
constructed `World(stage_units_in_meters=1.0)` and therefore ran at Isaac's
60 Hz default, despite the saved USD declaring 480 Hz; the new diagnostic
loader applied the authored 480 Hz explicitly.

Measured on the same machine and 60 Hz renderer:

```text
new day 50: 60 Hz 51.0 FPS; 120 Hz 34.7; 240 Hz 20.9; 480 Hz 12.0
equal 480 Hz: legacy 1.74 FPS; PlantState 11.89 FPS
historical 60 Hz loop: legacy 11.5 FPS; PlantState 50.5 FPS
```

Interactive and validation rates are now intentionally separate. GUI runs
default to 60 Hz and expose `--interactive-physics-hz 60|120|240|480`;
headless/stress runs retain `--physics-hz 480|960`. Stability telemetry records
authored/runtime rates, render updates, actual physics steps and simulated
time instead of assuming one physics step per rendered frame. The reusable
`performance_benchmark.py` writes
`exporter_v2_performance_comparison/1.0` reports for legacy/candidate stages.

The permanent benchmark was rerun after synchronizing Isaac Sim 4.5's runtime
`PhysicsScene` registry and reapplying the selected cadence after reset. With
30 render frames and 120 isolated physics steps per stage, it measured:

```text
rate       legacy FPS   PlantState FPS   render speedup
60 Hz          11.24          49.69           4.42x
120 Hz          6.34          34.30           5.41x
240 Hz          3.34          21.07           6.31x
480 Hz          1.70          11.15           6.54x
```

The day-50 `leaves` stage also passed five simulated seconds headless at an
effective 480 Hz: 2,400 physics steps, no NaN/Inf or invalid articulation, and
constant bounded displacement. The JSON report is
`/tmp/exporter_v2_day50_performance.json`; `/tmp` is intentionally not a
versioned artifact.

A full GUI session at the default effective 60 Hz ran for 15.83 simulated
seconds before normal user closure. It measured 39.39 rendered frames/s in the
interactive application, exposed 43 dynamic grabbable bodies, kept mouse grab
and invisible-collider picking enabled, and reported no non-finite body or
runtime error. Telemetry confirms interaction availability; subjective
Shift+drag behavior remains a human visual approval rather than an automated
claim.

The segmented PlantState visuals retain exact canonical endpoints, radii and
poses while removing redundant straight-axis samples: the day-50 `leaves`
stage decreased from 81,036 to 39,304 mesh triangles and from 3.9 MB to
2.3 MB. This is retained as a useful visual optimization, although timestep
comparison proved that it was not the main source of the 11 FPS result.
Phase J remains `IN PROGRESS`.

#### Phase J distributed leaf-joint checkpoint — 2026-08-24

The optimized day-50 leaf stage was preserved in Git commit `1cccf33` before
restoring distributed leaf-support bending. PlantState incremental exports now
expose `--leaf-joint-policy optimized|distributed`; `distributed` is the
PlantState default and does not affect legacy BRANCHES input.

Both policies preserve exactly 81 rigid bodies, 162 capsule colliders, 343
meshes and 39,304 triangles. `optimized` authors 43 D6 plus 38 FixedJoint;
`distributed` changes the 28 canonical leaf-rachis links to D6, producing 71
D6 plus the 10 fixed stem joints. Petiolules, terminal rachides and blades stay
rigid visuals without bodies, colliders or joints. Canonical poses, dimensions,
mass calculation, combined parent-child stiffness, damping and drive scale 1
remain unchanged.

The distributed stage passed five simulated seconds at an effective 480 Hz
(2,400 steps), with no NaN/Inf, invalid articulation or fixed-stem drift. The
same-process benchmark measured:

```text
rate       optimized FPS   distributed FPS
60 Hz            48.87             44.22
120 Hz           33.69             29.18
240 Hz           21.22             17.56
480 Hz           11.67              9.61
```

The 60 Hz interactive target of 20 FPS is therefore satisfied with substantial
margin. The first GUI inspection confirmed stable Shift+drag but rejected the
visual/feel gate: reduced sampling made branches appear cylindrical and leaf
supports felt harder than the historical V2. Phase J stays `IN PROGRESS`.

#### Phase J realistic-mesh corrective candidate — 2026-08-24

The original V2 segmented profile is restored as the PlantState default via
`--visual-quality realistic`: 14 radial samples, 5 mm axial spacing and 9
radius-transition samples on stem, laterals, petioles, rachides and rigid
petiolules. `--visual-quality performance` retains the previous 12/12 mm/5
profile as an explicit fallback. Day 50 realistic has 78,712 triangles versus
39,304 performance triangles, with identical canonical pose and
81-body/71-D6/10-Fixed physics topology.

No stiffness was changed. Leaf-only damping returns to the historical V2 ratio
0.3. Canonical dry biomass from visual-only leaf parts is converted from mg to
kg and aggregated into the supporting body mass and center of mass (4.717 g at
day 50), without creating petiolule/blade rigid bodies, colliders or joints.
Automated stability, benchmark and final GUI review remain required before
accepting this corrective checkpoint.

A quick same-process day-50 benchmark at 60 Hz measured 45.52 FPS for
`performance` and 46.71 FPS for `realistic`; PhysX throughput was 72.28 versus
71.90 steps/s. This short run indicates that restoring mesh quality has no
material interactive cost on the test machine. The one-second 480 Hz smoke
also passed all 480 steps without NaN/Inf or articulation failure.

---

# 2. Why this refactor is necessary

The current pipeline contains several stages where information from GroIMP is reconstructed or approximated.

GroIMP's `Dynamic_Model()` already exports a CSV after updating the model by calling `exportPlantGraph(...)`.

However, that CSV is not a lossless dump of the native GroIMP graph. The RGG exporter generates `organ_index` during traversal and reconstructs parent relationships through rules based on `rank`, `order`, `parent_rank`, and organ class.

ExporterV1 then reconstructs another hierarchy from these fields rather than consuming the actual GroIMP graph edges.

ExporterV2 introduces further intentional adaptations. For example, trunk internode lengths and radii are averaged before creating the trunk representation.  Lateral branch rotations may be generated from profile-defined angles, deterministic jitter, and anti-collision corrections rather than directly from the GroIMP turtle state.  Truss azimuth is currently reconstructed from phyllotaxis, and truss geometry is partly generated from exporter configuration.

These choices were reasonable because the original parser did not have direct access to enough GroIMP geometric information. Visually, the existing exporters already approximate the GroIMP plant closely, so their existing logic remains valuable as a fallback and as a reference implementation.

Now, however, GroPy gives direct access to:

```python
wb.getProjectGraph()
wb.runXLQuery(...)
```

and the bridge tests have already demonstrated that the real GroIMP model can be opened, stepped, and inspected programmatically.

Therefore this phase should determine how much information can be recovered directly before any exporter-specific reconstruction is applied.

---

# 3. Primary design principle

The system must explicitly separate:

```text
BIOLOGICAL / GEOMETRIC TRUTH
             ↓
       Canonical PlantState
             ↓
 SIMULATOR-SPECIFIC ADAPTATION
             ↓
        USD / PhysX
```

The extractor should preserve as much information as GroIMP provides.

The exporter may simplify that information for performance or physics stability, but the simplification must be explicit, measurable, and downstream of the canonical representation.

Do **not** modify the extracted geometry merely because Isaac Sim needs a simpler plant.

---

# 4. GroIMP is the geometric ground truth

Whenever possible, use the actual turtle interpretation of the GroIMP graph as the reference geometry.

This is especially important for orientation.

The model explicitly inserts turtle transformations during development. For example, main-stem growth uses sequences involving:

```text
RH(...)
RU(...)
Internode
RU(...)
RH(...)
```

while lateral branches and leaves use additional `RL`, `RU`, and `RH` transformations.

Leaf rendering is also explicitly turtle-based. A `Leaf` expands into transformations such as:

```text
RH(counterClocKWiseOrientationPetiole)
RL(anglePetiole)
Cylinder(lengthPetiole, ...)
```

and its petiolules introduce further turtle rotations.

Internodes themselves expand to cylinders using the actual organ dimensions:

```text
Cylinder(length, internode_width_m / 2)
```

Therefore, the new extractor must investigate the actual sequence and semantics of:

```text
RH
RL
RU
Translate
Internode
Leaf
Fruits
Meristem
Root
...
```

rather than deriving final angles only from biological attributes such as `rank`, `order`, or phyllotaxis.

---

# 5. Important fallback rule

Do not discard the current exporter logic.

The existing exporters already produce plants that are visually close to GroIMP.

If a particular piece of final geometry cannot be recovered reliably from the ProjectGraph or XL queries, inspect the current V1/V2 reconstruction logic and move the minimum required calculation into the **canonical extraction stage**.

The preferred hierarchy is:

```text
1. Direct GroIMP graph/turtle information
        ↓ if unavailable

2. Direct GroIMP organ attributes
        ↓ if insufficient

3. Deterministic reconstruction using model semantics
        ↓ only as final fallback

4. Existing exporter heuristic
```

Any fallback must be clearly marked in the resulting data, for example:

```json
{
  "orientation_source": "groimp_turtle"
}
```

versus:

```json
{
  "orientation_source": "derived_phyllotaxis"
}
```

This is important for later validation.

---

# 6. Canonical PlantState

Create a versioned canonical Python data model named conceptually `PlantState`.

JSON should be its persistent representation.

Do **not** use the current `branches_v2_day_N.json` directly as the canonical format. Those files are useful references, but they already contain V2-specific decisions such as averaged dimensions, PhysX configuration, branch subdivision, simplified leaf structures, and other exporter adaptations. For example, the existing day-1 JSON already represents the trunk as a single branch with three equal-height links rather than preserving the three original GroIMP internode lengths.

The new JSON should represent the plant **before V1/V2 adaptation**.

A tentative schema should resemble:

```json
{
  "schema_version": "1.0",

  "metadata": {
    "day": 1,
    "plant_id": 1,
    "source": "groimp_api",
    "groimp_model": "...",
    "seed": null
  },

  "organs": [
    {
      "id": "...",
      "groimp_node_id": 421092,

      "type": "Internode",

      "rank": 0,
      "order": 0,
      "organ_index": 0,

      "parent_id": "...",

      "dimensions": {
        "length": 0.008802679,
        "diameter": 0.0062854785
      },

      "local_transform": {
        "...": "..."
      },

      "world_transform": {
        "...": "..."
      },

      "world_start": [0, 0, 0],
      "world_end": [0, 0, 0],

      "orientation_source": "groimp_turtle"
    }
  ],

  "edges": [...],

  "turtle": [...],

  "diagnostics": {...}
}
```

The exact schema should be designed only after inspecting several different organ classes and later simulation days.

---

# 7. Store both local and resolved geometry

This is a hard requirement.

For every relevant geometric element, preserve both:

```text
LOCAL INFORMATION
parent relationship
local turtle transformations
local orientation
local dimensions

AND

RESOLVED INFORMATION
world transform
world origin
world endpoint
world orientation
```

The turtle-resolution algorithm must exist in one shared core module.

ExporterV1 and ExporterV2 must **not independently interpret RH/RL/RU**.

Conceptually:

```text
GroIMP Graph
    ↓
Turtle Resolver
    ↓
world transforms
    ↓
PlantState
```

Then:

```text
ExporterV1 ─┐
            ├─ read resolved PlantState
ExporterV2 ─┘
```

This is one of the most important architectural changes in the whole refactor.

---

# 8. Phase A — Build a GroIMP Ground Truth Inspector

Do not begin by rewriting ExporterV1 or ExporterV2.

Extend the current bridge experiments first.

Build an inspection tool that opens `project_bridge.gsz`, executes a controlled number of `Dynamic_Model` steps, and extracts the native graph.

For every relevant node, inspect:

```text
node ID
node type
graph edges

organ attributes
rank
order
parent_rank
plant_number
dimensions
biomass fields when useful

RH angle
RL angle
RU angle
Translate x/y/z

Leaf-specific parameters
Fruits-specific parameters
Truss-related parameters
```

The current bridge tests have already proven that direct extraction of organ attributes works, including real `Internode.length`.

Use XL queries where `getProjectGraph()` does not expose a field.

Do not assume that an attribute named `length` is useful for every organ. For example, the current test showed `Leaf.length == 0`, while the model actually stores fields such as:

```text
lengthPetiole
diameterPetiole
anglePetiole
counterClocKWiseOrientationPetiole
segmentsLength
lengthPetiolules
...
```

The leaf geometry is generated from those specialized fields.

---

# 9. Phase B — Understand the turtle graph exactly

This phase is critical.

Implement small controlled tests to determine exactly how GroIMP's ProjectGraph represents turtle state and branch scopes.

Use progressively more complex samples:

```text
single internode

two sequential internodes

RH + internode

RU + internode

RL + internode

nested branch [...]

main stem + leaf

main stem + lateral branch

real Dynamic_Model plant
```

The test must establish:

* multiplication/order convention;
* local coordinate axes;
* sign conventions;
* degrees vs radians;
* push/pop behavior created by `[...]`;
* how `RH`, `RL`, and `RU` affect the turtle frame;
* how organ expansion affects subsequent turtle state;
* how `Translate` behaves;
* whether ProjectGraph traversal order is sufficient to reproduce GroIMP rendering.

Once established, create **one turtle resolver with automated tests**.

Its output should be world transforms or equivalent orientation frames.

---

# 10. Phase C — Validate against GroIMP

Do not trust the reconstruction merely because the numbers look plausible.

Create validation tests.

For a selected plant state, compare reconstructed geometry against what GroIMP visually renders.

Where feasible, generate debug primitives representing:

```text
organ start point
organ end point
local frame axes
parent-child connection
```

The strongest useful test is endpoint validation.

For each collidable axis-like organ:

```text
predicted_start
predicted_end
predicted_direction
predicted_length
```

should match the GroIMP geometry.

Tests should cover at least:

```text
main stem
lateral branch
petiole
leaf rachis
truss rachis
pedicel if recoverable
```

---

# 11. Phase D — Compare new extraction with the current CSV

For identical simulation state, collect three representations:

```text
A. Native GroIMP extraction

B. graph_day_N.csv

C. current exporter output
```

The current CSV contains useful biological attributes and should be used as a regression reference.

For example, current day-1 data contains the three real trunk lengths:

```text
0.008802679
0.008836136
0.006407313
```

and leaf-specific petiole/orientation values.

Create a comparison report containing at least:

```text
organ counts by type

topological relationships

length differences

radius/diameter differences

local angle differences

world orientation differences

world endpoint differences
```

Do not fail simply because the new extractor returns **more valid organs** than the old exporter.

An organ-count increase may be intentional because the current pipeline filters some organs.

Classify differences as:

```text
EXPECTED IMPROVEMENT
EXPECTED SIMPLIFICATION
PHYSICS ADAPTATION
UNKNOWN DIFFERENCE
LIKELY BUG
```

Unknown or likely-bug differences require investigation.

---

# 12. Phase E — Build the canonical extractor (`COMPLETED`)

After the ground-truth tests succeed, implement a shared extraction package.

A possible structure is:

```text
src/
  plant_state/
    models.py
    schema.py
    json_io.py
    turtle.py
    validation.py

  groimp_bridge/
    groimp_source.py
    extractor.py
    queries.py
```

The important architectural boundary is:

```text
GroIMP-specific code
       ↓
Canonical PlantState
       ↓
Exporter-independent code
```

Do not place USD or PhysX concepts in the canonical extraction layer.

---

# 13. Phase F — JSON persistence (`COMPLETED`)

Every successfully extracted state should optionally be serialized as something similar to:

```text
output/
  day_1/
    plant_state_day_1.json

  day_2/
    plant_state_day_2.json

  ...

  day_10/
    plant_state_day_10.json
```

Reusing the current `output/day_N/` organization is desirable.

The JSON must contain enough information that **GroIMP is no longer required to reconstruct the plant**.

Thus:

```text
GroIMP → PlantState → JSON
```

and:

```text
JSON → PlantState
```

must produce semantically equivalent objects.

Add a round-trip test:

```python
state_a = extract_from_groimp(...)
save_json(state_a)

state_b = load_json(...)

assert_equivalent(state_a, state_b)
```

---

# 14. Phase G — Temporary legacy CSV adapter

**Status (2026-08-21): `SKIPPED BY DESIGN`.**

The repository checkpoint named `checkpoint: legacy CSV pipeline before
PlantState migration` preserves the still-working CSV implementation. No
`LegacyCsvAdapter` will be added: normal operation moves directly to canonical
PlantState JSON, while historical behavior remains recoverable from Git.

Do not delete the CSV pipeline immediately.

During migration:

```text
old CSV
   ↓
LegacyCsvAdapter
   ↓
PlantState
```

may exist temporarily.

Its only purpose is backwards compatibility and regression comparison.

Do **not** make the new PlantState schema conform to limitations of the old CSV.

Once the new GroIMP extraction, JSON round-trip, V1 migration, and V2 migration are validated, remove the old CSV path and old generated CSV snapshots from normal operation.

---

# 15. Phase H — ExporterV1 migration first

**Status (2026-08-21): `COMPLETED`.** See the implementation-status section
for APIs, count manifests, live results and the serverless workflow.

Migrate V1 before V2.

V1 is useful as the geometric reference implementation because it does not contain the full PhysX complexity of V2.

Currently V1 begins with:

```python
snapshot = load_snapshot(csv_path, ...)
```

and reconstructs its hierarchy from the CSV.

Replace this boundary conceptually with:

```python
state = load_plant_state(json_path)
export_plant_usd(state, ...)
```

Do not redesign all rendering code at once.

First introduce an adapter if necessary:

```text
PlantState
   ↓
V1-compatible representation
   ↓
existing V1 renderer
```

Then progressively remove redundant V1 geometry inference.

The target is that V1 becomes primarily:

```text
PlantState → static USD representation
```

rather than:

```text
CSV → infer plant → infer angles → static USD
```

---

# 16. Phase I — V1 before/after validation

**Status (2026-08-21): `COMPLETED`.** Days 1, 25 and 80 passed canonical organ,
geometry and topology audits; the legacy/new scale comparison and Isaac smoke
results are recorded above.

For representative days, generate:

```text
old V1 USD
new V1 USD
GroIMP reference
```

Compare both visually and numerically.

Important days should include:

```text
very early plant
first lateral branches
developed leaves
first trusses
mature plant
```

Track differences in:

```text
plant height
organ count
branch count
branch endpoints
orientation
bounding box
truss position
leaf position
```

A changed pose is not automatically wrong.

If:

```text
old exporter angle = heuristic
new angle = GroIMP turtle-derived
```

then the new pose should be accepted if it matches GroIMP more accurately.

Document that distinction.

---

# 17. Phase J — ExporterV2 migration

**Status (2026-08-21): `IN PROGRESS — FLEXIBLE VALIDATION BLOCKED`.**

Canonical generation and collision/manifest authoring are implemented. Do not
mark this phase complete until the mature flexible articulation and day-160
extension pass the checks listed in `docs/PHASE_J_HANDOFF_2026-08-21.md`.

Only after V1 correctly reproduces the canonical plant should V2 be migrated.

V2 should consume the same `PlantState`.

However:

```text
V1:
PlantState → static geometry

V2:
PlantState
   ↓
physics adaptation
   ↓
articulation / rigid bodies
   ↓
visual geometry
   ↓
Isaac USD
```

Shared parsing/topology/turtle code should therefore live outside both exporters.

Only representation-specific code remains separate.

---

# 18. Preserve all organs by default

The canonical extractor must preserve all detected organs.

Do not replicate current V2 filters in the extraction layer.

For example, the current V2 loader can filter lateral branches through profile-defined `organ_indices`; this is an exporter-level simplification.

Instead:

```text
PlantState = complete

Exporter policy = selective
```

ExporterV2 may later decide:

```text
render all
render subset
make static
merge segments
reduce physics resolution
```

but this must not destroy the source representation.

---

# 19. Joint-budget constraint

ExporterV2 must measure the effect of restoring previously filtered organs before enabling all of them dynamically.

The existing V2 parser already counts D6 joints and warns when the configured maximum is exceeded.

Treat approximately **220–230 dynamic joints as the desired practical ceiling**, leaving safety margin below the observed problematic region around 250.

Therefore every V2 migration test must report:

```text
canonical organ count
rendered organ count
static organ count
dynamic branch count
D6 joint count
filtered/merged organ count
```

The Phase J decision thresholds are:

```text
predicted articulations <= 220: no physical filtering
predicted articulations 221-230: warning and explicit aggregation review
predicted articulations > 230: explicit filtering or merging required
```

Visual geometry must still represent every canonical organ. Every filtered or
merged physical link must retain the IDs of all canonical organs represented
by that link. Exact coincident visual duplicates may share one visual only when
the decision is recorded, matching the conservative V1 policy.

If restoring every organ causes excessive physics complexity, use controlled simplification.

Preferred options include:

```text
make nonessential components static

merge multiple visual segments onto fewer physical links

reduce articulation resolution

disable physics for selected organ categories

retain visual geometry while simplifying collision/physics
```

Never silently delete canonical data.

---

# 20. Geometry and physics must remain separate

A key rule for V2:

> More complete geometry must not imply more physics bodies.

For example:

```text
Canonical plant:
15 leaf-related segments

Visual representation:
15 segments

Physics representation:
3 or 4 rigid/collider segments
```

may be perfectly valid.

This distinction is especially useful for leaves, which are already largely static or non-colliding.

---

# 21. Collision-safe authoring

The canonical `PlantState` must retain the exact GroIMP pose.

Collision correction belongs to the V2 authoring stage.

Only elements that participate in collision need this processing.

Examples:

```text
stems
lateral branches
petioles if colliders
rachises
truss rachises
pedicels if colliders
other cylindrical collider bodies
```

Do **not** run expensive anti-collision adjustment for visual-only leaf meshes or other non-colliding objects.

---

# 22. Collision correction policy

Start from the exact canonical pose:

```text
GroIMP pose
    ↓
collision test
```

If safe:

```text
keep exact pose
```

If unsafe:

```text
minimal rotation around parent axis
```

If still unsafe:

```text
minimal additional tilt
```

If still unsafe:

```text
small positional shift as final fallback
```

Every correction should be bounded.

For example, expose configuration such as:

```text
MAX_AZIMUTH_CORRECTION_DEG
MAX_TILT_CORRECTION_DEG
MAX_POSITION_SHIFT_M
COLLISION_MARGIN_M
```

Do not allow the collision solver to arbitrarily redesign the plant.

---

# 23. Track source pose and authored pose separately

For any corrected collider, preserve diagnostics such as:

```json
{
  "source_pose": {
    "...": "GroIMP-derived pose"
  },

  "authored_pose": {
    "...": "Isaac-safe pose"
  },

  "adaptation": {
    "reason": "initial collider overlap",
    "azimuth_delta_deg": 3.5,
    "tilt_delta_deg": 0.0,
    "shift_m": 0.0
  }
}
```

This is essential because after the migration the V2 plant may look slightly different for two legitimate reasons:

```text
1. geometry extraction became more accurate

2. physics-safe collision adaptation changed the pose slightly
```

Those effects must never be confused.

---

# 24. Improve the existing collision tests

There is already a useful collision-validation foundation in V2.

The current `test_collision_geometry.py` checks lateral branches, angular separation, trunk relationships, and approximate bounding overlap, but the implementation explicitly uses simplified assumptions such as same-parent checks and a hard-coded approximate distance.

Do not discard it.

Refactor it toward actual authored collider geometry.

Eventually test:

```text
capsule/cylinder vs cylinder

parent-child collider overlap

siblings on same attachment region

adjacent-rank branches

branch vs trunk

petiole vs trunk

rachis vs parent/support structure
```

Ignore intentional joint-contact regions where appropriate.

---

# 25. Reproducibility

Investigate whether GroIMP's random generator can be seeded deterministically for these tests.

The model itself uses stochastic operations such as `random(...)` and `normal(...)` during development.

If a stable GroIMP seed is available, fix it for validation runs.

If not, never compare two independently generated plants as though they were identical ground truth.

Instead:

```text
run GroIMP once
        ↓
extract canonical JSON
        ↓
use SAME JSON for
   old/new exporter comparisons
```

This provides reproducibility even if GroIMP itself cannot be fully deterministic.

---

# 26. Required validation suite

Before replacing the old parsing path, create automated tests covering four levels.

### Level 1 — Extraction correctness

Verify:

```text
node counts
type counts
IDs
attributes
graph edges
array fields
dimensions
```

### Level 2 — Turtle correctness

Verify:

```text
RH
RL
RU
Translate
branch push/pop
local frames
world frames
start/end points
```

### Level 3 — Canonical representation

Verify:

```text
GroIMP → PlantState
PlantState → JSON
JSON → PlantState
```

with round-trip equality within numerical tolerances.

### Level 4 — Exporter regression

For both V1 and later V2, compare:

```text
old output
new output
GroIMP reference
```

and classify differences.

---

# 27. Non-blocking validation rules

Regression tests must distinguish real regressions from intentional improvements.

For example, this must **not** automatically fail:

```text
old branch count = 6
new branch count = 9
```

if GroIMP actually contains nine branches and the old parser intentionally filtered three.

Instead report:

```text
+3 canonical branches restored
EXPECTED
```

Conversely:

```text
GroIMP = 9
PlantState = 8
```

should be considered suspicious.

---

# 28. Suggested quantitative report

Generate a machine-readable comparison report for important test days:

```json
{
  "organ_counts": {},
  "topology": {},
  "geometry": {
    "mean_endpoint_error": 0.0,
    "max_endpoint_error": 0.0,
    "mean_angle_difference": 0.0,
    "max_angle_difference": 0.0
  },
  "physics": {
    "d6_joint_count": 0,
    "collider_count": 0,
    "collision_adjustments": 0,
    "max_angle_correction": 0.0
  },
  "differences": []
}
```

This will be useful both for development and thesis validation.

---

# 29. Migration order

Do the work in this order:

```text
PHASE 1
Freeze current reference outputs

PHASE 2
GroIMP Ground Truth Inspector

PHASE 3
Understand RH/RL/RU/Translate and graph traversal

PHASE 4
Implement + test turtle resolver

PHASE 5
Design canonical PlantState

PHASE 6
GroIMP → PlantState extractor

PHASE 7
PlantState JSON persistence + round-trip tests

PHASE 8
Compare canonical state vs current CSV/exporter

PHASE 9
Migrate V1

PHASE 10
Validate V1 before/after against GroIMP

PHASE 11
Migrate V2

PHASE 12
Restore additional organs progressively

PHASE 13
Apply joint-budget policy

PHASE 14
Collision-safe V2 authoring

PHASE 15
V2 before/after validation

PHASE 16
Remove legacy CSV path once proven safe

PHASE 17
Use canonical extractor in the live GroIMP ↔ Isaac bridge
```

Do not skip directly from current bridge experiments to V2 integration.

---

# 30. Target command architecture

Eventually the offline workflow might become conceptually:

```bash
./run_export_v1.sh --day 10
```

and:

```bash
./run_export_v2.sh --day 10
```

where both load:

```text
output/day_10/plant_state_day_10.json
```

The live workflow should have separate commands, for example:

```bash
./launch_groimp_bridge.sh
```

and later:

```bash
./launch_isaac_bridge.sh
```

In live mode:

```text
GroIMP workbench
    ↓
GroIMPExtractor
    ↓
PlantState in memory
    ↓
V2 adapter
    ↓
Isaac
```

Saving the JSON should be optional in live operation and useful mainly for debugging/replay.

---

# 31. Important architectural invariant

At the end of this work, both of these must be true:

```python
state = load_plant_state("plant_state_day_10.json")
```

and:

```python
state = groimp_extractor.extract(workbench)
```

must return the **same Python domain model**.

Everything downstream must be unaware of where the data came from.

That is the core interface.

---

# 32. Acceptance criteria

This refactor is complete only when:

1. A real GroIMP plant can be extracted through GroPy into `PlantState`.
2. The native GroIMP topology is preserved rather than reconstructed from `rank` whenever native graph information is available.
3. Turtle transformations are resolved by one tested shared implementation.
4. Local and world transforms are available.
5. JSON round-trip preserves the plant state.
6. Offline V1 consumes the JSON/PlantState instead of the old CSV parser.
7. V1 geometry matches GroIMP at least as well as the current implementation.
8. V2 consumes the same PlantState.
9. V2 physics adaptation is clearly separated from source geometry.
10. Restored organs are tracked rather than silently filtered.
11. D6 joint count remains within an acceptable budget after configured simplification.
12. Collider interpenetration is detected before simulation.
13. Necessary collision corrections are minimal and recorded.
14. Differences between old and new exporters are automatically reported.
15. The old CSV pipeline can finally be removed without breaking offline export.

---

# 33. Scope discipline for the implementing LLM

**Do not implement this as one large refactor.**

Each phase must produce a small testable artifact.

Phases A through F and H through I have now been completed. Phase G was
deliberately skipped. Phase J is implemented but its flexible mature-plant
validation is blocked; the next implementation task is:

```text
NEXT TASK:
Resolve the Phase-J flexible day-80/day-160 articulation instability, complete
the mandatory Isaac stress matrix, and only then close Phase J.
```

V2 must preserve all canonical visual organs. Physical filtering or merging is
forbidden at or below 220 predicted articulations, requires warning and review
between 220 and 230, and is mandatory above 230. Every aggregate physical link
must retain references to all represented canonical organ IDs.

---

## Final intended architecture

```text
                     ┌────────────────────┐
                     │      GroIMP        │
                     └─────────┬──────────┘
                               │
                      GroPy / ProjectGraph
                               │
                               ▼
                  ┌────────────────────────┐
                  │ Canonical GroIMP       │
                  │ Extractor              │
                  │                        │
                  │ topology               │
                  │ organ attributes       │
                  │ turtle resolution      │
                  │ world transforms       │
                  └────────────┬───────────┘
                               │
                               ▼
                     ┌──────────────────┐
                     │    PlantState    │
                     └───────┬──────────┘
                             │
                 ┌───────────┴────────────┐
                 │                        │
                 ▼                        ▼
        plant_state_day_N.json       live in-memory
                 │                        │
                 └───────────┬────────────┘
                             │
                ┌────────────┴────────────┐
                ▼                         ▼
        ┌───────────────┐         ┌───────────────┐
        │  Exporter V1  │         │  Exporter V2  │
        │ static geom.  │         │ physics-aware │
        └───────────────┘         └───────┬───────┘
                                          │
                               collision adaptation
                               joint-budget adaptation
                                          │
                                          ▼
                                      Isaac Sim
```

This gives us the clean boundary we were missing: **GroIMP determines what the plant is; the exporters determine how that plant should be represented.**
