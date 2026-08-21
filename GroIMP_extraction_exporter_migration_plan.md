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

Still intentionally open: local transform matrices, the shared turtle resolver,
push/pop resolution, and endpoint validation. All phases after Phase A remain
pending.

**Next official task:** Phase B — controlled `RH`/`RL`/`RU`/`Translate` and
branch push/pop experiments, followed by the tested turtle resolver.

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

# 12. Phase E — Build the canonical extractor

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

# 13. Phase F — JSON persistence

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

Phase A has now been completed. The next implementation task must remain narrow:

```text
NEXT TASK:
Build the controlled turtle-semantics fixtures for Phase B.
```

It should use the completed inspector to establish:

```text
Exact multiplication/order convention.
Exact local coordinate axes and signs.
RH/RL/RU and Translate behavior.
Branch push/pop behavior.
Agreement between the resolver and GroIMP location/direction ground truth.
```

Only after those questions are answered should the canonical JSON schema be finalized.

The existing V1/V2 code should initially be treated as **reference material rather than code to rewrite**.

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
