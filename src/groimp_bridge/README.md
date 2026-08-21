# GroIMP Inspector and Turtle Resolver

The inspector reads the native GroIMP ProjectGraph before any USD- or
PhysX-specific adaptation. The shared turtle resolver reconstructs full world
frames from that snapshot. These are the completed first two migration
milestones; the
JSON report is diagnostic data and is **not** the future canonical
`PlantState` format.

## Run

Start GroIMP with GroLink enabled on port `58081`, then run:

```bash
uv run python -m groimp_bridge.inspector \
  --project model/project_bridge.gsz \
  --steps 1 \
  --output /tmp/groimp_inspection_day_1.json
```

The path-based command copies the GSZ and `model/input/` to an isolated
temporary directory. `Dynamic_Model` therefore writes only under `/tmp`; the
source project and its existing CSV outputs are not changed.

The public Python boundary is:

```python
from groimp_bridge import inspect_project, inspect_workbench, resolve_turtle

report = inspect_project("model/project_bridge.gsz", steps=1)
snapshot = inspect_workbench(an_already_open_gropy_workbench)
resolution = resolve_turtle(report.snapshot)
internode_pose = resolution.poses[421092]
world_matrix = internode_pose.incoming_frame.matrix
```

`inspect_workbench` neither advances nor closes the supplied workbench. This is
the entry point intended for the later live bridge.

## Data captured

The `groimp_inspection/1.0` report stores the complete node/edge graph, raw edge
codes, typed organ and turtle attributes, GroIMP `location()`/`direction()`
anchors, RGG console output, type counts, and extraction diagnostics. Supported
enrichment currently covers:

- `Root`, `Internode`, `Leaf`, `Truss`, `Fruits`, `Meristem`, and `PlantBase`;
- `RH`, `RL`, `RU`, `RG`, and `Translate`;
- scalar, boolean, nullable `float[]`, and direct world-anchor values.

Unknown node and edge types remain in the report instead of being discarded.
See `schema.md` for the wire shape.

## Turtle convention

`TurtleFrame` stores a local-to-world 4x4 matrix for column vectors. The first
three columns are GroIMP local X/left, Y/up, and Z/head; translation is the
fourth column. Local operations are post-multiplied (`world @ local`) and
angles are degrees:

- `RH` is a right-handed local-Z/head rotation;
- `RL` is a right-handed local-X/left rotation;
- `RU` is a right-handed local-Y/up rotation;
- `Translate(x, y, z)` translates along the current local axes;
- `RG` minimally aligns head with world negative Z, preserving roll;
- `Internode` advances along local head.

Successor and branch children both inherit the parent's outgoing frame. Each
branch is resolved from its own copy, so nested `[...]` scopes cannot alter a
sibling or the main successor path. Graph traversal order is therefore not a
source of turtle state; edges encode the required scopes.

The tomato RGG updates biological `Internode.length` during a model step. At a
mature step GroIMP can still render the preceding effective M-step. When native
world anchors are available, the resolver derives that effective axial advance
from the internode and its successor and reports the maximum discrepancy in
`TurtleResolution.diagnostics`. Snapshots without anchors use `length` as the
deterministic fallback.

## Tests

Offline tests require no GroIMP process:

```bash
uv run pytest \
  src/groimp_bridge/tests/test_offline_inspector.py \
  src/groimp_bridge/tests/test_offline_turtle.py -q
```

Controlled live turtle fixtures are opt-in:

```bash
RUN_GROIMP_TESTS=1 \
uv run pytest src/groimp_bridge/tests/test_live_turtle_fixture.py -q
```

The real-plant day-1 test is:

```bash
RUN_GROIMP_TESTS=1 \
uv run pytest src/groimp_bridge/tests/test_live_inspector.py \
  -m "groimp and not slow" -q
```

The optional day-25 and full mature day-80 tests are:

```bash
RUN_GROIMP_TESTS=1 RUN_GROIMP_SLOW_TESTS=1 \
uv run pytest src/groimp_bridge/tests/test_live_inspector.py \
  -m "groimp and slow" -q
```

The day-80 case currently validates 301 nodes, 300 edges, 121 branch edges,
26 internodes, 27 leaves, 9 trusses, and 9 fruit modules. All 301 nodes resolve,
and all 285 nodes carrying native GroIMP anchors are compared within an
absolute tolerance of `1e-6` for position and head direction.

When live tests are not enabled, or the local GroIMP server cannot be reached,
they skip with an explicit reason.

## Deliberate limitations

The resolver establishes organ-level turtle anchors and internode endpoints. It
does not yet validate rendered leaflets, fruit components, or meshes against a
GroIMP scene export; that belongs to migration Phase C. `PlantState`, Exporter
V1/V2, their CSV adapters, and Isaac Sim integration remain unchanged.
