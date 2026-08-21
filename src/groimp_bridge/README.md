# GroIMP Ground Truth Inspector

The inspector reads the native GroIMP ProjectGraph before any USD- or
PhysX-specific adaptation. It is the completed first migration milestone; its
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
from groimp_bridge import inspect_project, inspect_workbench

report = inspect_project("model/project_bridge.gsz", steps=1)
snapshot = inspect_workbench(an_already_open_gropy_workbench)
```

`inspect_workbench` neither advances nor closes the supplied workbench. This is
the entry point intended for the later live bridge.

## Data captured

The `groimp_inspection/1.0` report stores the complete node/edge graph, raw edge
codes, typed organ and turtle attributes, GroIMP `location()`/`direction()`
anchors, RGG console output, type counts, and extraction diagnostics. Supported
enrichment currently covers:

- `Root`, `Internode`, `Leaf`, `Truss`, `Fruits`, `Meristem`, and `PlantBase`;
- `RH`, `RL`, `RU`, and `Translate`;
- scalar, boolean, nullable `float[]`, and direct world-anchor values.

Unknown node and edge types remain in the report instead of being discarded.
See `schema.md` for the wire shape.

## Tests

Offline tests require no GroIMP process:

```bash
uv run pytest src/groimp_bridge/tests/test_offline_inspector.py -q
```

The live day-1 test is opt-in:

```bash
RUN_GROIMP_TESTS=1 \
uv run pytest src/groimp_bridge/tests/test_live_inspector.py \
  -m "groimp and not slow" -q
```

The optional day-25 reproductive-organ test is:

```bash
RUN_GROIMP_TESTS=1 RUN_GROIMP_SLOW_TESTS=1 \
uv run pytest src/groimp_bridge/tests/test_live_inspector.py \
  -m "groimp and slow" -q
```

When live tests are not enabled, or the local GroIMP server cannot be reached,
they skip with an explicit reason.

## Deliberate limitations

The inspector records GroIMP's resolved position and head direction but does
not yet calculate full local/world matrices, resolve turtle operations, or
validate organ endpoints. Those belong to migration Phase B. Exporter V1/V2
and their CSV adapters are unchanged by this milestone.
