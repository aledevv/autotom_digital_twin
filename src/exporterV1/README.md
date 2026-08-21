# ExporterV1 — static PlantState renderer

ExporterV1 consumes only canonical `plant_state/1.0` JSON. It preserves the
existing V1 visual language for stems, compound leaves, trusses and fruits,
while obtaining identity, placement and parentage from the canonical graph.
It never contacts GroIMP and has no CSV fallback.

Generate a stage without Isaac Sim:

```bash
uv run python -m exporterV1 --day 25
```

The default input and output are:

```text
data/plant_states/plant_state_day_25.json
data/usd_models/tree_v1_day_25.usda
data/usd_models/tree_v1_day_25.manifest.json
```

Paths and plant identity can be overridden:

```bash
uv run python -m exporterV1 \
  --day 25 --plant-id 1 \
  --input /tmp/plant.json \
  --output /tmp/plant.usda
```

Open the generated static stage interactively in Isaac Sim:

```bash
./run_mainV1.sh --day 25 --isaacsim
```

`run_mainV1.sh` generates only by default. Add `--isaacsim` to open the stage.
It also accepts `--plant-id`, `--input`, `--output`, `--generate-only` and
`--headless`. Headless mode requires `--isaacsim`, opens the stage for a short
smoke test and exits; GUI mode remains interactive until the window closes.

If the canonical JSON is missing, prepare it explicitly while GroIMP is
available:

```bash
uv run python -m groimp_bridge.extractor \
  --project model/project_bridge.gsz \
  --steps 25 --plant-id 1 \
  --output data/plant_states/plant_state_day_25.json
```

## Completeness contract

Every canonical organ receives a uniquely named USD organ prim based on its
node ID. Every canonical node also receives a topology prim carrying its
parent node and incoming edge kind. Internodes, leaves and `Fruits` modules
receive their normal V1 visuals. `Truss`, `Meristem` and `PlantBase` remain
visible in metadata/topology but receive no invented geometry.

Each export writes an `exporter_v1_manifest/1.0` sidecar. The manifest checks
canonical versus USD organ counts, expected versus created geometry, topology
parentage, node coverage and path uniqueness. V1 never filters canonical organ
or topology prims. If two visible organs have exactly the same type, world pose
and visual geometry, only the duplicate visual geometry is suppressed; the
second organ prim remains and records `autotom:geometryDuplicateOf`. Near
overlaps and intersections are never collapsed.

The checkpoint commit `d7f5038` is the last working legacy CSV pipeline.
