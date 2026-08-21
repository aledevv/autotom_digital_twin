# Canonical PlantState schema

`plant_state/1.0` is the exporter-independent representation of one plant. It
contains the selected native topology, typed biological organs, turtle
operations, resolved local/world transforms, and the validated supporting
geometry needed to replay the plant without GroIMP.

The package deliberately imports neither `groimp_bridge` nor ExporterV1/V2.
GroIMP adaptation lives in `groimp_bridge.extractor`; JSON loading is therefore
available in a serverless Python process:

```python
from plant_state import load_plant_state

state = load_plant_state("plant_state_day_80.json")
```

## Wire shape

```json
{
  "schema_version": "plant_state/1.0",
  "metadata": {
    "simulation_time": 80,
    "plant_id": 1,
    "source": "groimp_api",
    "source_model": "project_bridge.gsz",
    "source_project_sha256": "...",
    "units": {
      "angle": "degree",
      "area": "m2",
      "dry_biomass": "mg",
      "length": "m"
    },
    "conventions": {
      "transform_semantics": "local_to_world_column_vectors"
    }
  },
  "root_node_id": "node:421073",
  "nodes": [],
  "edges": [],
  "organs": [],
  "turtle_operations": [],
  "axes": [],
  "spheres": [],
  "diagnostics": {}
}
```

Nodes retain `groimp_node_id`, the original type and attributes, their native
parent edge, incoming/outgoing world frames, and local node effect. Organ
records normalize common biology and use typed properties for `PlantBase`,
`Root`, `Internode`, `Leaf`, `Truss`, `Fruits`, and `Meristem`.

Axis and sphere primitives retain both owner-local and world frames/landmarks.
Their provenance distinguishes direct GroIMP turtle geometry, anchor-calibrated
internode advances, and validated RGG production reconstruction. Renderer-cache
translations found during OBJ validation are diagnostic only and are never
applied to canonical truth.

IDs are stable within one GroIMP workbench. The schema intentionally makes no
cross-run identity guarantee because native GroIMP IDs can change when an
independent simulation is rebuilt.

## Persistence and validation

```python
from plant_state import (
    load_plant_state,
    plant_states_equivalent,
    save_plant_state,
    validate_plant_state,
)

validate_plant_state(state)
save_plant_state(state, "/tmp/plant_state.json")
replayed = load_plant_state("/tmp/plant_state.json")
assert plant_states_equivalent(state, replayed)
```

Serialization is strict and deterministic: keys are sorted, non-finite numbers
are rejected, the document ends with one newline, and loaders reject other
schema versions or unknown structural fields. Validation covers topology,
references, matrices, local/world consistency, dimensions, organ arrays, and
supported-organ geometry coverage.

Leaflet surface assets are not canonicalized in version 1.0. Their validated
petiole, rachis and petiolule axes are retained; no synthetic blade geometry is
invented.
