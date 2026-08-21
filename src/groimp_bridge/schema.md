# GroIMP inspection report schema

`groimp_inspection/1.0` is a versioned diagnostic report. It deliberately keeps
GroIMP type and field names and must not be used as the canonical `PlantState`.

```json
{
  "report_schema_version": "groimp_inspection/1.0",
  "metadata": {
    "source_project": "/absolute/path/project_bridge.gsz",
    "api_url": "http://localhost:58081/api/",
    "function_name": "Dynamic_Model",
    "steps_requested": 1,
    "steps_completed": 1,
    "simulation_time": 1,
    "captured_at_utc": "..."
  },
  "steps": [
    {
      "step": 1,
      "function_name": "Dynamic_Model",
      "console": [],
      "logs": []
    }
  ],
  "snapshot": {
    "root_id": 0,
    "nodes": [
      {
        "id": 421092,
        "type": "organs.Internode",
        "attributes": {
          "length": 0.008802679,
          "rank": 0
        },
        "world_anchor": {
          "position": [0.6, 0.6, 0.065],
          "direction": [0.0, 0.0, 1.0]
        }
      }
    ],
    "edges": [
      {
        "source": 421091,
        "target": 421092,
        "kind": "successor",
        "raw_code": 256
      }
    ],
    "counts_by_type": {},
    "diagnostics": {}
  },
  "diagnostics": {
    "isolation": "temporary_project_copy",
    "source_project_modified": false,
    "queried_types": [],
    "unenriched_node_types": [],
    "missing_optional_fields": [],
    "unknown_edge_codes": []
  }
}
```

Edge code `256` is currently classified as `successor` and `512` as `branch`.
Every code is retained in `raw_code`; unrecognized codes use `kind: "unknown"`.

Nullable GroIMP arrays are serialized as JSON `null`, distinct from initialized
empty arrays (`[]`). Nodes not covered by the enrichment registry still appear
with empty `attributes` and are listed under `unenriched_node_types`.

The Phase B `TurtleResolution` is deliberately not embedded into this report,
so `groimp_inspection/1.0` remains the raw diagnostic boundary. Call
`resolve_turtle(report.snapshot)` to obtain local-to-world matrices, incoming
and outgoing node frames, endpoints, traversal order, and resolver diagnostics.

## Phase C geometry report

`groimp_geometry_validation/1.0` is separate from the raw inspection schema.
It contains one check per selected reconstructed axis or sphere, with expected
identity, measured OBJ geometry, absolute errors, and one of `passed`,
`ambiguous`, `failed`, or `not_recoverable`. Diagnostics record the public
tolerances, OBJ axis mapping, selected nodes, and translation-only
renderer-cache offsets.

## Phase D comparison report

`groimp_migration_comparison/1.0` compares native biological organs, graph CSV,
actual V1 USD primitives, and V2 branches/terminal bodies. It stores counts,
deterministic semantic matches, parent topology, per-field maximum errors,
world endpoint/orientation differences, and one classification per difference:

```text
EXPECTED_IMPROVEMENT
EXPECTED_SIMPLIFICATION
PHYSICS_ADAPTATION
UNKNOWN_DIFFERENCE
LIKELY_BUG
```

Its status becomes `investigation_required` whenever an unknown or likely bug
remains; explained legacy and physics adaptations are non-blocking.

## Canonical PlantState

`plant_state/1.0` is a separate domain schema, not a revision of any diagnostic
report above. It retains a single selected plant subtree, typed organs, turtle
operations, local/world poses, and canonical axis/sphere primitives. It can be
loaded without GroIMP. The full contract and provenance rules are documented
in `src/plant_state/README.md`.
