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
