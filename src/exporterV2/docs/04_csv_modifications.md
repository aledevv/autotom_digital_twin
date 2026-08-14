# CSV Modifications

The GroIMP CSV is the source of topology and organ measurements, but V2 adapts it to a stable tomato-plant USD representation. The adapter preserves deterministic ordering while applying tomato-specific selection, completion, and physics metadata.

## Parsing Strategy

`parse_csv_to_branches()` reads the CSV once and passes the DataFrame to the standalone loaders. Public loader calls remain compatible: if called directly without `_dataframe`, they read the CSV themselves.

## Branch and Leaf Adaptation

- trunk internodes become the main `trunk` chain
- lateral branches are selected from configured organ indices
- missing leaf-pair data can be mirrored where the profile requires a paired morphology
- leaf components can be enabled independently: petiole, rachis, petiolules
- random orientation jitter remains deterministic through seeded generation

## Truss Adaptation

Trusses are generated from CSV fruit/truss data and current `TrussGeometryConfig`:

- truss rachis length and link count come from CSV-derived geometry
- pedicels stay one-link V2 pedicels
- tomatoes are generated as terminal sphere metadata with radius, mass, maturation color, and parent pedicel
- the detached tomato body is authored later in USD generation, not as a replacement for CSV structure

## Debug Switches

`OrganGenerationConfig` controls whether complete organ hierarchies are produced:

- `CREATE_LATERAL_BRANCHES`
- `CREATE_LEAF_BRANCHES`, `CREATE_PETIOLES`, `CREATE_LEAF_RACHIS`, `CREATE_PETIOLULES`
- `CREATE_TRUSSES`, `CREATE_TRUSS_RACHIS`, `CREATE_PEDICELS`, `CREATE_TOMATOES`

Disabling a parent disables its descendants in the parser output.
