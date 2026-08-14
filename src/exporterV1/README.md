# exporterV1 - Legacy Plant Model Exporter

CSV-based plant model exporter for tomato plant simulations from GroIMP.

## Overview

This is the legacy exporter (formerly `plant_model/`) that generates USD plant models from CSV data exported by GroIMP simulations. It creates detailed plant structures including:

- **Internodes**: Cylindrical stem segments
- **Leaves**: Compound leaves with petioles, rachis, and leaflet blades
- **Fruits**: Fruit trusses with individual fruits and pedicels
- **Roots**: Root system visualization

## Status

**Stable/Legacy** - This exporter is preserved for backward compatibility with existing GroIMP-based workflows. For new tree-based models, use `exporterV2`.

## Main Functions

### `load_snapshot(csv_path, day, plant_id)`

Load a plant snapshot from CSV data.

```python
from exporterV1 import load_snapshot

snapshot = load_snapshot("plant_data.csv", day=10, plant_id=1)
```

### `export_plant_usd(snapshot, output_path)`

Export a plant snapshot to USD format.

```python
from exporterV1 import export_plant_usd, load_snapshot

snapshot = load_snapshot("plant_data.csv", day=10, plant_id=1)
export_plant_usd(snapshot, "plant_model.usda")
```

## Module Structure

- `loader.py` - CSV data loading and hierarchy construction
- `models.py` - Data structures (OrganNode, PlantSnapshot, etc.)
- `constants.py` - Physical and geometric parameters
- `usd_exporter.py` - USD generation with physics
- `usd_helpers.py` - USD primitive and transform helpers
- `main.py` - Entry point for standalone usage
- `debug_viz.py` - Visualization utilities
- `graph_export.py` - Graph export utilities

## Dependencies

Requires the following from GroIMP CSV export:
- Organ hierarchy (parent_rank, parent_organ_class)
- Geometric parameters (length, width, area)
- Physiological state (age_dd, dry_biomass_mg)
- Leaf compound structure (segments, blades, inclination)
- Fruit data (radii, ripening state)
