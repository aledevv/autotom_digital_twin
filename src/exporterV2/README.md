# ExporterV2 - Modular Tree Model Generator

Production-ready tree model generator with **clean separation** between generic tree building and cultivar-specific logic.

## Architecture

```
exporterV2/
├── core/              # Generic tree builder (reusable for any plant)
├── adapters/          # Data source adapters (CSV, manual, etc.)
├── profiles/          # Cultivar-specific configurations
└── main.py            # Entry point
```

---

## Quick Start

### CSV Mode (with groIMP data)
```bash
./run_mainV2.sh --day 100
```

### Python API
```python
from exporterV2.adapters.groimp_csv import parse_csv_to_branches
from exporterV2.core.usd import build_stage

# Load from CSV (uses tomato profile by default)
branches, json_path = parse_csv_to_branches(day=100)

# Generate USD
stage, stem_path = build_stage("output.usda", branches=branches)
```

### Manual Configuration
```python
from exporterV2.core import tree_config
from exporterV2.core.usd import build_stage

# Define custom tree
tree_config.BRANCHES = [
    {"id": "trunk", "parent": None, "n_links": 10, ...},
    {"id": "branch_1", "parent": "trunk", ...},
]

# Generate USD
stage, stem_path = build_stage("output.usda")
```

---

## Core Modules (Generic)

### `core/tree_config.py`
- `BRANCHES` - Tree configuration format
- `GLOBAL_SCALE` - World-space scaling
- `validate_branches()` - Configuration validation
- `clamp_radius()` - PhysX stability constraints

### `core/usd/`
- `build_stage()` - USD stage generation
- Articulated physics with flexible joints
- Automatic collision filtering

### `core/physics.py`
- PhysX scene settings for Isaac Sim
- Articulation configuration

---

## Adapters (Data Sources)

### `adapters/groimp_csv/`
Parses groIMP CSV export files and converts to generic BRANCHES format.

**Functions:**
- `parse_csv_to_branches(day, plant_id, profile)` - Complete pipeline
- `load_trunk_internodes()` - Load trunk data
- `load_lateral_branches()` - Load lateral branches with filtering
- `load_leaves()` - Load leaves with filtering

**Profile-driven:**
- Filtering logic controlled by cultivar profile
- Default: tomato profile with opposite pair filtering

---

## Profiles (Cultivar-Specific)

### `profiles/tomato_default.py`

Configuration for standard tomato plant:
- Lateral branches: opposite pairs (organ_index 0+1), 45° tilt
- Trunk leaves: 180° opposite pair filtering
- Lateral leaves: clone missing, random orientation

**Create your own profile:**
```python
MY_PROFILE = {
    "lateral_branches": {
        "organ_indices": [0, 1, 2, 3],  # All 4 branches
        "tilt_deg": 60.0,                # Different tilt
    },
    # ...
}

branches, _ = parse_csv_to_branches(day=100, profile=MY_PROFILE)
```

---

## Output

### JSON Configuration
```json
{
  "metadata": {
    "day": 100,
    "n_branches": 108,
    "total_links": 133,
    "profile": "Tomato Default"
  },
  "branches": [...]
}
```

### USD Stage
- Articulated physics (PhysX)
- Flexible joints with automatic spring/damping
- Collision filtering (no self-collision)
- Compatible with Isaac Sim

---

## Testing

```bash
# Test with different days
./run_mainV2.sh --day 1
./run_mainV2.sh --day 50
./run_mainV2.sh --day 160

# Check output
ls output/day_100/
cat output/day_100/branches_v2_day_100.json
```

---

## Refactoring History

**Phase 1-2 (August 2026):**
- ✅ Restructured directories (core/adapters/profiles)
- ✅ Extracted cultivar configuration to profiles
- ✅ Preserved all existing functionality

See `REFACTORING_SUMMARY.md` for details.

---

## Related

- **exporterV1:** Original CSV parser (deprecated)
- **recursive_tree:** Generic tree experiments (uses core/ directly)
- **example_custom_tree.py:** Manual BRANCHES configuration example

---

## Maintainer
Alessandro - Digital Twin Project
