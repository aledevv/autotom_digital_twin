# ExporterV2 Architecture

## Overview

ExporterV2 is a modular tree model generator with **clean separation** between generic tree-building logic and cultivar-specific parsing.

```
┌─────────────────────────────────────────────┐
│           ExporterV2 Pipeline               │
├─────────────────────────────────────────────┤
│                                             │
│  1. CSV Data  →  2. Adapter  →  3. Core    │
│     (groIMP)      (Parser)      (USD Gen)   │
│                                             │
│  ┌──────────┐   ┌──────────┐   ┌────────┐ │
│  │graph_day │──▶│ Profiles │──▶│BRANCHES│ │
│  │_100.csv  │   │ (Tomato) │   │ Format │ │
│  └──────────┘   └──────────┘   └────────┘ │
│                                      │      │
│                                      ▼      │
│                              ┌─────────────┐│
│                              │ USD + PhysX ││
│                              └─────────────┘│
└─────────────────────────────────────────────┘
```

---

## Directory Structure

```
exporterV2/
├── core/                    # Generic tree builder (reusable)
│   ├── tree_config.py      # BRANCHES format & physics
│   ├── physics.py          # PhysX configuration
│   └── usd/                # USD generation
│       ├── stage.py        # Stage orchestration
│       ├── geometry.py     # Cylinder creation
│       ├── joints.py       # Articulation joints
│       └── collision.py    # Collision filtering
│
├── adapters/                # Data source adapters
│   └── groimp_csv/         # groIMP CSV adapter
│       ├── parser.py       # CSV loading & filtering
│       └── leaf_builder.py # Leaf construction
│
├── profiles/                # Cultivar configurations
│   ├── tomato_default.py   # Tomato cultivar
│   └── simple_plant.py     # Example alternative
│
├── docs/                    # Documentation (this folder)
├── tests/                   # Test suite
└── main.py                  # Entry point
```

---

## Core Modules (Generic)

### `core/tree_config.py`
- **BRANCHES format** - Universal tree configuration
- Physics parameters (spring, damping, mass)
- Validation and constraints

### `core/usd/`
- **stage.py** - Build USD stage with articulated physics
- **geometry.py** - Create cylinder links
- **joints.py** - Flexible joints (D6 with springs)
- **collision.py** - Parent-child collision filtering

### `core/physics.py`
- PhysX scene settings for Isaac Sim
- Articulation configuration

---

## Adapters (Data Sources)

### `adapters/groimp_csv/`
Converts groIMP CSV export to generic BRANCHES format.

**Key functions:**
- `parse_csv_to_branches()` - Complete pipeline
- `load_trunk_internodes()` - Trunk data
- `load_lateral_branches()` - Lateral branches (profile-driven)
- `load_leaves()` - Leaves (profile-driven)

**Profile-driven logic:**
- Filtering (opposite pairs, phyllotaxis, etc.)
- Orientation (jitter, collision check)
- Cloning (missing leaves)

---

## Profiles (Cultivar-Specific)

### `profiles/tomato_default.py`

Configuration for standard tomato plant:

```python
TOMATO_PROFILE = {
    "lateral_branches": {
        "organ_indices": [0, 1],      # Opposite pairs
        "tilt_deg": 45.0,
        "rot_base_deg": [0.0, 180.0],
        "rot_jitter_deg": 45.0,       # Random variance
        "min_angle_separation_deg": 60.0,
    },
    "trunk_leaves": {
        "filter_strategy": "opposite_pairs_180deg",
        "phyllotaxis_deg": 137.5,
    },
    "lateral_leaves": {
        "organ_indices": [0, 1],
        "clone_missing": True,
        "tilt_deg": 35.0,
        "rot_range_deg": (-90, 90),
    },
}
```

---

## Data Flow

```
1. CSV File (graph_day_100.csv)
        ↓
2. Adapter (adapters/groimp_csv/parser.py)
   - Load trunk internodes
   - Load lateral branches (filter by profile)
   - Load leaves (filter by profile)
   - Apply collision checks
        ↓
3. BRANCHES Format (universal)
   [
     {"id": "trunk", "parent": None, "n_links": 10, ...},
     {"id": "Branch_r1_o0", "parent": "trunk", ...},
     {"id": "Leaf_r1_o0_petiole", "parent": "trunk", ...},
   ]
        ↓
4. Core (core/usd/stage.py)
   - Generate USD geometry
   - Create articulation joints
   - Apply PhysX settings
        ↓
5. USD Output (tree_v2_day_100.usda)
```

---

## Key Design Principles

1. **Separation of Concerns**
   - Core = generic (any plant)
   - Adapter = data-source specific
   - Profile = cultivar-specific

2. **Modularity**
   - CSV adapter is just one option
   - Can add JSON, database, procedural adapters

3. **Configurability**
   - All cultivar logic in profiles
   - No hardcoded values in core

4. **Testability**
   - Each component tests independently
   - Automated test suite

---

## Extension Points

### Add New Cultivar
```python
# profiles/my_cultivar.py
MY_PROFILE = {
    "lateral_branches": {
        "organ_indices": [0, 1, 2, 3],  # All 4
        "tilt_deg": 60.0,
    },
    # ...
}

# Use it
branches = parse_csv_to_branches(day=100, profile=MY_PROFILE)
```

### Add New Adapter
```python
# adapters/json_adapter/parser.py
def parse_json_to_branches(json_path):
    # Convert JSON → BRANCHES format
    return branches

# Use it
from exporterV2.core.usd import build_stage
stage, stem = build_stage("output.usda", branches=branches)
```

---

## Version History

- **v2.2.0** (Dec 2024) - Architecture refactoring (Phase 1-2)
- **v2.1.0** (Dec 2024) - Lateral branches & leaves
- **v2.0.0** (Dec 2024) - Initial modular refactoring
- **v1.0.0** (Nov 2024) - Monolithic CSV parser

---

**See also:**
- [02_vs_v1.md](02_vs_v1.md) - Comparison with v1
- [03_csv_modifications.md](03_csv_modifications.md) - CSV deviations
- [06_implementation_notes.md](06_implementation_notes.md) - Lessons learned
