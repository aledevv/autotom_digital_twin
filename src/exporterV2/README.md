# exporterV2 - Recursive Tree Model Exporter

Production-ready tree model generator using recursive branch structures with articulated physics.

## Overview

This exporter generates tree-like plant structures using a recursive branching model. Each branch can attach to any link of its parent branch, enabling complex hierarchical structures. Physics simulation uses Euler-Bernoulli beam theory for realistic flexibility.

## Key Features

- **Recursive branching**: Unlimited hierarchy depth (subject to PhysX limits)
- **Physics-based flexibility**: Spring-damper joints based on material properties
- **Collision filtering**: Automatic filtering to prevent spurious contacts
- **Scalable geometry**: GLOBAL_SCALE parameter for physics stability
- **Isaac Sim integration**: Ready for simulation and visualization

## Quick Start

### Basic Usage

```python
from exporterV2 import build_stage

# Generate with default configuration
stage, stem_path = build_stage("tree.usda")
stage.GetRootLayer().Save()
```

### Custom Configuration

```python
from exporterV2 import build_stage, tree_config

# Modify configuration
tree_config.GLOBAL_SCALE = 5.0
tree_config.BRANCHES = [
    {
        "id": "trunk",
        "parent": None,
        "attach_link": None,
        "n_links": 8,
        "radius": 0.05,
        "height": 0.10,
        "tilt": 0.0,
        "rot": 0.0,
    },
    {
        "id": "branch1",
        "parent": "trunk",
        "attach_link": 5,
        "n_links": 4,
        "radius": 0.02,
        "height": 0.08,
        "tilt": 45.0,
        "rot": 0.0,
    },
]

stage, stem_path = build_stage("custom_tree.usda")
```

### Run in Isaac Sim

**Option 1: Using the launcher script (recommended)**
```bash
./run_exporterV2.sh
```

This generates the USD and loads it in Isaac Sim in one step.

**Option 2: Using main.py directly**
```bash
~/isaacsim/python.sh src/exporterV2/main.py
```

**Option 3: Load only (if USD already generated)**
```bash
~/isaacsim/python.sh -m src.exporterV2.load_tree
```

## BRANCHES Configuration

Each branch is defined by a dictionary:

| Field | Type | Description |
|-------|------|-------------|
| `id` | str | Unique identifier |
| `parent` | str/None | Parent branch ID (None for root) |
| `attach_link` | int/None | 1-based link index on parent (None for root) |
| `n_links` | int | Number of rigid segments |
| `radius` | float | Cylinder radius [m, pre-scale] |
| `height` | float | Cylinder height per link [m, pre-scale] |
| `tilt` | float | Tilt angle from parent Z-axis [deg] |
| `rot` | float | Azimuthal rotation around parent Z [deg] |
| `roll` | float | Roll around branch's own axis [deg] (optional) |

## Physics Parameters

Physics is computed automatically from geometry and material properties:

- **Young's Modulus**: 80 MPa (tomato stem)
- **Damping Ratio**: 0.3 (critical damping)
- **Density**: 1000 kg/m³ (plant tissue)

Spring constant K and damping D are calculated per-link using Euler-Bernoulli beam theory.

## Module Structure

- `tree_config.py` - Configuration and physics helpers
- `generate_tree.py` - USD generation with articulated physics
- `load_tree.py` - Isaac Sim loader with PhysX settings

## Relationship to experiments/recursive_tree

The `experiments/recursive_tree` folder contains the alpha/experimental version of this code. 
It includes extensive tests, documentation, and measurement tools. The exporterV2 is the 
production-ready extraction of the core functionality.

## Validation

To verify physics parameters:

```python
from exporterV2.tree_config import print_tree_summary
print_tree_summary()
```

This prints spring constants, damping coefficients, and natural periods for all branches.
