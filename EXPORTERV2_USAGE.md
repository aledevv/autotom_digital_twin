# exporterV2 Usage Guide

Quick reference for using the new tree model exporter.

## Quick Start

### Launch with default configuration

```bash
./run_exporterV2.sh
```

This will:
1. Generate the tree USD from `tree_config.BRANCHES`
2. Apply PhysX settings
3. Open Isaac Sim
4. Start the simulation

**Output:** `/data/usd_models/tree_v2.usda`

---

## Custom Configuration

### Method 1: Modify tree_config.py directly

Edit `src/exporterV2/tree_config.py`:

```python
GLOBAL_SCALE = 3.0  # Adjust scale

BRANCHES = [
    {
        "id": "trunk",
        "parent": None,
        "attach_link": None,
        "n_links": 10,
        "radius": 0.08,
        "height": 0.15,
        "tilt": 0.0,
        "rot": 0.0,
    },
    # Add more branches...
]
```

Then run:
```bash
./run_exporterV2.sh
```

### Method 2: Use example_custom_tree.py

```bash
~/isaacsim/python.sh src/exporterV2/example_custom_tree.py
```

This shows how to programmatically configure before running.

### Method 3: Python API

```python
from exporterV2 import build_stage, tree_config

# Configure
tree_config.GLOBAL_SCALE = 5.0
tree_config.BRANCHES = [...]

# Generate
stage, stem_path = build_stage("my_tree.usda")
stage.GetRootLayer().Save()
```

---

## Configuration Parameters

### BRANCHES Format

Each branch is a dictionary with these fields:

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `id` | str | ✓ | Unique identifier |
| `parent` | str/None | ✓ | Parent branch ID (None for root) |
| `attach_link` | int/None | ✓ | 1-based link index on parent |
| `n_links` | int | ✓ | Number of rigid segments |
| `radius` | float | ✓ | Cylinder radius [m, pre-scale] |
| `height` | float | ✓ | Cylinder height per link [m, pre-scale] |
| `tilt` | float | ✓ | Tilt angle from parent Z [deg] |
| `rot` | float | ✓ | Azimuthal rotation [deg] |
| `roll` | float | ✗ | Roll around branch axis [deg] |

### Global Parameters

```python
GLOBAL_SCALE = 2.0        # Geometry scale multiplier
BEND_LIMIT_DEG = 30.0     # Joint angle limit
GAP = 0.001               # Gap between links [m, pre-scale]

# Physics (BioConfig class)
YOUNG_MODULUS = 80.0e6    # [Pa] Material stiffness
DAMPING_RATIO = 0.3       # Dimensionless damping
PLANT_DENSITY = 1000.0    # [kg/m³] Tissue density
```

---

## Examples

### Example 1: Simple vertical trunk

```python
BRANCHES = [
    {
        "id": "trunk",
        "parent": None,
        "attach_link": None,
        "n_links": 10,
        "radius": 0.10,
        "height": 0.20,
        "tilt": 0.0,
        "rot": 0.0,
    },
]
```

### Example 2: Trunk with lateral branches

```python
BRANCHES = [
    {
        "id": "trunk",
        "parent": None,
        "attach_link": None,
        "n_links": 8,
        "radius": 0.08,
        "height": 0.15,
        "tilt": 0.0,
        "rot": 0.0,
    },
    {
        "id": "branch_left",
        "parent": "trunk",
        "attach_link": 5,      # Attach to 5th link of trunk
        "n_links": 4,
        "radius": 0.04,
        "height": 0.12,
        "tilt": 45.0,          # 45° from vertical
        "rot": 90.0,           # Point to the left
    },
    {
        "id": "branch_right",
        "parent": "trunk",
        "attach_link": 5,      # Same attachment point
        "n_links": 4,
        "radius": 0.04,
        "height": 0.12,
        "tilt": 45.0,
        "rot": 270.0,          # Point to the right
    },
]
```

### Example 3: Recursive sub-branches

```python
BRANCHES = [
    {
        "id": "trunk",
        "parent": None,
        "attach_link": None,
        "n_links": 8,
        "radius": 0.10,
        "height": 0.20,
        "tilt": 0.0,
        "rot": 0.0,
    },
    {
        "id": "main_branch",
        "parent": "trunk",
        "attach_link": 5,
        "n_links": 6,
        "radius": 0.05,
        "height": 0.15,
        "tilt": 40.0,
        "rot": 0.0,
    },
    {
        "id": "sub_branch",
        "parent": "main_branch",
        "attach_link": 3,      # Attach to main_branch
        "n_links": 3,
        "radius": 0.02,
        "height": 0.10,
        "tilt": 35.0,
        "rot": 60.0,
    },
]
```

---

## Validation

Check your configuration before running:

```bash
cd src/exporterV2
python tree_config.py
```

This prints a physics summary table showing:
- Link counts per branch
- Mass, stiffness, damping for each branch
- Natural periods
- Total link count vs. PhysX limit

---

## Troubleshooting

### Error: "Total link count exceeds PhysX limit"

Reduce `n_links` in some branches or split into smaller sub-branches.

### Error: "Unknown parent 'xyz'"

Check that:
1. Parent branch `id` is spelled correctly
2. Parent branch is defined BEFORE child branch in the list

### Branches look disconnected

Check `attach_link` value:
- Must be between 1 and parent's `n_links`
- 1 = bottom link, n_links = top link

### Physics instability

Try:
- Increase `GLOBAL_SCALE` (2.0 → 5.0)
- Reduce `n_links` per branch
- Check that child radius < parent radius

---

## Files Created

When you run exporterV2, these files are created:

- `/data/usd_models/tree_v2.usda` - Main USD scene

To change the output filename, modify `get_output_usd_path()` in `generate_tree.py`.

---

## Advanced: Locked Joints (for testing)

Generate a rigid tree (no flexibility):

```python
from exporterV2 import build_stage_locked

stage, stem_path = build_stage_locked("rigid_tree.usda")
stage.GetRootLayer().Save()
```

This creates FixedJoints instead of flexible D6 joints, useful for geometry verification.

---

## See Also

- `src/exporterV2/README.md` - Full module documentation
- `RESTRUCTURING_SUMMARY.md` - Overview of exporter organization
- `src/experiments/recursive_tree/` - Alpha version with tests
