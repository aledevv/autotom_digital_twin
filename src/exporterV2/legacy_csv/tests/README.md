# Truss Tests and Demos

Tests and demonstrations for truss (tomato cluster) implementation in exporterV2.

## Overview

The truss feature implements tomato clusters with:
- **Rachis**: Articulated main stem
- **Pedicels**: Lateral branches alternating ±90°, plus coaxial terminal
- **Tomatoes**: Spheres attached with FixedJoint (excluded from articulation)
- **Physics**: Euler-Bernoulli beam theory for spring-damper joints

## Unit Tests

### test_truss_builder.py

Comprehensive unit tests for truss_builder functions (no pxr required).

**Run with:**
```bash
cd src/exporterV2/adapters/groimp_csv/tests
python test_truss_builder.py
```

**Tests (7 total):**
1. Rachis generation with phyllotaxis orientation
2. Lateral pedicel pairs (alternating ±90°)
3. Terminal pedicel (coaxial)
4. Complete truss configuration
5. Radius clamping (MIN_LINK_RADIUS_WORLD)
6. Tomato definitions with mass calculation
7. Complete config (branches + tomatoes)

**Expected:** All tests pass ✓

---

## Visual Demos (Isaac Sim)

### demo_complete_truss.py - Locked Joints Demo

Complete truss with **locked joints** (rigid, no flexibility).
Good for initial geometry validation.

**Generate USD:**
```bash
uv run python src/exporterV2/adapters/groimp_csv/tests/demo_complete_truss.py
```

**Output:** `data/usd_models/truss_complete_demo.usda`

**Structure:**
- Trunk: 5 links (vertical)
- Rachis: 4 links (60° droop)
- Pedicels: 6 lateral + 1 terminal
- Tomatoes: 7 spheres (varying sizes, mix ripe/unripe)
- Joints: All FixedJoint (completely rigid)

**Load in Isaac Sim:**
```bash
cd src/exporterV2/adapters/groimp_csv/tests
~/isaacsim/python.sh load_truss_demo.py
```

**Expected behavior:**
- Structure remains completely rigid
- No bending or flexing
- Tomatoes stay attached
- Good for verifying geometry correctness

---

### demo_flexible_truss.py - Flexible Joints Demo

Complete truss with **flexible D6 joints** (spring-damper physics).
Tests realistic physics simulation.

**Generate USD:**
```bash
uv run python src/exporterV2/adapters/groimp_csv/tests/demo_flexible_truss.py
```

**Output:** `data/usd_models/truss_flexible_demo.usda`

**Structure:**
- Same as locked demo
- Joints: D6 with spring-damper (flexible)
- Physics: Euler-Bernoulli beam theory

**Load in Isaac Sim:**
```bash
~/isaacsim/python.sh load_truss_demo.py  # Use same loader
```

**Expected behavior:**
- Trunk mostly vertical with slight sway
- Rachis droops naturally under weight
- Pedicels bend under tomato mass
- Smooth spring-damper motion
- No violent oscillations or explosions

**Stability tuning:**
If simulation is unstable:
1. Increase `DAMPING_RATIO` in `tree_config.py` (0.1 → 0.2)
2. Decrease `GLOBAL_SCALE` (heavier → more stable)
3. Increase `MIN_LINK_RADIUS_WORLD` (thicker → stiffer)

---

## Physics Parameters

### Automatic Calculation

All physics parameters are calculated automatically from geometry using Euler-Bernoulli beam theory:

```
I = π r⁴ / 4              [m⁴]       Second moment of area
K = E * I / L             [N·m/rad]  Spring constant
J = moment of inertia     [kg·m²]    Rotational inertia
D = 2ζ√(K·J)             [N·m·s/rad] Damping coefficient
```

**Constants (tree_config.py):**
- `YOUNG_MODULUS`: 50 MPa (mature tomato stem)
- `DAMPING_RATIO`: 0.1 (ζ, dimensionless)
- `PLANT_DENSITY`: 1000 kg/m³
- `TOMATO_DENSITY`: 1000 kg/m³
- `GLOBAL_SCALE`: 2.0 (all dimensions × 2)
- `MIN_LINK_RADIUS_WORLD`: 2mm (PhysX stability limit)

### Typical Values (GLOBAL_SCALE=2.0)

**Trunk (r=1cm → 2cm):**
- K ≈ 400-800 N·m/rad
- D ≈ 10-20 N·m·s/rad
- Mass ≈ 2.5 kg/link

**Rachis (r=2mm → 4mm):**
- K ≈ 1-5 N·m/rad
- D ≈ 0.1-0.5 N·m·s/rad
- Mass ≈ 0.01-0.02 kg/link

**Pedicel (r=1.2mm → 2.4mm):**
- K ≈ 0.3-1 N·m/rad
- D ≈ 0.05-0.2 N·m·s/rad
- Mass ≈ 0.002-0.005 kg/link

**Tomato (r=2.5-3.5cm → 5-7cm):**
- Mass ≈ 0.065-0.180 kg
- Fixed joint (no K/D)

---

## Collision Filtering

### Automatic Filtering

Collision filtering is applied automatically via `add_sibling_collision_filtering()`:

1. **Sibling pedicels**: All pedicels attached to same rachis link filter each other
2. **Parent-child**: Each branch filters with parent and parent's neighbor
3. **Tomato-pedicel**: Each tomato filters with its parent pedicel

### Geometry Validation

Pre-simulation checks for intersecting geometry:

```python
validate_truss_geometry(tomatoes, branch_registry, branches, margin=0.001)
```

**Checks:**
- Tomato-tomato overlaps (sphere-sphere)
- Tomato-rachis overlaps (sphere-cylinder)
- 1mm safety margin by default

**Warnings** indicate potential PhysX instability. Adjust truss parameters to eliminate overlaps.

---

## Integration with CSV Parser

### Future Work (Task 8)

The truss builder is ready for CSV integration:

```python
from exporterV2.adapters.groimp_csv.truss_builder import truss_to_complete_config

# From CSV FruitsNode data
truss_dict = {
    "rachis_length": fruits_node.rachis_length,
    "rachis_radius": fruits_node.rachis_radius,
    "n_fruits": fruits_node.fruit_nr,
    "tomato_radii": fruits_node.fruit_radii,
    "maturation": fruits_node.fruit_age_dd / fruits_node.ripening_dd,
    # ... other parameters
}

branches, tomatoes = truss_to_complete_config(
    truss_dict,
    parent_trunk_id="trunk",
    rank=fruits_node.rank
)
```

---

## Troubleshooting

### Simulation explodes on PLAY

**Cause:** PhysX numerical instability

**Solutions:**
1. Check geometry validation warnings
2. Increase `GLOBAL_SCALE` (makes everything heavier)
3. Increase `MIN_LINK_RADIUS_WORLD` (thicker links)
4. Increase `DAMPING_RATIO` (more energy dissipation)
5. Check for pre-intersecting geometry

### Tomatoes fall off pedicels

**Cause:** FixedJoint not applied correctly

**Check:**
- Physics Inspector shows FixedJoint for each tomato
- Body0 = pedicel, Body1 = tomato
- LocalPos0 at pedicel tip
- LocalPos1 with correct offset

### Structure too stiff / too floppy

**Tune:**
- `YOUNG_MODULUS`: Higher = stiffer (20-100 MPa range)
- `DAMPING_RATIO`: Higher = less oscillation (0.05-0.3 range)
- Link dimensions: Thicker = stiffer, longer = more flexible

### Pedicels collide with each other

**Expected:** Sibling filtering should prevent this

**Check:**
- `add_sibling_collision_filtering()` called in stage builder
- Pedicels attach to same rachis link
- FilteredPairsAPI applied in Physics Inspector

---

## File Structure

```
adapters/groimp_csv/
├── truss_builder.py              # Truss branch generation
└── tests/
    ├── README.md                 # This file
    ├── test_truss_builder.py     # Unit tests (7 tests)
    ├── demo_complete_truss.py    # Locked joints demo
    ├── demo_flexible_truss.py    # Flexible joints demo
    └── load_truss_demo.py        # Isaac Sim loader
```

---

## References

- **Euler-Bernoulli Beam Theory**: [Wikipedia](https://en.wikipedia.org/wiki/Euler%E2%80%93Bernoulli_beam_theory)
- **PhysX Articulation**: Isaac Sim documentation
- **exporterV2 Design**: See `/src/exporterV2/README.md`
