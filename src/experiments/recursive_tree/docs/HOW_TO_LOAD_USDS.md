# How to Load Scalability Test USDs in Isaac Sim

Quick guide per caricare i test USD generati in Isaac Sim.

---

## Option 1: Modify load_recursive_tree.py

**File**: `src/experiments/recursive_tree/load_recursive_tree.py`

**Change line ~10:**
```python
# OLD:
usd_path = "src/experiments/recursive_tree/recursive_tree_tomato.usda"

# NEW - Choose one:
usd_path = "src/experiments/recursive_tree/tests/scalability_usds/baseline_tomato_realistic.usda"
usd_path = "src/experiments/recursive_tree/tests/scalability_usds/petiolule_ld_10.usda"
usd_path = "src/experiments/recursive_tree/tests/scalability_usds/petiole_tilt_90.usda"
# ... etc
```

**Run**:
```bash
./run_experiment.sh recursive_tree load_recursive_tree.py
```

---

## Option 2: Isaac Sim GUI

1. Launch Isaac Sim
2. File → Open
3. Navigate to: `~/isaacsim/autotom_digital_twin/src/experiments/recursive_tree/tests/scalability_usds/`
4. Select any `.usda` file
5. Click "Open"

---

## Option 3: Create Loader Script

Create: `src/experiments/recursive_tree/load_scalability_test.py`

```python
#!/usr/bin/env python3
"""Load a scalability test USD in Isaac Sim."""
import sys
from omni.isaac.kit import SimulationApp

# Config
config = {
    "headless": False,
    "width": 1920,
    "height": 1080,
}
simulation_app = SimulationApp(config)

from pxr import Usd, UsdGeom
from omni.isaac.core import World

def main():
    # Choose config (or pass via command line)
    config_name = sys.argv[1] if len(sys.argv) > 1 else "baseline_tomato_realistic"
    
    usd_path = f"src/experiments/recursive_tree/tests/scalability_usds/{config_name}.usda"
    
    print(f"Loading: {usd_path}")
    
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    
    # Load the USD
    stage = world.stage
    tree_prim = stage.DefinePrim(f"/World/tomato_{config_name}", "Xform")
    tree_prim.GetReferences().AddReference(usd_path)
    
    # Enable physics
    world.reset()
    
    print(f"✅ Loaded {config_name}")
    print("Press PLAY to start simulation, ESC to exit")
    
    # Run
    while simulation_app.is_running():
        world.step(render=True)
    
    simulation_app.close()

if __name__ == "__main__":
    main()
```

**Usage**:
```bash
./run_experiment.sh recursive_tree load_scalability_test.py baseline_tomato_realistic
./run_experiment.sh recursive_tree load_scalability_test.py petiole_tilt_90
```

---

## Available Configs

Run `ls tests/scalability_usds/` to see all 15:

```
baseline_tomato_realistic
five_petioles_50_links
min_radius_1mm_world
min_radius_2mm_world
mixed_angles
petiole_ld_10
petiole_tilt_30
petiole_tilt_60
petiole_tilt_90
petiolule_ld_10
petiolule_ld_12
petiolule_ld_8
radius_ratio_2_5
radius_ratio_3_5
six_petioles_50_links
```

---

## What to Look For in Isaac Sim

### Stable Config:
- Plant settles to resting position within ~2s
- No oscillations
- No drift
- Joints remain within reasonable range

### Unstable Config:
- Continuous oscillations (>10s)
- Links "vibrating" or "jittering"
- Gradual drift/rotation
- Eventual collapse or explosion

### Delayed Divergence (user's insight):
- Looks stable for first 10s
- At ~15-30s: starts oscillating
- Indicates numerical instability building up over time

---

## Comparison Guide

**L/D Tests** (compare droop):
```bash
baseline_tomato_realistic  # L/D=5 (baseline)
petiolule_ld_8             # L/D=8 (more droop expected)
petiolule_ld_10            # L/D=10 (significant droop)
petiolule_ld_12            # L/D=12 (likely unstable)
```

**Tilt Tests** (compare horizontal branches):
```bash
petiole_tilt_30   # sin(30°)=0.5 → less droop
baseline_tomato   # sin(45°)=0.71 → medium droop
petiole_tilt_60   # sin(60°)=0.87 → more droop
petiole_tilt_90   # sin(90°)=1.0 → maximum droop (horizontal)
```

**Complexity Tests**:
```bash
baseline_tomato           # 41 links (4 petioles)
six_petioles_50_links     # 59 links (6 petioles) - near PhysX limit
```

---

## Tips

1. **Start with baseline**: Verify it's stable before testing edge cases
2. **Compare side-by-side**: Load 2 configs, run both, compare behavior
3. **Use slow-motion**: Isaac Sim timeline → 0.1× speed to see oscillations clearly
4. **Record metrics**: Position/velocity of petiolule tips over time
5. **Test duration**: Run for 30s minimum (user's insight about delayed divergence)

---

## Next: Task 3

After manually inspecting a few configs, proceed to **automated convergence tests** (Task 3) to systematically classify all 15 configs as STABLE/MARGINAL/UNSTABLE.
