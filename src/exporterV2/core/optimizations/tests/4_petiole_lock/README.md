# Task 4: Petiole Lock Tests

Tests for Petiole Lock optimization technique (Task 4).

**Status**: ✅ DONE - Task 4 completed

## What Task 4 Implements

**Petiole Lock**: Converts petiolule joints from D6 (articulated, 6 DOF) to Fixed (static, 0 DOF).

### Benefits
- **Reduces DOF**: Each petiolule goes from 6 DOF → 0 DOF
- **No visual change**: Geometry remains identical
- **Minimal impact**: Petiolules are small terminal branches
- **Priority 1**: Applied first (highest priority)

### How It Works
1. Identifies petiolules by naming pattern (`Petiolule_*`) or by parent (Rachis)
2. Adds `joint_type: "fixed"` metadata to branch config
3. Preserves all geometry (n_links, height, radius, attachments)
4. USD builder reads metadata and creates FixedJoint instead of D6Joint

## Files

### Core Module
- **`techniques/petiole_lock.py`**: Petiole Lock technique implementation
  - `PetioleLockTechnique` class extending `OptimizationTechnique`
  - Identifies petiolules
  - Estimates DOF reduction
  - Applies joint_type metadata
  - Validates topology preservation

### Integration
- **`core/usd/stage.py`**: Modified to read `joint_type` metadata
  - Checks `branch_def.get("joint_type")` 
  - Creates FixedJoint if `joint_type == "fixed"`
  - Backward compatible (defaults to D6 if no metadata)

### Unit Tests
- **`test_petiole_lock.py`**: Automated tests (8 tests, all passing ✅)
  - Petiolule identification
  - can_apply() logic
  - DOF reduction estimation
  - Simple application
  - Geometry preservation
  - Mixed branch types
  - Validation
  - Edge case: no petiolules

### Visual Tests (Isaac Sim)
- **`generate_comparison_usd.py`**: Generate baseline vs petiole_lock USD
- **`compare_isaac_sim.py`**: Load both USD files side-by-side in Isaac Sim

## Running Tests

### Unit Tests
```bash
# Run all unit tests (8 tests)
uv run python src/exporterV2/core/optimizations/tests/4_petiole_lock/test_petiole_lock.py
```

### Isaac Sim Visual Comparison

**Step 1: Generate USD files**
```bash
cd /home/alessandro/isaacsim/autotom_digital_twin
uv run python src/exporterV2/core/optimizations/tests/4_petiole_lock/generate_comparison_usd.py
```

This creates:
- `usd_output/baseline.usda` - Articulated petiolules (D6 joints)
- `usd_output/petiole_lock.usda` - Fixed petiolules (Fixed joints)

**Step 2: Compare in Isaac Sim**
```bash
cd /home/alessandro/isaacsim/autotom_digital_twin
~/isaacsim/python.sh src/exporterV2/core/optimizations/tests/4_petiole_lock/compare_isaac_sim.py
```

**Step 3: Observe**
- Press PLAY in Isaac Sim
- **Left plant (x=-1.0)**: Baseline with articulated petiolules
- **Right plant (x=+1.0)**: Petiole lock with fixed petiolules
- Watch petiolules: baseline should oscillate, locked should stay rigid

### What to Verify
✅ **Visual**: Both plants look geometrically identical  
✅ **Physics**: Petiole lock petiolules don't move (fixed to rachis)  
✅ **Stability**: Petiole lock should be more stable (less oscillation)  
✅ **Performance**: Petiole lock should run faster (fewer DOF)  

## Usage Example

```python
from techniques import PetioleLockTechnique

technique = PetioleLockTechnique()

branches = [
    {"id": "trunk", "n_links": 5},
    {"id": "Petiolule_r1_o0_l0_lf0", "n_links": 1, "parent": "Rachis_r1_o0_l0"},
    {"id": "Petiolule_r1_o0_l0_lf1", "n_links": 1, "parent": "Rachis_r1_o0_l0"},
]

# Check if applicable
if technique.can_apply(branches):
    # Estimate reduction
    dof_reduction = technique.estimate_reduction(branches)
    print(f"Will reduce {dof_reduction} DOF")  # 12 (2 petiolules × 6 DOF)
    
    # Apply technique
    modified, report = technique.apply(branches)
    print(f"Locked {report.details['petiolules_locked']} petiolules")
    
    # Export USD with locked petiolules
    stage = build_stage(modified)
    stage.GetRootLayer().Export("plant_with_locked_petiolules.usda")
```

## Test Coverage

✅ **Petiolule identification** - By name pattern and parent  
✅ **Applicability check** - With/without petiolules, already fixed  
✅ **DOF estimation** - Correct count (6 per petiolule)  
✅ **Application** - Adds joint_type metadata correctly  
✅ **Geometry preservation** - n_links, height, radius unchanged  
✅ **Mixed branches** - Only petiolules get metadata  
✅ **Validation** - Topology preserved, errors detected  
✅ **Edge cases** - No petiolules handled gracefully  
✅ **Isaac Sim integration** - Visual comparison ready  

## Integration Status

✅ **Technique implementation** - Complete  
✅ **Unit tests** - 8/8 passing  
✅ **Stage.py integration** - joint_type metadata supported  
✅ **USD generation** - Baseline vs optimized  
✅ **Isaac Sim comparison** - Side-by-side visual test  

## Next Steps

- **Task 5**: Lateral Branch Reduction (reduce n_links in laterals)
- **Task 6**: Stem Collapse (uses Tasks 2+3 for remapping + collision)

Petiole Lock is complete and ready for production use!
