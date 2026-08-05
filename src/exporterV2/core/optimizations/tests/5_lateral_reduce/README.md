# Task 5: Lateral Branch Reduction - Test Suite

## Overview

Implements the **Lateral Branch Reduction** optimization technique that reduces the number of segments (n_links) in lateral branches incrementally while preserving total branch length through height recalculation and remapping child attachment points.

## Technique Details

**Priority**: 2 (after Petiole Lock - medium visual impact)

**Strategy**:
- Reduces n_links by 1 per iteration for all applicable branches
- Reduces smallest branches first (by radius) → least significant mechanically
- In case of tie, reduces lower branches first (by attach_link)
- In case of tie, alphabetically by branch ID

**Impact**:
- Reduces joint count (less articulation)
- Preserves total branch length by recalculating per-link height
- Remaps child attachments using geometry remapping (Task 3)

**Minimum Constraint**: Respects `min_segments` from config (default: 1)

## Files

```
tests/5_lateral_reduce/
├── test_lateral_reduce.py          # 12 unit tests (all passing)
├── generate_comparison_usd.py      # USD generation script
├── compare_isaac_sim.py            # Isaac Sim visual comparison (TODO)
├── README.md                       # This file
└── usd_output/                     # Generated files
    ├── baseline.usda               # 3 lateral branches with 3+3+2 = 8 links
    └── lateral_reduce.usda         # 3 lateral branches with 1+1+1 = 3 links
```

## Unit Tests (12)

Run tests:
```bash
cd /home/alessandro/isaacsim/autotom_digital_twin
uv run pytest src/exporterV2/core/optimizations/tests/5_lateral_reduce/test_lateral_reduce.py -v
```

**Tests**:
1. ✅ `test_identify_lateral_branches` - Identifies Branch_r*_o* and LateralLeaf_r*_o*
2. ✅ `test_can_reduce` - Checks n_links > min_segments
3. ✅ `test_reduction_priority` - Verifies smallest → lowest → alphabetical
4. ✅ `test_can_apply` - Detects reducible branches
5. ✅ `test_estimate_reduction` - Counts total reducible links
6. ✅ `test_apply_single_branch` - Single branch 3→2 with height recalculation
7. ✅ `test_apply_with_child_remapping` - Child attachment remapped correctly
8. ✅ `test_apply_multiple_branches` - Multiple branches reduced with priority
9. ✅ `test_apply_respects_minimum` - min_segments constraint enforced
10. ✅ `test_validate_success` - Validation passes for correct reduction
11. ✅ `test_validate_detects_errors` - Validation catches invalid modifications
12. ✅ `test_no_reducible_branches` - Handles case with no reducible branches

**Result**: ✅ **12/12 tests passing**

## USD Generation

Generate USD files for Isaac Sim comparison:
```bash
cd /home/alessandro/isaacsim/autotom_digital_twin
uv run python src/exporterV2/core/optimizations/tests/5_lateral_reduce/generate_comparison_usd.py
```

**Output**:
- `baseline.usda` (45KB, 931 lines) - 3 lateral branches with full articulation
- `lateral_reduce.usda` (31KB, 650 lines) - 3 lateral branches reduced to 1 link each

**Difference**: 5 links removed (8 → 3)

## Isaac Sim Visual Test

**Load files manually**:
```bash
# Baseline (multi-segment lateral branches)
~/isaacsim/python.sh -m isaacsim \
  /home/alessandro/isaacsim/autotom_digital_twin/src/exporterV2/core/optimizations/tests/5_lateral_reduce/usd_output/baseline.usda

# Lateral Reduce (single-segment lateral branches)
~/isaacsim/python.sh -m isaacsim \
  /home/alessandro/isaacsim/autotom_digital_twin/src/exporterV2/core/optimizations/tests/5_lateral_reduce/usd_output/lateral_reduce.usda
```

**What to observe**:
1. **Baseline**: Lateral branches articulated with 2-3 segments each
2. **Lateral Reduce**: Lateral branches are single rigid segments
3. **Length**: Total branch length preserved (e.g., 3×0.2m → 1×0.6m)
4. **Attachments**: Petioles correctly remapped (attach_link=2 → attach_link=1)
5. **Visual**: Similar overall shape, less flexibility
6. **Performance**: Reduced model should be more stable (fewer degrees of freedom)

## Implementation Details

### Branch Identification

**Lateral branches**: `Branch_r{rank}_o{organ}`
**Lateral leaves**: `LateralLeaf_r{rank}_o{organ}`

Both types are reduced by this technique.

### Reduction Algorithm

1. Find all reducible branches (n_links > min_segments)
2. Sort by priority (radius, attach_link, ID)
3. For each branch:
   - Reduce n_links by 1
   - Recalculate height: `new_height = old_height * old_n_links / new_n_links`
   - Remap children using `remap_attachment_height()` from Task 3
4. Iterate until no more reductions possible

### Geometry Preservation

**Height recalculation**:
```python
old_length = n_links * height         # e.g., 3 × 0.2m = 0.6m
new_height = old_length / new_n_links # e.g., 0.6m / 1 = 0.6m per link
```

**Child remapping** (uses Task 3 geometry remapping):
- Preserves absolute height of attachment point
- Example: attach_link=2 on 3-link branch → attach_link=1 on 1-link branch

### Validation

Checks:
- ✅ Same branch count
- ✅ Same branch IDs
- ✅ Same parent-child relationships
- ✅ Only lateral branches/leaves modified
- ✅ n_links >= min_segments
- ✅ Total length preserved (within 1% tolerance)
- ✅ Radii unchanged
- ✅ Child attachments valid (1 <= attach_link <= parent_n_links)

## Integration with Optimizer

This technique integrates into the `BudgetOptimizer` workflow:

```python
from exporterV2.core.optimizations.optimizer import BudgetOptimizer
from exporterV2.core.optimizations.techniques.lateral_reduce import LateralBranchReductionTechnique

# Register technique
optimizer = BudgetOptimizer(config_path="budget_config.yaml")
optimizer.register_technique(LateralBranchReductionTechnique(min_segments=1))

# Apply optimization
optimized_branches, report = optimizer.optimize(branches)
```

**Priority order** (from config):
1. Petiole Lock (Priority 1) - No geometry change
2. **Lateral Reduce (Priority 2)** ← This technique
3. Stem Collapse (Priority 3) - Higher impact
4. Truss Static (Priority 4)
5. Leaf Branch Reduce (Priority 5)

## Next Steps

1. ✅ Unit tests complete (12/12 passing)
2. ✅ USD generation script complete
3. 🔴 Isaac Sim comparison script (TODO)
4. 🔴 Visual validation checklist (TODO)

After Isaac Sim testing, proceed to **Task 6: Stem Collapse**.

## Notes

- Technique works for both `Branch_r*_o*` and `LateralLeaf_r*_o*` patterns
- Reduction is iterative: can be applied multiple times until min_segments reached
- Child remapping uses geometry remapping from Task 3 for accuracy
- Fallback to proportional mapping if geometry remapping unavailable
- Validation ensures topology and geometry constraints preserved

## Technical Details

**File**: `techniques/lateral_reduce.py` (330+ lines)
**Class**: `LateralBranchReductionTechnique`
**Dependencies**: Task 1 (infrastructure), Task 3 (geometry remapping)
**Test Coverage**: 12 unit tests covering all major scenarios

**Performance**: < 1ms per technique application on typical plant (< 100 branches)
