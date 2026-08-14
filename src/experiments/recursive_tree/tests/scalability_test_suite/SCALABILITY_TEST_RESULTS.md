# Scalability Test Results - Task 2 Complete

**Date**: 2024-08-03  
**Status**: ✅ **15/15 GEOMETRY TESTS PASSED**

---

## Summary

All 15 geometry scalability tests passed successfully. USD files generated and saved for Isaac Sim inspection.

**Test Coverage:**
- ✅ Baseline realistic tomato configuration (41 links)
- ✅ L/D slenderness variations (4 tests: L/D 8, 10, 12, petiole 10)
- ✅ Radius ratio tests (2 tests: 2.5×, 3.5×)
- ✅ Complexity tests (2 tests: 50, 59 links)
- ✅ Minimum radius tests (2 tests: 2mm, 1mm world)
- ✅ Tilt angle tests (4 tests: 30°, 60°, 90°, mixed)

**Key Finding**: 
- PhysX articulation limit is **64 links total**
- 7 petioles (68 links) → FAILS validation ❌
- 6 petioles (59 links) → PASSES ✅

---

## Test Results Table

| # | Config Name | Category | Status | Links | Max Error | Expected Risk |
|---|-------------|----------|--------|-------|-----------|---------------|
| 1 | baseline_tomato_realistic | Baseline | ✅ PASS | 41 | 0.000mm | SAFE/MARGINAL |
| 2 | petiolule_ld_8 | L/D | ✅ PASS | 41 | 0.000mm | MARGINAL |
| 3 | petiolule_ld_10 | L/D | ✅ PASS | 41 | 0.000mm | RISKY |
| 4 | petiolule_ld_12 | L/D | ✅ PASS | 41 | 0.000mm | UNSAFE |
| 5 | petiole_ld_10 | L/D | ✅ PASS | 41 | 0.000mm | RISKY |
| 6 | radius_ratio_2_5 | Radius | ✅ PASS | 41 | 0.000mm | MARGINAL |
| 7 | radius_ratio_3_5 | Radius | ✅ PASS | 41 | 0.000mm | RISKY |
| 8 | six_petioles_50_links | Complexity | ✅ PASS | 59 | 0.000mm | MARGINAL |
| 9 | five_petioles_50_links | Complexity | ✅ PASS | 50 | 0.000mm | MARGINAL |
| 10 | min_radius_2mm_world | Min Radius | ✅ PASS | 41 | 0.000mm | MARGINAL |
| 11 | min_radius_1mm_world | Min Radius | ✅ PASS | 41 | 0.000mm | UNSAFE |
| 12 | petiole_tilt_30 | Tilt | ✅ PASS | 41 | 0.000mm | SAFE |
| 13 | petiole_tilt_60 | Tilt | ✅ PASS | 41 | 0.000mm | MARGINAL |
| 14 | petiole_tilt_90 | Tilt | ✅ PASS | 41 | 0.000mm | RISKY |
| 15 | mixed_angles | Tilt | ✅ PASS | 41 | 0.000mm | MARGINAL |

**Note**: "Expected Risk" indicates predicted behavior in Isaac Sim convergence tests (Task 3).

---

## USD Files Generated

All USD files saved in: `tests/scalability_usds/`

You can load them in Isaac Sim by modifying `load_recursive_tree.py`:

```python
# Change USD path to:
usd_path = "src/experiments/recursive_tree/tests/scalability_usds/baseline_tomato_realistic.usda"
```

**Files (15 total):**
- `baseline_tomato_realistic.usda`
- `petiolule_ld_8.usda`, `petiolule_ld_10.usda`, `petiolule_ld_12.usda`
- `petiole_ld_10.usda`
- `radius_ratio_2_5.usda`, `radius_ratio_3_5.usda`
- `six_petioles_50_links.usda`, `five_petioles_50_links.usda`
- `min_radius_2mm_world.usda`, `min_radius_1mm_world.usda`
- `petiole_tilt_30.usda`, `petiole_tilt_60.usda`, `petiole_tilt_90.usda`
- `mixed_angles.usda`

---

## Validation Performed

Each test verified:
1. ✅ **Validation** - tree_config.validate_branches() passes
2. ✅ **USD Generation** - USDA file created successfully
3. ✅ **Geometry Correctness** - Joint positions match expected values (<1mm error)
4. ✅ **Physics Validity** - No NaN/inf in inertia/COM calculations
5. ✅ **USD Persistence** - File saved for manual Isaac Sim loading

---

## Key Constraints Discovered

### PhysX Articulation Limit
- **Hard limit**: 64 links total (including stem + all branches)
- **Safe maximum**: 59 links (6 petioles × 3 petiolules + stem)
- **Validation catches**: Tree config validation catches this before USD generation

### Collision Threshold (TBD in Task 3)
- Geometry passes with radius as low as 1mm world (0.5mm pre-scale)
- Isaac Sim convergence tests will verify if collision detection works at these scales

### L/D Ratios (TBD in Task 3)
- Geometry correct up to L/D=12 (petiolule)
- Droop predictions not validated yet (need Isaac Sim physics)
- "Delayed divergence" hypothesis needs 30s convergence tests

---

## Next Steps: Task 3 - Isaac Sim Convergence Tests

**Goal**: Test which configurations are **stable in simulation** (not just geometrically valid).

**Approach**:
1. **Phase A**: 10s quick screening on all 15 configs
   - Classify: STABLE / MARGINAL / UNSTABLE
   - Metrics: position drift, velocity oscillations, articulation state

2. **Phase B**: 30s long-term tests on MARGINAL configs
   - Detect "delayed divergence" (user's insight)
   - Identify configs that seem stable at 10s but fail at 30s

3. **Output**: List of SAFE configs for robot interaction tests (Task 4)

**Test configurations for Task 3**:
- Prioritize: baseline, L/D 8/10, tilt 30°/60°/90° (6 HIGH priority configs)
- Optional: Add MEDIUM priority if time permits (radius ratios, complexity)

---

## Test Infrastructure

**Test file**: `src/experiments/recursive_tree/tests/test_scalability.py`

**Key functions**:
- `generate_baseline_tomato()` - 41-link realistic config
- `modify_config(base, changes)` - Apply parameter modifications
- `test_config_geometry(name, branches, status, save_usd=True)` - Full validation + USD save
- `compute_ld_ratio(r, h, n_links)` - Calculate L/D for slenderness analysis

**Usage**:
```bash
uv run src/experiments/recursive_tree/tests/test_scalability.py
```

**Add new tests**:
1. Copy existing test function (e.g., `test_2_1_petiolule_ld_8`)
2. Modify parameters via `modify_config()` or create custom `branches` list
3. Add function call to `main()` runner
4. USD auto-saved to `tests/scalability_usds/`

---

## Observations

### Geometry vs Physics
- **All 15 configs** are geometrically valid
- But geometry ≠ physics stability
- Example: L/D=12 petiolule has correct geometry, but predicted droop ~155mm → likely unstable in Isaac Sim

### Radius Findings
- 1mm world radius (0.5mm pre-scale, after 2× scaling) → geometry valid
- Whether collision/physics work at this scale → TBD in Task 3
- This approaches the theoretical limits of PhysX collision detection

### Complexity Limits
- 59 links (6 petioles) → max practical complexity
- 68 links (7 petioles) → exceeds PhysX limit
- Real tomato plants likely stay <<59 links per articulation
- For larger plants: need multiple articulations or static geometry

---

## Technical Notes

**USD Scale Factor**: All dimensions use 2× scale in USD (world units)
- Example: 0.004m radius → 0.008m in world

**Coordinate System**: 
- Stem: +Z up
- Petioles: tilted from stem
- Petiolules: tilted from petiole

**Physics Settings** (for Task 3):
- PhysX timestep: 1/480 Hz = 2.08ms
- Position iterations: 64
- Velocity iterations: 8
- Damping: 0.2 linear, 0.5 angular

**Test Duration** (Task 3):
- Quick screen: 10s (4800 steps)
- Long-term: 30s (14400 steps) for MARGINAL configs

---

## Appendix: Configuration Details

### Baseline Tomato Realistic (41 links)
```
Stem: 5 links, r=4mm, h=30mm, vertical
4× Petiole: 3 links each, r=2.3mm, h=27mm, tilt=45°
  → 12× Petiolule: 2 links each, r=1.5mm, h=15mm, tilt=30°
```

**Hierarchy**:
```
stem (5)
├─ petiole_1 (3)
│  ├─ petiolule_1_1 (2)
│  ├─ petiolule_1_2 (2)
│  └─ petiolule_1_3 (2)
├─ petiole_2 (3) ...
├─ petiole_3 (3) ...
└─ petiole_4 (3) ...

Total: 5 + 4×(3 + 3×2) = 5 + 4×9 = 41 links
```

---

**Generated**: 2024-08-03  
**Test Suite**: `test_scalability.py`  
**Ready for**: Task 3 (Isaac Sim Convergence Tests)
