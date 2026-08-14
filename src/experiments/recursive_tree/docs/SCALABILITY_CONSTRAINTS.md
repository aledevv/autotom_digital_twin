# Scalability & Stability Constraints - Recursive Tree

**Purpose**: This document defines the constraint ranges for creating stable recursive trees in Isaac Sim, based on biological data (tomato plant) and existing codebase stability patterns.

**Target Plant**: Tomato (Solanum lycopersicum)  
**Target Use Case**: Robot interaction with complex branching structure (depth=3: stem → petiole → petiolule → leaflets)

**Status**: ✅ Research Complete - Ready for Test Implementation

---

## 1. Biological Constraints (Real Tomato Data)

Data source: `data/organ_full_statistics.csv` (Day 1, 60, 160)

### 1.1 Organ Dimensions

| Organ | Diameter (mm) | Length (mm) | L/D Ratio | Notes |
|-------|---------------|-------------|-----------|-------|
| **Stem (Internode)** | 6.0 - 8.7 | 9.3 - 40.0 | **1.3 - 3.6** | Very thick, low slenderness |
| **Petiole** | 2.7 - 5.7 | 14.1 - 35.0 | **5.3 - 5.9** | Moderate slenderness |
| **Petiolule** | 1.5 - 3.0* | 10 - 20* | **~5 - 7*** | Thin, potentially critical for stability |

*Estimated from `leaf_segments_length` and biological proportions (petiolule ≈ 0.5-0.6× petiole diameter)

**Key insight**: Il petiolule è lo "stelo" che collega il petiole alla foglia - è sottile (1.5-3mm) e potenzialmente critico per instabilità.

### 1.2 Parent-Child Radius Ratios

From real data:
- **Stem/Petiole ratio**: ~1.7× (7.9mm / 4.6mm @ day 60)
- **Petiole/Petiolule ratio**: ~1.5-2.0× (estimated from biological hierarchy)

**Biological range**: Parent radius = [1.5×, 2.5×] × child radius

---

## 2. Isaac Sim PhysX Constraints (From Existing Code)

### 2.1 Hard Limits

| Constraint | Value | Source | Notes |
|------------|-------|--------|-------|
| **Max total links** | 64 | PhysX articulation limit | Already validated in `tree_config.py` |
| **Min world radius** | 0.002m (2mm) | Collision detection estimate | Below this: collision misses likely |
| **Max recursion depth** | Unlimited* | Logical | *As long as total_links ≤ 64 |

### 2.2 PhysX Solver Settings (From Codebase Analysis)

**Scene-level settings observed in codebase**:

| Experiment | TimeSteps/s | Pos Iter | Vel Iter | E (MPa) | Notes |
|------------|-------------|----------|----------|---------|-------|
| `three_point_test` | **6000 Hz** | 128 | 32 | 35 | Very high stiffness |
| `cantilever_test` | **480 Hz** | 128 | 32 | 1.5-50 | High stiffness, calibrated |
| `recursive_tree` | **480 Hz** | 64 | 8 | 50 | Current tomato config |
| Most articulations | **120 Hz** | 64 | 8 | N/A | Standard default |

**Common settings across all**:
- SolverType: `"TGS"` (Temporal Gauss-Seidel)
- EnableCCD: `True` (Continuous Collision Detection)
- EnableStabilization: `True`
- EnabledSelfCollisions: `False`
- SleepThreshold: `0.0` (always awake)

**Rule of thumb from code**: Higher stiffness (K) requires higher timestep frequency to avoid instability.

**For tomato plant (E=50 MPa)**:
- Recommended: **480 Hz + 64/8 iterations** (already used in `recursive_tree`)
- If instability occurs: increase to **480 Hz + 128/32 iterations**
- For extreme slenderness: may need **1000-2000 Hz** (to test)

---

## 3. Droop Theory Predictions (From droop_theory.py)

Euler-Bernoulli cantilever deflection under gravity:

```
δ_tip = (q × L⁴) / (8 × E × I)
δ_effective = δ_tip × sin(tilt_angle)

where:
  q = ρ × g × π × r²  [N/m] (load per unit length)
  E = 50 MPa (tomato stem, from BioConfig)
  I = π × r⁴ / 4 [m⁴] (second moment of area)
  ρ = 1000 kg/m³ (plant tissue density)
```

### 3.1 Slenderness Effect on Droop

For E=50 MPa, ρ=1000 kg/m³, assuming horizontal beam (tilt=90°):

| L/D | Length (m) | Radius (m) | δ_tip (mm) | Droop Category |
|-----|-----------|-----------|------------|----------------|
| **3.6** | 0.029 | 0.008 | **0.15** | Stem-like: negligible droop |
| **5.9** | 0.027 | 0.0046 | **2.5** | Petiole-like: small, acceptable |
| **8.0** | 0.040 | 0.005 | **18** | Getting noticeable |
| **10.0** | 0.050 | 0.005 | **55** | Large droop! |
| **12.0** | 0.060 | 0.005 | **155** | Very large, likely unstable |

**Critical insight**: L/D > 10 leads to droop > 50mm for typical branch dimensions → likely causes instability in simulation.

**For tilted branches**: δ_effective = δ_tip × sin(tilt)
- Vertical branch (tilt=0°): No droop
- 45° branch: 71% of horizontal droop
- Horizontal branch (tilt=90°): Full droop

---

## 4. Stability Constraints (Hypothesis - To Be Tested)

### 4.1 Slenderness Ratio (L/D)

| Range | Status | Expected Behavior | Test Method |
|-------|--------|-------------------|-------------|
| **L/D < 6** | ✅ **SAFE** | Stable, droop < 5mm | Geometry + 10s convergence |
| **L/D = 6-10** | ⚠️ **MARGINAL** | Acceptable droop (5-55mm), check long-term | **60s test for delayed divergence!** |
| **L/D = 10-15** | ⚠️ **RISKY** | High droop (50-300mm), oscillations likely | Visual + force test |
| **L/D > 15** | ❌ **UNSAFE** | Extreme droop, divergence expected | Quick fail expected |

**Tomato plant reference**:
- Stem: L/D = 3.6 → **SAFE**
- Petiole: L/D = 5.9 → **SAFE** (at boundary)
- Petiolule: L/D ≈ 6-7 → **MARGINAL** (needs testing!)

### 4.2 Parent-Child Radius Ratio

| Range | Status | Expected Behavior | Test Method |
|-------|--------|-------------------|-------------|
| **1.5× - 2.0×** | ✅ **SAFE** | Biologically realistic, stable attachment | Geometry test |
| **2.0× - 3.0×** | ⚠️ **MARGINAL** | Thin child, attachment stress | Convergence + force test |
| **> 3.0×** | ⚠️ **RISKY** | Very thin child, high attachment stress | Likely failure |

**Tomato plant reference**:
- Stem/Petiole = 1.7× → **SAFE**
- Petiole/Petiolule ≈ 1.5-2.0× → **SAFE**

### 4.3 Minimum World Radius

| Value (world) | Status | Expected Behavior | Test Method |
|---------------|--------|-------------------|-------------|
| **≥ 4mm** | ✅ **SAFE** | Petiole-level, proven in existing tests | Existing tests pass |
| **2-4mm** | ⚠️ **MARGINAL** | Petiolule level, collision OK but physics? | Convergence test |
| **< 2mm** | ❌ **RISKY** | Collision misses, numerical issues | Failure expected |

**Tomato plant reference** (with GLOBAL_SCALE=2):
- Petiolule: 1.5-3mm pre-scale → **3-6mm world** → **SAFE**

### 4.4 Total Complexity (Links & Branches)

| Config | Total Links | Branches | Status | Notes |
|--------|-------------|----------|--------|-------|
| **Simple** | 10-20 | 1-3 | ✅ **SAFE** | Existing tests pass |
| **Moderate** | 30-45 | 4-8 | ⚠️ **MARGINAL** | Tomato-realistic config |
| **Complex** | 50-64 | 10+ | ⚠️ **RISKY** | At PhysX limit, performance? |

**Tomato realistic config**: 41 links (5 stem + 4×(3 petiole + 3×2 petiolule)) → **MARGINAL** range.

---

## 5. Expected Safe Configuration Ranges (Summary)

| Parameter | SAFE (✅ Green) | MARGINAL (⚠️ Yellow) | UNSAFE (❌ Red) |
|-----------|----------------|---------------------|----------------|
| **L/D ratio** | < 6 | 6-10 | > 10 |
| **Parent/child radius** | 1.5-2.0× | 2.0-3.0× | > 3.0× |
| **Min world radius** | ≥ 4mm | 2-4mm | < 2mm |
| **Total links** | < 40 | 40-60 | > 60 |
| **Branch density** | ≤ 4 per link | 5-6 per link | > 6 per link |
| **PhysX timestep** | 480 Hz | 240 Hz | < 120 Hz |
| **Solver iterations** | 64/8 | 32/4 | < 16/2 |

**Tomato realistic config** (baseline):
- 5 stem links (L/D=3.6) → ✅ SAFE
- 4 petioles (L/D=5.9) → ✅ SAFE
- 3×2 petiolules per petiole (L/D≈6-7) → ⚠️ **MARGINAL** (needs testing!)
- Total 41 links → ⚠️ MARGINAL
- **Overall assessment**: Good starting point, petiolules are the critical element to test!

---

## 6. Open Questions (To Be Answered by Tests)

### Critical Questions:

1. **Delayed divergence threshold**: What L/D ratio causes crash after 30s instead of 10s? (your key insight!)
2. **Petiolule stability**: Is L/D=6-7 stable for thin petiolules (3-6mm world radius)?
3. **Branch density limit**: Can we have 6 petioles per stem link without performance issues?
4. **Force resistance**: What impulse force (1N? 5N? 10N?) causes permanent deformation?

### Secondary Questions:

5. **Tilt angle effect**: Are horizontal branches (90°) more unstable than 45° branches?
6. **Collision threshold**: What's the actual minimum radius where collisions start failing?
7. **Performance scaling**: Does FPS degrade linearly with link count, or suddenly at 50+ links?
8. **Multiple branches per link**: Does having 4 branches attached to same link cause instability?

---

## 7. Test Plan Summary (Incremental Approach)

### Task 1: Research & Constraints ✅ COMPLETE

This document.

### Task 2: Geometry Limit Tests (USD-only, ~2-3h)

**Objective**: Verify that config variations generate valid USD without Isaac Sim.

**Test strategy**: Start from tomato-realistic baseline, push ONE parameter at a time.

**Test cases** (~12 tests):
1. Baseline: Tomato realistic (41 links, L/D≈4-6, ratio≈1.7×)
2. Push L/D: 8, 10, 12, 15 (find slenderness limit)
3. Push radius ratio: 2.5×, 3×, 4× (find thin-child limit)
4. Push total links: 50, 60, 64 (PhysX scaling)
5. Push min radius: 2mm, 1mm world (collision threshold)
6. Push branch density: 6 petioles, 5 petiolules (complexity)

**Pass criteria**:
- `validate_branches()` passes
- USD generates without errors
- Geometry positions match analytical (< 1mm error)
- No NaN/inf in physics calculations (K, D)

**Output**: List of configs to test in Isaac Sim (Phase 2).

### Task 3: Convergence Tests (Isaac Sim, headless, ~3-4h)

**Objective**: Detect convergence vs divergence, including **delayed instability**.

**Two-phase approach**:

**Phase A: Quick screening (10s per config)**
- All configs from Task 2 that passed
- Simulate 10s @ 480 Hz
- Measure: max velocity, settling time, FPS
- Classify: STABLE / MARGINAL / UNSTABLE

**Phase B: Long-term validation (30s per MARGINAL config)** ← **Your key insight!**
- Only configs marked MARGINAL in Phase A
- Simulate 30s @ 480 Hz
- Monitor: velocity RMS over 5s windows (must decrease monotonically)
- Detect: **delayed divergence** (parts keep moving randomly after 15s+)

**Pass criteria**:
- Settling time < 5s
- Velocity → 0 (< 0.01 m/s)
- No delayed explosions
- FPS > 30

**Output**:
- CSV with convergence metrics per config
- List of configs needing visual inspection

### Task 4: Force Resistance Tests (Isaac Sim, ~2-3h)

For STABLE configs from Task 3:

**Impulse force (robot touch)**:
- Apply 1N, 5N, 10N impulses (0.1s each) to leaf/petiolule tip
- Measure: max deflection, recovery time per force level

**Sustained force (robot pull)**:
- Apply 1N, 5N, 10N forces for 2s, then release
- Measure: permanent deformation per force level

**Pass criteria**:
- Recovery < 5s
- Residual deformation < 10% of max deflection

### Task 5: Visual Inspection (Isaac Sim with rendering, ~1h)

For MARGINAL configs:
- Render simulation video (mp4)
- Screenshot every 10s
- Human review for: jittering, interpenetration, explosion

---

## 8. Tomato Realistic Reference Config (Baseline)

For use in tests:

```python
# Stem (main trunk)
# Pre-scale: r=0.004m (4mm), h=0.03m (30mm), L/D=3.75
# World (×2): r=8mm, h=60mm, total_L=300mm (5 links)
STEM = {
    "id": "stem",
    "parent": None,
    "attach_link": None,
    "n_links": 5,
    "radius": 0.004,  # 4mm → 8mm world (avg day 60)
    "height": 0.030,  # 30mm → 60mm world (avg internodes)
    "tilt": 0.0,
    "rot": 0.0,
}

# Petiole (branch holding leaflets)
# Pre-scale: r=0.0023m (2.3mm), h=0.027m (27mm), L/D=5.87
# World (×2): r=4.6mm, h=54mm, total_L=162mm (3 links)
PETIOLE_TEMPLATE = {
    "id": "petiole_{i}",
    "parent": "stem",
    "attach_link": 2,  # Varies: 2, 3, 4, 5
    "n_links": 3,
    "radius": 0.0023,  # 2.3mm → 4.6mm world (avg day 60)
    "height": 0.027,   # 27mm → 54mm world (avg length)
    "tilt": 45.0,      # Realistic angle
    "rot": 0.0,        # Varies: 0°, 90°, 180°, 270°
}

# Petiolule (leaflet stem - THE CRITICAL ELEMENT!)
# Pre-scale: r=0.0015m (1.5mm), h=0.015m (15mm), L/D=5.0
# World (×2): r=3mm, h=30mm, total_L=60mm (2 links)
PETIOLULE_TEMPLATE = {
    "id": "petiolule_{i}_{j}",
    "parent": "petiole_{i}",
    "attach_link": 2,  # Middle of petiole
    "n_links": 2,
    "radius": 0.0015,  # 1.5mm → 3mm world (estimated)
    "height": 0.015,   # 15mm → 30mm world (from leaf_segments)
    "tilt": 30.0,      # Smaller tilt
    "rot": 0.0,        # Varies per leaflet position
}

# Full tree: 4 petioles × (3 links + 3 petiolules × 2 links) + 5 stem = 41 links
```

---

## 9. Next Steps

**✅ Step 1.1 COMPLETE**: Constraints document created

**→ Step 1.2 NEXT**: User review + feedback on this document

**User Review Results**:
1. ✅ L/D thresholds (6, 10, 15) approved - will adjust during testing if needed
2. ✅ Test all tilt angles: 30°, 45°, 60°, 90° systematically
3. ✅ Long-term test duration: **30s** is sufficient for delayed divergence detection
4. ✅ Force magnitudes for robot interaction: **1N, 5N, 10N**

**→ Step 1.3 IN PROGRESS**: Creating specific test configurations for Task 2

---

## References

- **Biological data**: `data/organ_full_statistics.csv`, `data/organ_minmax_features.csv`, `data/report_table.md`
- **Existing code**: `cantilever_test/run_cantilever.py`, `recursive_tree/droop_theory.py`, `recursive_tree/load_recursive_tree.py`
- **Physics parameters**: `tree_config.py` (E=50 MPa, ρ=1000 kg/m³, ζ=0.2)
