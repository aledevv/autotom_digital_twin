# Scalability Test Configurations

**Purpose**: Defines specific test configurations for Task 2 (Geometry Limits) and Task 3 (Convergence Tests).

**Based on**: `SCALABILITY_CONSTRAINTS.md` hypothesis and tomato biological data.

**Status**: ✅ Ready for Implementation

---

## Test Strategy

**Incremental approach**: Start from tomato-realistic baseline, push ONE parameter at a time to find limits.

**Categories**:
1. **Baseline** - Tomato realistic config (reference)
2. **Slenderness (L/D)** - Push ratio to find instability threshold
3. **Radius Ratio** - Push parent/child ratio to find thin-child limit
4. **Total Complexity** - Push link count to PhysX limit
5. **Minimum Radius** - Push to collision/physics threshold
6. **Tilt Angles** - Test all angles systematically

---

## Category 1: Baseline (Reference)

### Config 1.1: Tomato Realistic (BASELINE)

**Description**: Biologically accurate tomato plant configuration.

**Expected**: ✅ SAFE/MARGINAL (41 links, L/D=3.6-5.9, petiolule L/D≈6)

```python
BRANCHES = [
    # Stem (main trunk) - 5 links
    {
        "id": "stem",
        "parent": None,
        "attach_link": None,
        "n_links": 5,
        "radius": 0.004,   # 4mm → 8mm world
        "height": 0.030,   # 30mm → 60mm world (L/D = 3.75)
        "tilt": 0.0,
        "rot": 0.0,
    },
    # Petiole 1 - 3 links, attached to stem link 2
    {
        "id": "petiole_1",
        "parent": "stem",
        "attach_link": 2,
        "n_links": 3,
        "radius": 0.0023,  # 2.3mm → 4.6mm world
        "height": 0.027,   # 27mm → 54mm world (L/D = 5.87)
        "tilt": 45.0,
        "rot": 0.0,
    },
    # Petiolules for petiole_1 (3 leaflets)
    {
        "id": "petiolule_1_1",
        "parent": "petiole_1",
        "attach_link": 1,
        "n_links": 2,
        "radius": 0.0015,  # 1.5mm → 3mm world
        "height": 0.015,   # 15mm → 30mm world (L/D = 5.0)
        "tilt": 30.0,
        "rot": 0.0,
    },
    {
        "id": "petiolule_1_2",
        "parent": "petiole_1",
        "attach_link": 2,
        "n_links": 2,
        "radius": 0.0015,
        "height": 0.015,
        "tilt": 30.0,
        "rot": 120.0,
    },
    {
        "id": "petiolule_1_3",
        "parent": "petiole_1",
        "attach_link": 3,
        "n_links": 2,
        "radius": 0.0015,
        "height": 0.015,
        "tilt": 30.0,
        "rot": 240.0,
    },
    # Petiole 2 - attached to stem link 3
    {
        "id": "petiole_2",
        "parent": "stem",
        "attach_link": 3,
        "n_links": 3,
        "radius": 0.0023,
        "height": 0.027,
        "tilt": 45.0,
        "rot": 90.0,
    },
    # Petiolules for petiole_2
    {
        "id": "petiolule_2_1",
        "parent": "petiole_2",
        "attach_link": 1,
        "n_links": 2,
        "radius": 0.0015,
        "height": 0.015,
        "tilt": 30.0,
        "rot": 0.0,
    },
    {
        "id": "petiolule_2_2",
        "parent": "petiole_2",
        "attach_link": 2,
        "n_links": 2,
        "radius": 0.0015,
        "height": 0.015,
        "tilt": 30.0,
        "rot": 120.0,
    },
    {
        "id": "petiolule_2_3",
        "parent": "petiole_2",
        "attach_link": 3,
        "n_links": 2,
        "radius": 0.0015,
        "height": 0.015,
        "tilt": 30.0,
        "rot": 240.0,
    },
    # Petiole 3 - attached to stem link 4
    {
        "id": "petiole_3",
        "parent": "stem",
        "attach_link": 4,
        "n_links": 3,
        "radius": 0.0023,
        "height": 0.027,
        "tilt": 45.0,
        "rot": 180.0,
    },
    # Petiolules for petiole_3
    {
        "id": "petiolule_3_1",
        "parent": "petiole_3",
        "attach_link": 1,
        "n_links": 2,
        "radius": 0.0015,
        "height": 0.015,
        "tilt": 30.0,
        "rot": 0.0,
    },
    {
        "id": "petiolule_3_2",
        "parent": "petiole_3",
        "attach_link": 2,
        "n_links": 2,
        "radius": 0.0015,
        "height": 0.015,
        "tilt": 30.0,
        "rot": 120.0,
    },
    {
        "id": "petiolule_3_3",
        "parent": "petiole_3",
        "attach_link": 3,
        "n_links": 2,
        "radius": 0.0015,
        "height": 0.015,
        "tilt": 30.0,
        "rot": 240.0,
    },
    # Petiole 4 - attached to stem link 5
    {
        "id": "petiole_4",
        "parent": "stem",
        "attach_link": 5,
        "n_links": 3,
        "radius": 0.0023,
        "height": 0.027,
        "tilt": 45.0,
        "rot": 270.0,
    },
    # Petiolules for petiole_4
    {
        "id": "petiolule_4_1",
        "parent": "petiole_4",
        "attach_link": 1,
        "n_links": 2,
        "radius": 0.0015,
        "height": 0.015,
        "tilt": 30.0,
        "rot": 0.0,
    },
    {
        "id": "petiolule_4_2",
        "parent": "petiole_4",
        "attach_link": 2,
        "n_links": 2,
        "radius": 0.0015,
        "height": 0.015,
        "tilt": 30.0,
        "rot": 120.0,
    },
    {
        "id": "petiolule_4_3",
        "parent": "petiole_4",
        "attach_link": 3,
        "n_links": 2,
        "radius": 0.0015,
        "height": 0.015,
        "tilt": 30.0,
        "rot": 240.0,
    },
]

# Total: 5 (stem) + 4×(3 (petiole) + 3×2 (petiolules)) = 41 links
```

**Metrics**:
- Total links: 41
- Stem L/D: 3.75 (SAFE)
- Petiole L/D: 5.87 (SAFE)
- Petiolule L/D: 5.0 (SAFE, but close to MARGINAL)

---

## Category 2: Slenderness (L/D) Tests

Push L/D ratio incrementally to find instability threshold.

### Config 2.1: Petiolule L/D = 8 (MARGINAL)

**Description**: Increase petiolule length to push into MARGINAL range.

**Expected**: ⚠️ MARGINAL (needs 30s test for delayed divergence)

**Changes from baseline**:
- Petiolule height: 0.015 → **0.024** (24mm pre-scale → 48mm world)
- Petiolule L/D: 5.0 → **8.0**

```python
# All petiolule entries change:
{
    "id": "petiolule_*",
    "parent": "petiole_*",
    "attach_link": *,
    "n_links": 2,
    "radius": 0.0015,
    "height": 0.024,  # CHANGED: 24mm → 48mm world (L/D = 8.0)
    "tilt": 30.0,
    "rot": *,
}
```

---

### Config 2.2: Petiolule L/D = 10 (RISKY)

**Description**: Push to L/D=10 threshold (droop ≈ 55mm predicted).

**Expected**: ⚠️ RISKY (likely large oscillations or slow convergence)

**Changes from baseline**:
- Petiolule height: 0.015 → **0.030** (30mm pre-scale → 60mm world)
- Petiolule L/D: 5.0 → **10.0**

```python
{
    "id": "petiolule_*",
    "height": 0.030,  # CHANGED: 30mm → 60mm world (L/D = 10.0)
}
```

---

### Config 2.3: Petiolule L/D = 12 (UNSAFE expected)

**Description**: Beyond safe threshold (droop ≈ 155mm predicted).

**Expected**: ❌ UNSAFE (divergence or failure expected)

**Changes from baseline**:
- Petiolule height: 0.015 → **0.036** (36mm pre-scale → 72mm world)
- Petiolule L/D: 5.0 → **12.0**

```python
{
    "id": "petiolule_*",
    "height": 0.036,  # CHANGED: 36mm → 72mm world (L/D = 12.0)
}
```

---

### Config 2.4: Petiole L/D = 10 (RISKY)

**Description**: Test longer petiole slenderness (thicker than petiolule).

**Expected**: ⚠️ MARGINAL/RISKY (larger mass → different dynamics than petiolule)

**Changes from baseline**:
- Petiole height: 0.027 → **0.046** (46mm pre-scale → 92mm world)
- Petiole L/D: 5.87 → **10.0**

```python
{
    "id": "petiole_*",
    "height": 0.046,  # CHANGED: 46mm → 92mm world (L/D = 10.0)
}
```

---

## Category 3: Radius Ratio Tests

Push parent/child radius ratio to find thin-child limit.

### Config 3.1: Radius Ratio 2.5× (MARGINAL)

**Description**: Petiolule radius reduced to 2.5× thinner than petiole.

**Expected**: ⚠️ MARGINAL (thin child, attachment stress)

**Changes from baseline**:
- Petiolule radius: 0.0015 → **0.00092** (~0.9mm pre-scale → 1.8mm world)
- Ratio: 2.3/1.5 ≈ 1.5× → **2.3/0.92 ≈ 2.5×**

```python
{
    "id": "petiolule_*",
    "radius": 0.00092,  # CHANGED: 0.9mm → 1.8mm world (ratio 2.5×)
}
```

---

### Config 3.2: Radius Ratio 3.5× (RISKY)

**Description**: Very thin petiolule (approaching collision threshold).

**Expected**: ⚠️ RISKY (attachment stress + physics instability)

**Changes from baseline**:
- Petiolule radius: 0.0015 → **0.00066** (~0.66mm pre-scale → 1.3mm world)
- Ratio: **2.3/0.66 ≈ 3.5×**

```python
{
    "id": "petiolule_*",
    "radius": 0.00066,  # CHANGED: 0.66mm → 1.3mm world (ratio 3.5×)
}
```

---

## Category 4: Total Complexity Tests

Push total link count toward PhysX limit (64).

### Config 4.1: 6 Petioles (50 links)

**Description**: Increase branch density on stem.

**Expected**: ⚠️ MARGINAL (complexity + performance)

**Changes from baseline**:
- Add petiole_5 and petiole_6 with their petiolules
- Total links: 41 → **50** (5 + 6×(3 + 3×2))

```python
# Add petiole 5 (attach to stem link 3, rot=45°)
# Add petiole 6 (attach to stem link 4, rot=315°)
# Each with 3 petiolules × 2 links
```

---

### Config 4.2: 7 Petioles (59 links)

**Description**: Near PhysX limit.

**Expected**: ⚠️ RISKY (at limit, performance degradation expected)

**Changes from baseline**:
- Add petiole_5, petiole_6, petiole_7
- Total links: 41 → **59** (5 + 7×(3 + 3×2))

---

### Config 4.3: 8 Petioles + Reduced Petiolules (62 links)

**Description**: Maximum complexity within PhysX limit.

**Expected**: ⚠️ RISKY (just under limit, may have performance issues)

**Changes from baseline**:
- 8 petioles, but only 2 petiolules per petiole (instead of 3)
- Total links: **62** (5 + 8×(3 + 2×2))

---

## Category 5: Minimum Radius Tests

Push world radius to collision/physics threshold.

### Config 5.1: Petiolule Radius 2mm World (MARGINAL)

**Description**: Test at estimated collision threshold.

**Expected**: ⚠️ MARGINAL (collision may start failing)

**Changes from baseline**:
- Petiolule radius: 0.0015 → **0.001** (1mm pre-scale → 2mm world)
- At minimum safe threshold from constraints doc

```python
{
    "id": "petiolule_*",
    "radius": 0.001,  # CHANGED: 1mm → 2mm world (at threshold)
}
```

---

### Config 5.2: Petiolule Radius 1mm World (UNSAFE expected)

**Description**: Below collision threshold.

**Expected**: ❌ UNSAFE (collision misses + numerical issues likely)

**Changes from baseline**:
- Petiolule radius: 0.0015 → **0.0005** (0.5mm pre-scale → 1mm world)

```python
{
    "id": "petiolule_*",
    "radius": 0.0005,  # CHANGED: 0.5mm → 1mm world (below threshold)
}
```

---

## Category 6: Tilt Angle Tests

Test all angles systematically (user requested: 30°, 45°, 60°, 90°).

### Config 6.1: Petiole Tilt 30° (SAFE expected)

**Description**: Less tilt = less droop.

**Expected**: ✅ SAFE (lower effective droop than 45°)

**Changes from baseline**:
- Petiole tilt: 45° → **30°**

---

### Config 6.2: Petiole Tilt 60° (MARGINAL expected)

**Description**: Higher tilt = more droop.

**Expected**: ⚠️ MARGINAL (droop increases with sin(60°) ≈ 0.87)

**Changes from baseline**:
- Petiole tilt: 45° → **60°**

---

### Config 6.3: Petiole Tilt 90° (RISKY expected)

**Description**: Horizontal branch = maximum droop.

**Expected**: ⚠️ RISKY (full droop, worst case)

**Changes from baseline**:
- Petiole tilt: 45° → **90°**

---

### Config 6.4: Combined Angles Test

**Description**: Different tilts per level (stem=0°, petiole=60°, petiolule=30°).

**Expected**: ⚠️ MARGINAL (realistic variation)

**Changes from baseline**:
- Stem: 0° (unchanged)
- Petiole: 45° → **60°**
- Petiolule: 30° (unchanged)

---

## Summary Table

| Config | Category | Key Change | Total Links | Expected Status | Priority |
|--------|----------|------------|-------------|-----------------|----------|
| 1.1 | Baseline | Tomato realistic | 41 | ✅ SAFE/MARGINAL | **HIGH** |
| 2.1 | L/D | Petiolule L/D=8 | 41 | ⚠️ MARGINAL | **HIGH** |
| 2.2 | L/D | Petiolule L/D=10 | 41 | ⚠️ RISKY | **HIGH** |
| 2.3 | L/D | Petiolule L/D=12 | 41 | ❌ UNSAFE | MEDIUM |
| 2.4 | L/D | Petiole L/D=10 | 41 | ⚠️ RISKY | MEDIUM |
| 3.1 | Ratio | Ratio 2.5× | 41 | ⚠️ MARGINAL | MEDIUM |
| 3.2 | Ratio | Ratio 3.5× | 41 | ⚠️ RISKY | MEDIUM |
| 4.1 | Complexity | 6 petioles | 50 | ⚠️ MARGINAL | MEDIUM |
| 4.2 | Complexity | 7 petioles | 59 | ⚠️ RISKY | LOW |
| 4.3 | Complexity | 8 petioles | 62 | ⚠️ RISKY | LOW |
| 5.1 | Min radius | 2mm world | 41 | ⚠️ MARGINAL | MEDIUM |
| 5.2 | Min radius | 1mm world | 41 | ❌ UNSAFE | LOW |
| 6.1 | Tilt | 30° petiole | 41 | ✅ SAFE | HIGH |
| 6.2 | Tilt | 60° petiole | 41 | ⚠️ MARGINAL | HIGH |
| 6.3 | Tilt | 90° petiole | 41 | ⚠️ RISKY | HIGH |
| 6.4 | Tilt | Mixed angles | 41 | ⚠️ MARGINAL | MEDIUM |

**Total configs**: 16

**Recommended testing order**:
1. **Phase 1 (HIGH priority)**: 1.1, 2.1, 2.2, 6.1, 6.2, 6.3 (6 configs)
2. **Phase 2 (MEDIUM priority)**: 2.3, 2.4, 3.1, 3.2, 4.1, 5.1, 6.4 (7 configs)
3. **Phase 3 (LOW priority)**: 4.2, 4.3, 5.2 (3 configs - if time permits)

---

## Implementation Notes

### For Task 2 (Geometry Tests)

Each config will be tested in `test_scalability.py`:
1. Define BRANCHES list with config
2. Call `validate_branches()`
3. Generate USD with `build_stage()`
4. Verify geometry positions (< 1mm error)
5. Check physics calculations (no NaN/inf)

### For Task 3 (Convergence Tests)

Configs that pass Task 2 will be tested in Isaac Sim:
1. Load USD, initialize PhysX (480 Hz, 64/8 iterations)
2. **Phase A (10s)**: Quick screening → classify
3. **Phase B (30s)**: Long-term test for MARGINAL configs
4. Record metrics: max_velocity, settling_time, fps, stability_class

---

## Next Steps

**✅ Step 1.3 COMPLETE**: Test configurations defined

**→ Task 2 NEXT**: Implement geometry limit tests in `tests/test_scalability.py`

**Implementation approach**:
- Start with HIGH priority configs (baseline + L/D + tilts)
- Verify all pass geometry tests
- Then proceed to Task 3 (Isaac Sim convergence tests)
