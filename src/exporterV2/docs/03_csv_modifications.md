# CSV Modifications

How ExporterV2 deviates from raw groIMP CSV data.

---

## Overview

The groIMP CSV export provides **raw geometric data**, but needs processing for realistic plant models:

1. **Angle adjustments** (phyllotaxis fallback)
2. **Filtering** (opposite pairs selection)
3. **Random jitter** (natural variance)
4. **Collision checks** (prevent overlap)
5. **Leaf cloning** (complete missing pairs)

---

## 1. Angle Adjustments

### Problem
CSV provides `ccw_orientation` (counter-clockwise angle from +X axis), but:
- Some organs missing orientation data
- Need fallback for consistency

### Solution: Phyllotaxis Fallback

```python
# From profile
phyllotaxis_deg = 137.5  # Golden angle

# If CSV missing orientation:
rot_deg = (rank * phyllotaxis_deg) % 360
```

**Applied to:**
- Trunk leaves (when `ccw_orientation` missing)
- Lateral leaves (deterministic base)

**Not applied to:**
- Lateral branches (use profile `rot_base_deg` + jitter)

---

## 2. Filtering

### Problem
CSV contains **all organs** (up to 4 lateral branches, multiple leaves per rank). Not all needed for tomato model.

### Solution: Profile-Driven Selection

#### Trunk Leaves
```python
filter_strategy = "opposite_pairs_180deg"

# Select only organs with 180° separation
# Example: rank 5 has organs at [0°, 90°, 180°, 270°]
# → Keep [0°, 180°] only
```

**Why?** Tomato typically has opposite leaf pairs, not whorls.

#### Lateral Branches
```python
organ_indices = [0, 1]  # First two only

# From CSV: rank 3 has organs [0, 1, 2, 3]
# → Keep [0, 1] only (opposite pair)
```

**Why?** Tomato rarely has all 4 lateral branches per node.

#### Lateral Leaves
```python
organ_indices = [0, 1]
clone_missing = True

# If CSV has only organ_index=0:
# → Clone to create organ_index=1 at 180°
```

**Why?** Ensure symmetry even if CSV incomplete.

---

## 3. Random Jitter

### Problem
Perfect symmetry looks artificial.

### Solution: Random Rotation Jitter

#### Lateral Branches
```python
rot_base_deg = [0.0, 180.0]  # Base angles
rot_jitter_deg = 45.0         # Max deviation

# Example:
# Branch 0: 0° + random(-45°, +45°) = -23°
# Branch 1: 180° + random(-45°, +45°) = 201°
```

**Constraints:**
- Must maintain **60° minimum separation** (collision check)
- Checks same rank + adjacent ranks (rank±1)

#### Lateral Leaves
```python
rot_range_deg = (-90, 90)  # Range for random rotation

# Leaf 0: random(-90°, +90°) = 34°
# Leaf 1: Leaf 0 + 180° = 214°
```

**Why?** Natural variance without collision risk.

---

## 4. Collision Checks

### Problem
Random jitter can cause branches to overlap → simulation explosion.

### Solution: Geometric Validation

```python
min_angle_separation_deg = 60.0

# For each new branch:
# 1. Generate random angle
# 2. Check against existing branches (same rank + rank±1)
# 3. If separation < 60°, retry (max 100 attempts)
# 4. Accept or skip if all attempts fail
```

**Example (day 100 test):**
- Parent rank 0: 25.0° / 206.7° (sep: 178.3°) ✅
- Parent rank 1: 90.0° / 271.7° (sep: 178.3°) ✅
- Parent rank 2: 12.5° / 161.2° (sep: 148.7°) ✅

**Verified:** No collisions in geometric tests.

---

## 5. Leaf Cloning

### Problem
CSV sometimes has only 1 leaf per lateral branch (incomplete pair).

### Solution: Mirror Missing Leaf

```python
if clone_missing and len(lateral_leaves) == 1:
    # Clone existing leaf
    cloned = copy.deepcopy(lateral_leaves[0])
    cloned["organ_index"] = 1
    cloned["ccw_orientation"] += 180.0  # Opposite side
    lateral_leaves.append(cloned)
```

**Why?** Ensure bilateral symmetry for lateral branches.

---

## 6. Lateral Leaf Orientation

### Problem
CSV orientation for lateral leaves is relative to **world frame**, but should be relative to **parent branch**.

### Solution: Coaxial Alignment

```python
# Lateral leaf tilt relative to branch
tilt_deg = 35.0  # Inclined upward like "/"

# Not perpendicular (75°) like "_/"
```

**Why?** User feedback: "possiamo farli inclinare piu verso su" (more coaxial with branch).

---

## Summary Table

| Modification | CSV Data | ExporterV2 Output | Reason |
|--------------|----------|-------------------|--------|
| Trunk leaf angle | `ccw_orientation` | Use CSV or phyllotaxis fallback | Handle missing data |
| Trunk leaf count | All organs | Filter to opposite pairs | Tomato morphology |
| Lateral branch count | All organs (0-4) | Filter to [0, 1] | Tomato morphology |
| Lateral branch angle | N/A | `rot_base_deg` + jitter | Natural variance |
| Collision check | ❌ | ✅ 60° min separation | Prevent simulation failure |
| Lateral leaf count | 1 or 2 | Always 2 (clone if needed) | Symmetry |
| Lateral leaf tilt | N/A | 35° (coaxial) | User preference |

---

## Configuration

All modifications controlled via **profiles** (`profiles/tomato_default.py`):

```python
TOMATO_PROFILE = {
    "lateral_branches": {
        "organ_indices": [0, 1],
        "rot_jitter_deg": 45.0,
        "min_angle_separation_deg": 60.0,
    },
    "trunk_leaves": {
        "filter_strategy": "opposite_pairs_180deg",
        "phyllotaxis_deg": 137.5,
    },
    "lateral_leaves": {
        "clone_missing": True,
        "tilt_deg": 35.0,
    },
}
```

**See also:**
- [04_collision_checks.md](04_collision_checks.md) - Detailed collision system
- [06_implementation_notes.md](06_implementation_notes.md) - Lessons learned
