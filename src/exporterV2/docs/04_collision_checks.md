# Collision Checks

Anti-collision system for lateral branches to prevent simulation failures.

---

## Problem

Random rotation jitter can cause **lateral branches to overlap**:

```
     /    ← Branch 1 (201°)
    |
   /      ← Branch 0 (-23°)
  |
```

If angle separation < threshold → **cylinders intersect** → **PhysX explosion** 💥

---

## Solution: Geometric Validation

### Algorithm

```python
def generate_safe_rotation(
    base_angle: float,
    jitter_range: float,
    existing_branches: List[Dict],
    min_separation: float = 60.0,
    max_attempts: int = 100
) -> Optional[float]:
    """
    Generate random rotation with collision check.
    
    Checks against:
    - Same rank branches
    - Adjacent rank branches (rank±1)
    """
    
    for attempt in range(max_attempts):
        # Random jitter
        angle = base_angle + random.uniform(-jitter_range, jitter_range)
        
        # Check all existing branches
        collision_free = True
        for existing in existing_branches:
            # Check same rank + rank±1
            if abs(existing.rank - current_rank) <= 1:
                separation = angular_distance(angle, existing.rotation)
                if separation < min_separation:
                    collision_free = False
                    break
        
        if collision_free:
            return angle  # ✅ Safe
    
    return None  # ❌ Failed after max_attempts
```

---

## Parameters

### From Profile (`profiles/tomato_default.py`)

```python
"lateral_branches": {
    "rot_jitter_deg": 45.0,              # Max random deviation
    "min_angle_separation_deg": 60.0,    # Min safety margin
}
```

### Why 60°?

User requirement: *"60 direi, ma vanno controllati rami di rank identico, sopra e sotto"*

**Trade-off:**
- Too small (e.g., 30°) → Risk of collision
- Too large (e.g., 90°) → Limited jitter freedom

---

## Rank-Based Checks

### Why Check rank±1?

Branches at **adjacent ranks** can also collide due to internode spacing:

```
Rank 3:  ──────/     ← Branch at 45°
              |
Rank 2:  ────/       ← Branch at 50° → Only 5° separation! ❌
            |
```

### Implementation

```python
# Check same rank
for existing in same_rank_branches:
    check_separation(new_angle, existing.rotation)

# Check rank-1 (above)
for existing in rank_minus_1_branches:
    check_separation(new_angle, existing.rotation)

# Check rank+1 (below)
for existing in rank_plus_1_branches:
    check_separation(new_angle, existing.rotation)
```

---

## Test Results

### Day 100 Validation

Run: `python3 src/exporterV2/tests/test_collision_geometry.py`

**Sample output:**

```
Lateral Branch Collision Analysis:
-----------------------------------
Parent rank 0:
  Branch 0: rotation = 25.0°
  Branch 1: rotation = 206.7°
  Separation: 178.3° ✅

Parent rank 1:
  Branch 0: rotation = 90.0°
  Branch 1: rotation = 271.7°
  Separation: 178.3° ✅

Parent rank 2:
  Branch 0: rotation = 12.5°
  Branch 1: rotation = 161.2°
  Separation: 148.7° ✅

Parent rank 3:
  Branch 0: rotation = 77.5°
  Branch 1: rotation = 226.2°
  Separation: 148.7° ✅

✅ PASS: All separations ≥ 60.0°
✅ PASS: Rotation variance (8 unique angles)
✅ PASS: No bounding box overlaps
```

---

## Edge Cases

### 1. All Attempts Fail

If 100 attempts don't find safe angle:

```python
if safe_angle is None:
    logger.warning(f"Could not find collision-free angle for branch {id}")
    # Skip this branch (don't add to output)
```

**Rare:** Only if existing branches occupy most of 360° space.

### 2. First Rank (No Previous Branches)

```python
if not existing_branches:
    # No collision check needed
    return base_angle + random_jitter
```

### 3. Opposite Pairs

For organ_indices = [0, 1] with rot_base_deg = [0°, 180°]:

```python
# Branch 0: Check against rank±1 only
# Branch 1: Check against rank±1 + Branch 0 (same rank)
```

Ensures **both branches** in pair are collision-free.

---

## Geometric Validation

### Angular Separation

```python
def angular_distance(angle1: float, angle2: float) -> float:
    """Shortest angular distance between two angles."""
    diff = abs(angle1 - angle2) % 360
    return min(diff, 360 - diff)
```

**Example:**
- `angular_distance(10°, 350°)` = 20° (not 340°)

### Bounding Box Check (Future)

Currently checks **angular separation only**. Could add:

```python
def check_cylinder_overlap(branch1, branch2) -> bool:
    """Check if cylinder bounding boxes intersect."""
    # Consider position, length, radius
    # More accurate but slower
```

---

## Configuration

Adjust safety margin in profile:

```python
# Conservative (less jitter)
"min_angle_separation_deg": 90.0

# Aggressive (more jitter, higher risk)
"min_angle_separation_deg": 45.0

# Balanced (current)
"min_angle_separation_deg": 60.0
```

---

## Limitations

1. **Angular only** - Doesn't account for branch length/radius
2. **Local check** - Doesn't verify global plant geometry
3. **Static** - No dynamic collision during simulation

**Mitigation:** Geometric tests verify output (`test_collision_geometry.py`).

---

## Future Improvements

1. **3D bounding box checks** (position + orientation)
2. **PhysX collision query** (pre-simulation validation)
3. **Adaptive jitter** (reduce range if collisions frequent)
4. **Visual debug mode** (render collision zones)

---

**See also:**
- [03_csv_modifications.md](03_csv_modifications.md) - Why jitter is needed
- [05_testing.md](05_testing.md) - How to run tests
- [07_troubleshooting.md](07_troubleshooting.md) - If collisions occur
