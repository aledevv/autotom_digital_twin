# Implementation Notes

Lessons learned, tricks, and design decisions during ExporterV2 development.

---

## Key Lessons

### 1. Filtering Needs Siblings, Not Just Parent-Child

**Problem:**
Initial filtering only checked parent-child relationship:
```python
# ❌ Wrong: Only looks at parent
def filter_leaves(parent, leaves):
    return [leaf for leaf in leaves if leaf.parent == parent]
```

**Issue:** Can't identify "opposite pairs" without sibling context.

**Solution:**
Include siblings in filtering logic:
```python
# ✅ Correct: Looks at all leaves on same rank
def filter_opposite_pairs(all_leaves, rank):
    leaves_at_rank = [l for l in all_leaves if l.rank == rank]
    # Now can check angular separation between siblings
    return select_180deg_pairs(leaves_at_rank)
```

**Why it matters:** "opposite_pairs_180deg" strategy requires comparing multiple organs at same rank.

---

### 2. Collision Check Needs rank±1, Not Just Same Rank

**Problem:**
Initial collision check only compared branches at same rank:
```python
# ❌ Incomplete
for existing in same_rank_branches:
    check_separation(new_angle, existing.rotation)
```

**Issue:** Branches at adjacent ranks can also collide due to vertical spacing.

**Solution:**
Check same rank + adjacent ranks:
```python
# ✅ Complete
for existing in existing_branches:
    if abs(existing.rank - current_rank) <= 1:
        check_separation(new_angle, existing.rotation)
```

**User feedback:** "60 direi, ma vanno controllati rami di rank identico, sopra e sotto"

---

### 3. Leaf Orientation on Laterals Relative to Branch, Not World

**Problem:**
Initial lateral leaf orientation was perpendicular to branch:
```python
# ❌ Too perpendicular (like "_/")
tilt_deg = 75.0
```

**Issue:** Leaves looked unnatural, like horizontal shelves.

**Solution:**
Make leaves more coaxial with branch:
```python
# ✅ Inclined upward (like "/")
tilt_deg = 35.0
```

**User feedback:** "possiamo farli inclinare piu verso su. _/ <--- questo e' l'inizio di lat leaf"

---

### 4. Path Resolution for Nested Modules

**Problem:**
CSV files referenced as `../data/graph_day_1.csv`, but working directory varies.

**Solution:**
Resolve paths relative to module location:
```python
# adapters/groimp_csv/parser.py
import os
from pathlib import Path

MODULE_DIR = Path(__file__).parent
DATA_DIR = MODULE_DIR.parent.parent / "data"
csv_path = DATA_DIR / f"graph_day_{day}.csv"
```

**Why it matters:** Works regardless of where script is called from.

---

### 5. Lazy Imports to Avoid pxr Dependency

**Problem:**
`from pxr import Usd` in top-level imports breaks tests outside Isaac Sim.

**Solution:**
Import `pxr` only when needed:
```python
# ❌ Breaks tests
from pxr import Usd

def build_stage():
    stage = Usd.Stage.CreateNew("output.usda")
```

```python
# ✅ Works everywhere
def build_stage():
    from pxr import Usd  # Lazy import
    stage = Usd.Stage.CreateNew("output.usda")
```

**Why it matters:** Enables testing without full Isaac Sim installation.

---

### 6. Profile-Driven Design for Reusability

**Problem:**
Hardcoded tomato-specific logic in parser:
```python
# ❌ Not reusable
def load_leaves(csv_path):
    leaves = parse_csv(csv_path)
    return leaves[::2]  # Hardcoded: every other leaf
```

**Solution:**
Move cultivar logic to profiles:
```python
# ✅ Reusable
def load_leaves(csv_path, profile):
    leaves = parse_csv(csv_path)
    if profile["filter_strategy"] == "opposite_pairs_180deg":
        return filter_opposite_pairs(leaves)
    return leaves
```

**Why it matters:** Can support multiple cultivars without changing core code.

---

### 7. Deterministic Randomness (Seed-Based)

**Problem:**
Random jitter different every run → hard to debug.

**Solution:**
Seed random generator:
```python
import random
random.seed(42)  # Reproducible

# Or per-branch seeding
seed = hash(f"{branch_id}_{rank}")
random.seed(seed)
```

**Trade-off:** Debugging easier, but all plants identical. Could add `--random-seed` flag.

---

## Design Patterns

### 1. Adapter Pattern
Separate data-source logic (CSV) from tree-building logic (USD).

**Benefits:**
- Can add new data sources without changing core
- Core remains generic and reusable

### 2. Strategy Pattern
Different filtering strategies via profiles:
- `opposite_pairs_180deg`
- `all_organs`
- `first_n`

**Benefits:**
- Configurability without code changes
- Easy A/B testing

### 3. Builder Pattern
Stage construction in `core/usd/stage.py`:
```python
stage, root = build_stage()
add_trunk(stage, root, trunk_config)
add_branches(stage, root, branch_configs)
add_leaves(stage, root, leaf_configs)
```

**Benefits:**
- Clear separation of concerns
- Testable components

---

## Performance Considerations

### 1. Collision Check Complexity
```python
# O(n²) in worst case
for new_branch in branches:
    for existing in existing_branches:
        check_separation(new_branch, existing)
```

**Mitigation:** Only check rank±1 (reduces n significantly).

### 2. JSON vs USD Generation
- **JSON export:** ~0.1s (fast, no Isaac Sim)
- **USD generation:** ~15s (requires Isaac Sim)

**Use JSON for rapid iteration, USD for final output.**

---

## Future Improvements

1. **Bounding box collision** (current: angular only)
2. **Adaptive jitter** (reduce range if collisions frequent)
3. **Visual debug mode** (render collision zones)
4. **Profile validation** (catch config errors early)
5. **Parallel USD generation** (multi-threading for large plants)

---

## Common Pitfalls

### ❌ Assuming CSV completeness
CSV may have missing organs → always check and handle gracefully.

### ❌ World-frame orientation
Lateral elements need parent-relative orientation, not world-relative.

### ❌ Forgetting rank±1 checks
Collision detection must consider vertical neighbors.

### ❌ Hardcoding cultivar logic
Always use profiles for plant-specific behavior.

---

**See also:**
- [01_architecture.md](01_architecture.md) - Overall design
- [03_csv_modifications.md](03_csv_modifications.md) - Why modifications needed
- [07_troubleshooting.md](07_troubleshooting.md) - Fixing common issues
