# Joint-Budget Optimization - Quick Start Guide

> **Quick reference** per iniziare l'implementazione e usare il sistema di ottimizzazione.

## Per Iniziare l'Implementazione

### 1. Leggi i Documenti

1. **OPTIMIZATION_IMPLEMENTATION_PLAN.md** - Checklist dettagliata di tutte le 12 task
2. **OPTIMIZATION_DESIGN.md** - Architettura tecnica e specifiche implementative
3. **Questo file** - Quick reference per iniziare

### 2. Setup Iniziale

```bash
# Crea la struttura cartelle (Task 1)
cd src/exporterV2/core
mkdir -p optimizations/{techniques,collision,geometry,tests/visual_validation}
```

### 3. Ordine di Implementazione Consigliato

**Phase 1 - Infrastructure** (Task 1-3):
- Task 1: Setup base (optimizer.py, base.py, budget_config.yaml)
- Task 2: Collision detection (sphere, aabb, broad_phase)
- Task 3: Geometry remapping

**Phase 2 - Techniques** (Task 4-8):
- Task 4: Petiole lock (più semplice, no remapping)
- Task 5: Lateral reduce (medio, no remapping)
- Task 6: Stem collapse (complesso, usa remapping + collision)
- Task 7: Truss static (placeholder se truss non implementato)
- Task 8: Leaf branch reduce

**Phase 3 - Integration** (Task 9-12):
- Task 9: Integration tests
- Task 10: Visual validation
- Task 11: Parse pipeline integration
- Task 12: Documentation

---

## Per Usare il Sistema (Dopo Implementazione)

### Uso Base

```python
from exporterV2.core.optimizations import BudgetOptimizer

# Load optimizer with default config
optimizer = BudgetOptimizer()

# Optimize branches configuration
optimized_branches, report = optimizer.optimize(branches)

# Print report
print(report)
# Output:
# ========================================
# Joint-Budget Optimization Report
# ========================================
# Original joints: 320
# Budget: 250
# Lower bound: 45
# 
# Techniques applied:
#   1. Petiole Lock: 320 → 290 (-30 joints)
#   2. Lateral Reduce: 290 → 260 (-30 joints)
#   3. Stem Collapse: 260 → 250 (-10 joints)
# 
# Final joints: 250 ✓
# Total reduction: -70 joints (21.9%)
# ========================================
```

### Con Parse Pipeline

```python
# In main.py
branches, json_path = parse_csv_to_branches(day=50, optimize=True)
```

### CLI

```bash
# Run with optimization
./run_mainV2.sh --day 50 --optimize

# Output includes optimization report before USD generation
```

---

## File Structure Reference

```
exporterV2/core/optimizations/
├── __init__.py                 # Exports: BudgetOptimizer
├── optimizer.py                # Main orchestrator class
├── budget_config.yaml          # Configuration (edit this!)
│
├── techniques/
│   ├── base.py                 # Abstract base: OptimizationTechnique
│   ├── petiole_lock.py         # Priority 1
│   ├── lateral_reduce.py       # Priority 2
│   ├── stem_collapse.py        # Priority 3
│   ├── truss_static.py         # Priority 4
│   └── leaf_branch_reduce.py   # Priority 5
│
├── collision/
│   ├── sphere.py               # Stage 1: Fast pre-check
│   ├── aabb.py                 # Stage 2: Precision
│   └── broad_phase.py          # Orchestration
│
├── geometry/
│   ├── remapping.py            # Attachment height remapping
│   └── bounds.py               # Bounding volume helpers
│
└── tests/
    ├── test_optimizer.py
    ├── test_*.py               # Per-technique tests
    └── visual_validation/
        └── run_visual_test.py
```

---

## Configuration Quick Ref

### budget_config.yaml Key Sections

```yaml
budget:
  max_joints: 250              # ← EDIT THIS for your hardware

structural_limits:
  trunk: { min_links: 1 }
  lateral_branch: { min_links: 1 }
  petiole: { min_links: 1 }
  # ...

techniques:
  - id: "petiole_lock"
    priority: 1                # ← Lower number = applied first
    enabled: true              # ← Set false to disable technique
    params:
      # technique-specific params
```

### Enable/Disable Techniques

```yaml
techniques:
  - id: "stem_collapse"
    enabled: false             # Disable stem collapse
```

### Adjust Collision Safety Margin

```yaml
techniques:
  - id: "stem_collapse"
    params:
      collision_check:
        safety_margin: 0.02    # Increase for more conservative checks
```

---

## Testing Reference

### Run All Tests

```bash
cd src/exporterV2
pytest core/optimizations/tests/ -v
```

### Run Specific Technique Test

```bash
pytest core/optimizations/tests/test_stem_collapse.py -v
```

### Run Visual Validation

```bash
# Generate test USD files and load in IsaacSim
python core/optimizations/tests/visual_validation/run_visual_test.py
```

---

## Common Issues & Solutions

### Issue: "Budget impossible to meet (lower bound > budget)"

**Causa**: La pianta richiede più joints del budget anche nella versione minima.

**Soluzioni**:
1. Aumenta `max_joints` in config
2. Riduci complessità pianta (meno foglie, meno lateral branches)
3. Usa profilo semplificato (es. `SIMPLE_PLANT_PROFILE`)

### Issue: "Collision detected after remapping"

**Causa**: Stem collapse ha creato overlap tra branches.

**Soluzioni**:
1. Aumenta `safety_margin` in collision config
2. Disabilita `stem_collapse` temporaneamente
3. Debug con visual validation per vedere overlap

### Issue: "Technique X has no effect"

**Causa**: Technique `can_apply()` ritorna False.

**Debug**:
```python
# In optimizer, add debug print:
if not technique.can_apply(branches, current_joints, budget):
    print(f"[DEBUG] {technique.name} cannot apply: <reason>")
```

---

## Development Workflow

### Starting a New Task

1. **Check Implementation Plan**: Verifica status e dipendenze
2. **Read Design Doc**: Sezione relativa al task
3. **Write Tests First**: TDD approach quando possibile
4. **Implement**: Segui specifiche nel design doc
5. **Run Tests**: Assicurati passino tutti
6. **Update Plan**: Marca task come ✅ DONE

### Committing Changes

```bash
# After completing Task N
git add .
git commit -m "feat(optimizations): Complete Task N - <description>

- Implemented <component>
- Added tests for <feature>
- All tests passing

Refs: OPTIMIZATION_IMPLEMENTATION_PLAN.md Task N"
```

### Debugging Tips

1. **Enable debug logging**:
   ```yaml
   # budget_config.yaml
   logging:
     level: "DEBUG"
     show_collision_details: true
   ```

2. **Use pytest with prints**:
   ```bash
   pytest -s tests/test_optimizer.py  # -s shows print statements
   ```

3. **Visual validation**:
   - Genera USD prima e dopo ottimizzazione
   - Carica in IsaacSim side-by-side
   - Verifica posizioni branches preservate

---

## Key Algorithms Quick Ref

### Lower Bound Calculation

```python
lower_bound = (
    n_trunk * 1 +           # Min 1 link per trunk
    n_lateral * 1 +         # Min 1 link per lateral branch
    n_petiole * 1 +         # Min 1 link per petiole
    n_truss * 1             # Min 1 link per truss
    # petiolules and rachis can be 0
)
```

### Attachment Remapping

```
absolute_height_before = sum(heights[0:original_link_idx])
new_link_idx = int(absolute_height_before / new_segment_height)
offset_z = absolute_height_before - (new_link_idx * new_segment_height)
```

### Collision Check

```
Stage 1 (Fast):
  distance = |center1 - center2|
  overlap = distance < (radius1 + radius2 + margin)

Stage 2 (Precision):
  For each axis (x, y, z):
    overlap_axis = (min1 <= max2) AND (max1 >= min2)
  overlap_total = overlap_x AND overlap_y AND overlap_z
```

---

## Next Steps

✅ **Ready to Start**: Go to Task 1 in `OPTIMIZATION_IMPLEMENTATION_PLAN.md`

📖 **Need More Details**: Read `OPTIMIZATION_DESIGN.md`

❓ **Questions**: Check design decisions section in design doc

🐛 **Issues**: Add to GitHub issues or update implementation plan

---

**Last Updated**: YYYY-MM-DD
