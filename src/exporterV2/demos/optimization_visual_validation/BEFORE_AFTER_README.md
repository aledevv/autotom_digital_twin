# Before/After Optimization Comparison

Visual comparison of optimization effects on real CSV plant (day 100).

## Quick Start

```bash
cd /home/alessandro/isaacsim/autotom_digital_twin/src/exporterV2/demos/optimization_visual_validation

# 1. Generate USD files (if not already generated)
uv run python generate_before_after_usd.py

# 2. Load side-by-side in Isaac Sim
./load_before_after.sh
```

## Files Generated

The script creates two USD files in `usd_output_before_after/`:

1. **`day_100_baseline.usda`** (165 joints, 136 branches)
   - Original plant from CSV day 100
   - No optimization applied
   - Full detail: trunk 10 links, separate petiole+rachis

2. **`day_100_optimized_budget_50.usda`** (121 joints, 119 branches)
   - Optimized with aggressive budget (50 joints target)
   - Techniques applied:
     - **Stem Collapse**: trunk 10 → 3 links (-7 joints)
     - **Leaf Branch Reduce**: merged petiole+rachis (-37 joints)
   - Total reduction: 44 joints (26.7%)

## Visual Differences

When loaded side-by-side, you'll see:

| Feature | Baseline (LEFT) | Optimized (RIGHT) |
|---------|----------------|-------------------|
| **Trunk** | 10 thin segments | 3 thick segments |
| **Trunk height** | 0.49m total | 0.49m total (preserved) |
| **Leaves** | Petiole + Rachis (2-3 links) | Merged single segment |
| **Leaf positions** | Original | Preserved at same heights |
| **Lateral branches** | 1 link each | 1 link each (unchanged) |

## Optimization Strategy

Budget target: **50 joints** (aggressive, forces maximum reduction)

**Techniques NOT applied** (could reduce further):
- Petiole Lock: D6 petiolules → Fixed joints (~-48 joints)
- Lateral Reduce: Not applicable (already 1 link each)

**Why over budget (121 > 50)?**
- Lower bound: 28 joints (structural minimum)
- Achieved: 121 joints
- To reach 50, need Petiole Lock technique

## Commands

### Generate USD
```bash
uv run python generate_before_after_usd.py
```

### Load in Isaac Sim (side-by-side)
```bash
./load_before_after.sh
```

### Load individually
```bash
# Baseline only
~/isaacsim/python.sh -m isaacsim 'usd_output_before_after/day_100_baseline.usda'

# Optimized only
~/isaacsim/python.sh -m isaacsim 'usd_output_before_after/day_100_optimized_budget_50.usda'
```

## Customization

To change the budget or day, edit `generate_before_after_usd.py`:

```python
DAY = 100                    # Change day number
AGGRESSIVE_BUDGET = 50       # Change target budget
```

Then regenerate:
```bash
uv run python generate_before_after_usd.py
```

## Integration with Main Pipeline

This demonstrates Task 11 integration. The same optimization can be run via:

```bash
# Use main.py with --optimize flag
./run_mainV2.sh --day 100 --optimize
```

The `--optimize` flag applies the same `BudgetOptimizer` with budget from `budget_config.yaml` (default: 250 joints).
