# Optimization Technique Combinations — Visual Comparison Guide

This test suite generates 8 USD files with different technique combinations,
all compared against a common baseline. Use this to visually verify each 
technique's effect in Isaac Sim.

## Test Combinations

| ID | Label | Short | Techniques | D6 Joints | Δ from Baseline |
|----|-------|-------|------------|-----------|-----------------|
| 0 | Baseline | `baseline` | (none) | 99 | — |
| 1 | Petiole Lock | `P` | petiole_lock | 75 | -24 |
| 2 | Lateral Reduce | `L` | lateral_reduce | 79 | -20 |
| 3 | Stem Collapse | `S` | stem_collapse | 92 | -7 |
| 4 | Leaf Reduce | `F` | leaf_reduce | 67 | -32 |
| 5 | P+L | `P+L` | petiole_lock + lateral_reduce | 55 | -44 |
| 6 | P+F | `P+F` | petiole_lock + leaf_reduce | 43 | -56 |
| 7 | Full Optimization | `Full` | All 4 techniques | 16 | -83 |

## USD Files

All files are in `usd_output_combinations/`:

- `combo_0_baseline.usda` — Baseline (no optimization)
- `combo_1_p.usda` — Petiole Lock only
- `combo_2_l.usda` — Lateral Reduce only
- `combo_3_s.usda` — Stem Collapse only
- `combo_4_f.usda` — Leaf Reduce only
- `combo_5_p_l.usda` — Petiole Lock + Lateral Reduce
- `combo_6_p_f.usda` — Petiole Lock + Leaf Reduce
- `combo_7_full.usda` — All techniques applied

## How to Load in Isaac Sim

### Option 1: Single USD

Load one combination to inspect:

```bash
~/isaacsim/python.sh -m isaacsim 'src/exporterV2/demos/optimization_visual_validation/usd_output_combinations/combo_0_baseline.usda'
```

Replace `combo_0_baseline.usda` with any combination filename.

### Option 2: Side-by-Side Comparison (Recommended)

Use the loader script to load baseline + one combination side by side:

```bash
cd /home/alessandro/isaacsim/autotom_digital_twin
~/isaacsim/python.sh src/exporterV2/demos/optimization_visual_validation/load_combination_isaacsim.py --combo 1
```

Replace `1` with the combination ID from the table above (0-7).

The script loads:
- **Left**: Baseline (always)
- **Right**: Selected combination

This lets you directly compare the effect of the technique(s).

## What to Verify Per Combination

### Combo 0 — Baseline
- ✓ All components present (trunk, laterals, leaves, petiolules)
- ✓ All joints articulated (D6)

### Combo 1 — P (Petiole Lock)
- ✓ Petiolules are Fixed joints (don't oscillate in simulation)
- ✓ Geometry identical to baseline
- ✓ D6 joints: 99 → 75 (-24 petiolules)

### Combo 2 — L (Lateral Reduce)
- ✓ Lateral branches have 1 segment (vs 5 in baseline)
- ✓ Laterals appear more rigid
- ✓ Leaves on laterals still attached correctly
- ✓ D6 joints: 99 → 79 (-20 lateral segments)

### Combo 3 — S (Stem Collapse)
- ✓ Trunk has 3 segments (vs 10 in baseline)
- ✓ Lateral branches still distributed along trunk (not all at top)
- ✓ Trunk appears "chunkier" (fewer segments, same height)
- ✓ D6 joints: 99 → 92 (-7 trunk segments)

### Combo 4 — F (Leaf Reduce)
- ✓ Each leaf has 1 segment (petiole+rachis merged)
- ✓ Petiolules distributed along merged segment (not all at top)
- ✓ Leaves appear as rigid rods
- ✓ D6 joints: 99 → 67 (-32 rachis segments)

### Combo 5 — P+L
- ✓ Combines Combo 1 + Combo 2 effects
- ✓ Petiolules Fixed + laterals reduced
- ✓ D6 joints: 99 → 55 (-44)

### Combo 6 — P+F
- ✓ Combines Combo 1 + Combo 4 effects
- ✓ Petiolules Fixed + leaves merged
- ✓ D6 joints: 99 → 43 (-56)

### Combo 7 — Full
- ✓ All techniques applied
- ✓ Highly simplified structure
- ✓ D6 joints: 99 → 16 (-83, 84% reduction!)

## Common Issues to Check

If you see any of these, there's a bug:

- **Branches disappear** → technique removed branches incorrectly
- **Plant collapses in simulation** → attachment remapping failed
- **Petiolules in wrong position** → `attach_frac` not propagated
- **Leaves all bunched at top** → remapping didn't preserve absolute height
- **Trunk too short/long** → height recalculation wrong

## Regenerate USD Files

If you modify the techniques or plant structure:

```bash
cd /home/alessandro/isaacsim/autotom_digital_twin
uv run python src/exporterV2/demos/optimization_visual_validation/generate_combinations_usd.py
```

This will regenerate all 8 USD files in `usd_output_combinations/`.

## Run Non-Visual Tests

Before visual inspection, verify joint counts and structure:

```bash
uv run pytest src/exporterV2/demos/optimization_visual_validation/validate_combinations.py -v
```

This checks:
- Joint counts match expected values
- No orphaned branches
- `attach_frac` values correct
- Trunk/lateral/leaf structure preserved
