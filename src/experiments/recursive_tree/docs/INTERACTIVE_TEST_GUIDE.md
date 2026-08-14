# Interactive Convergence Test - Quick Guide

## Start Testing

```bash
cd /home/alessandro/isaacsim/autotom_digital_twin
./run_experiment.sh recursive_tree tests/test_interactive_convergence.py
```

---

## Workflow

### 1. Config Loads
- Isaac Sim opens with the test configuration
- Terminal shows config name, DOFs, USD file

### 2. Observe Simulation
- **Press PLAY ▶️** in Isaac Sim
- Watch for 10-30 seconds
- Look for these behaviors:

### 3. What to Look For

#### ✅ STABLE (Type 1)
- Plant settles within **< 5 seconds**
- **No visible oscillations** after settling
- Branches stay in resting position
- Root stays fixed (no drift)
- **Example**: Baseline tomato, conservative L/D ratios

#### ⚠️ MARGINAL (Type 2)
- Takes **5-20 seconds** to settle
- **Some oscillations** but eventually stops
- Small drift (< 2cm) acceptable
- Borderline usable
- **Example**: L/D=8, moderate tilt angles

#### ❌ UNSTABLE (Type 3)
- **Continuous jittering/vibration** (> 20s)
- Never settles or keeps oscillating
- Significant drift (> 5cm)
- Branches "bouncing" indefinitely
- May collapse or explode
- **Example**: L/D=12, horizontal branches (90°), very thin petiolules

### 4. Classify in Terminal
- Return to terminal
- **Press ENTER** when ready
- Choose classification: 1/2/3
- Add optional notes (e.g., "horizontal branch droops but stable")

### 5. Repeat
- Next config loads automatically
- Results saved after each test (safe to interrupt)

---

## Tips

### Visual Cues for Classification

**STABLE**:
- "Dead plant" - no movement after 5s
- Gravity pulls it down → stops → done

**MARGINAL**:
- "Swaying in breeze" - moves slowly but decreasing
- Takes 10-20s to fully stop

**UNSTABLE**:
- "Earthquake" - continuous shaking
- "Jello" - wobbly, never stops
- "Explosion" - flies apart

### What to Test

**Priority configs** (test these first):
1. `baseline_tomato_realistic` - should be STABLE
2. `petiolule_ld_8` - probably MARGINAL
3. `petiolule_ld_10` - probably UNSTABLE
4. `petiole_tilt_90` - probably UNSTABLE (horizontal)
5. `six_petioles_50_links` - check complexity handling

### Common Issues

**Config looks stable but...**:
- Check if branches are slowly drifting downward
- Zoom in on petiolules (thin tips) - are they vibrating?
- If unsure → classify as MARGINAL

**Plant explodes immediately**:
- Classify as UNSTABLE
- Note: "immediate explosion" in comments

**Simulation freezes**:
- Ctrl+C to skip config
- Classified as SKIPPED automatically

---

## Resume Testing

If interrupted, just re-run the command:
```bash
./run_experiment.sh recursive_tree tests/test_interactive_convergence.py
```

Already tested configs are **automatically skipped**. Progress saved in:
```
tests/convergence_results_interactive.json
```

---

## Expected Classifications (Predictions)

Based on scalability analysis:

### Likely STABLE ✅
- `baseline_tomato_realistic` (L/D=5, balanced)
- `petiole_tilt_30` (low droop)
- `petiolule_ld_8` (borderline but should hold)

### Likely MARGINAL ⚠️
- `mixed_angles` (varied stresses)
- `radius_ratio_2_5` (thinner but manageable)
- `min_radius_2mm_world` (at collision threshold)

### Likely UNSTABLE ❌
- `petiolule_ld_10` (droop ~55mm)
- `petiolule_ld_12` (droop ~155mm)
- `petiole_tilt_90` (horizontal → max droop)
- `radius_ratio_3_5` (too thin, 1.3mm world)
- `min_radius_1mm_world` (below collision threshold)

### Unknown ❓
- `six_petioles_50_links` (59 links - complexity test)
- `five_petioles_50_links` (50 links - alt complexity)
- `petiole_ld_10` (thicker branch with high L/D)

---

## Output Format

After all tests, you'll see:

```
================================================================================
                            FINAL SUMMARY
================================================================================

✅ STABLE:   3
⚠️  MARGINAL: 4
❌ UNSTABLE: 6
⏭️  SKIPPED:  2
📊 TESTED:   13/15

Config                              Status       Duration(s)  Notes
--------------------------------------------------------------------------------
baseline_tomato_realistic           ✅ STABLE     12.3         Quick settle
petiolule_ld_8                      ⚠️  MARGINAL  18.5         Slow but converges
petiolule_ld_10                     ❌ UNSTABLE   25.0         Continuous jitter
...

Results saved: tests/convergence_results_interactive.json
```

---

## Next Steps

After classifying all configs:

### Task 4: Force Resistance Tests
Use **STABLE configs only** for robot interaction tests:
- 1N, 5N, 10N impulse forces
- Sustained external forces
- Recovery time after perturbation

### If No Stable Configs
- Adjust parameters (reduce L/D, increase radius)
- Create new "ultra-safe" baseline
- Test with stricter PhysX solver settings

---

## Keyboard Shortcuts

- **PLAY**: Space or timeline play button
- **PAUSE**: Space or timeline pause button
- **STOP**: Stop button (resets simulation)
- **ESC**: Exit Isaac Sim (will prompt to save results)
- **Ctrl+C**: Skip current config (in terminal)

---

## Troubleshooting

**Isaac Sim doesn't open**:
```bash
# Check if already running
ps aux | grep isaac-sim
# Kill if stuck
pkill -f isaac-sim
```

**Terminal doesn't show prompt**:
- Check Isaac Sim window - might need to focus terminal
- Press Enter in terminal to trigger prompt

**Results not saving**:
- Check file permissions in `tests/` directory
- Results should save after EACH test

**Wrong config displayed**:
- Check USD file name in terminal output
- Verify `scalability_usds/` directory has correct files

---

**Ready to start? Run the script and follow terminal instructions!** 🍅
