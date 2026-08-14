# Locked Joint Comparison Test

## Purpose

Test if the instability problem is caused by:
1. **Joint flexibility/drives** → locked version should be completely stable
2. **Geometry/collisions/USD structure** → locked version will also show problems

## Files Generated

- `test2_stem_1petiole_LOCKED.usda` - USD with all FixedJoint (completely rigid)
- `_load_test2_locked.py` - Isaac Sim loader script

## How to Test

### 1. Test LOCKED version (rigid joints)

```bash
cd ~/isaacsim && ./python.sh /home/alessandro/isaacsim/autotom_digital_twin/src/experiments/recursive_tree/tests/_load_test2_locked.py
```

**What to observe:**
- Before PLAY: Stem vertical, petiole at 45° angle (like this: `|/`)
- Press PLAY
- **Expected:** Geometry should NOT move at all (completely frozen)
- **If it moves/changes:** Problem is in USD structure, not joint tuning

### 2. Test FLEXIBLE version (normal D6 joints)

```bash
cd ~/isaacsim && ./python.sh /home/alessandro/isaacsim/autotom_digital_twin/src/experiments/recursive_tree/tests/_load_baseline.py
```

**What to observe:**
- Before PLAY: Same geometry `|/`
- Press PLAY
- **If it changes to Y shape:** Joint drives pulling to wrong target (should be fixed now)
- **If it jitters/oscillates:** Joint stiffness/damping needs tuning OR solver iterations too low

## Diagnosis Guide

| LOCKED Behavior | FLEXIBLE Behavior | Diagnosis | Solution |
|----------------|-------------------|-----------|----------|
| Stable (frozen) | Changes to Y | Drive target wrong | ✅ Fixed (targetPosition=0) |
| Stable (frozen) | Jitters/oscillates | Drive tuning needed | Increase solver iterations or adjust K/D |
| Moves/unstable | Moves/unstable | USD structure problem | Check joint frames, collision filtering |
| Stable (frozen) | Stable with droop | Working correctly! | This is expected (gravity + flexibility) |

## Key Differences

### LOCKED (test2_stem_1petiole_LOCKED.usda)
- All joints: `PhysicsFixedJoint`
- No drive settings (stiffness/damping/target)
- Should be completely rigid
- Collision filtering: ✅ YES

### FLEXIBLE (solver_baseline_pos64_vel8.usda)
- All joints: `PhysicsJoint` with D6 drives
- Drive settings: stiffness, damping, targetPosition=0
- Has controlled flexibility
- Collision filtering: ✅ YES

## Next Steps Based on Results

### If LOCKED is stable:
✅ USD structure is correct  
→ Focus on joint tuning:
  - Try higher solver iterations (test_solver_settings.py)
  - Adjust stiffness/damping values
  - Check drive limits

### If LOCKED is unstable:
❌ Deeper problem exists  
→ Check:
  - Joint local frames (localPos0/localRot0)
  - Mass distribution
  - Collision filtering (should be there now)
  - Attachment point positioning

## Regenerate

To regenerate the locked version:

```bash
uv run src/experiments/recursive_tree/tests/test_locked_comparison.py
```
