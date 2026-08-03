# Collision Filtering Verification

## Test File
`test2_stem_1petiole_DAMPING_FIX.usda`

## Verification Results ✅

### Filtering Count
- **Xform with PhysicsFilteredPairsAPI**: 7 ✅
- **Total filteredPairs relationships**: 14 (7 Xform + 7 Cylinder) ✅

### Dual-Level Filtering Confirmed

#### Example 1: Stem Internal Joint (Link 01 → Link 02)
```
✅ stem_Link_02 (Xform) filters: </World/Stem/stem_Link_01>
✅ stem_Link_02/Cylinder filters: </World/Stem/stem_Link_01/Cylinder>
```

#### Example 2: Attachment Joint (stem → petiole)
```
✅ petiole_1_Link_01 (Xform) filters: </World/Stem/stem_Link_03>
✅ petiole_1_Link_01/Cylinder filters: </World/Stem/stem_Link_03/Cylinder>
```

## Structure in USD

### RigidBody Level (Xform)
```
def Xform "stem_Link_02" (
    prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI", "PhysicsFilteredPairsAPI"]
)
{
    prepend rel physics:filteredPairs = </World/Stem/stem_Link_01>
    ...
}
```

### Collision Shape Level (Cylinder)
```
def Cylinder "Cylinder" (
    prepend apiSchemas = ["PhysicsCollisionAPI", "PhysicsFilteredPairsAPI"]
)
{
    prepend rel physics:filteredPairs = </World/Stem/stem_Link_01/Cylinder>
    ...
}
```

## Why Both Levels Matter

1. **RigidBody filtering (Xform)**: Should theoretically propagate to child collision shapes
2. **Collision shape filtering (Cylinder)**: Explicit guarantee that shapes don't collide

Some PhysX implementations may not automatically propagate RigidBody-level filtering to shapes, so having both ensures compatibility.

## All Fixes Applied

This test includes ALL fixes discovered during debugging:

1. ✅ **targetPosition = 0** (not 45°)
2. ✅ **Explicit COM** (height/2 along Z)  
3. ✅ **Correct damping ratio** (D * sqrt(5) ≈ 2.236 for attachment joint)
4. ✅ **Dual-level collision filtering** (RigidBody + Cylinder)

## Test Command

```bash
cd ~/isaacsim && ./python.sh /home/alessandro/isaacsim/autotom_digital_twin/src/experiments/recursive_tree/tests/_load_test2_damping_fix.py
```

## Expected Behavior

With all fixes applied:
- ✅ Geometry should start at |/ (stem vertical, petiole 45°)
- ✅ NO snap to Y shape when pressing PLAY
- ✅ NO collisions between adjacent links (filtered)
- ✅ Proper damping (no oscillations)
- ✅ Stable simulation with natural droop from gravity

## Regenerate Test

```bash
uv run src/experiments/recursive_tree/tests/test_damping_fix.py
```
