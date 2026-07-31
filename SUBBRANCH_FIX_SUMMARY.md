# Sub-Branch Static Position Bug Fix

## Summary

Fixed the incorrect initial world-space position of sub-branches (branches attaching to already-tilted parent branches) in the recursive tree articulation system. The smallest sub-branch (subA1, radius=5cm) was appearing in the wrong position when simulation was OFF (static render), but would snap to the correct position when simulation started.

## Root Cause

In `generate_recursive_tree_usda.py` line ~372, the attachment position calculation was:

```python
rotated_offset = rot_z.TransformDir(base_offset_local)
start_pos = attach_base + rotated_offset
```

This computed the radial offset by rotating around **world Z only** (azimuth). This is correct for trunk children (branchA), because the trunk is vertical with identity orientation. 

However, for sub-branches attaching to **already-tilted parents** (e.g., subA1 attaching to branchA which is tilted 45°), we need to transform the offset by the **parent's full orientation**, not just the azimuth rotation.

## The Fix

### 1. Track Parent Orientations

Added `orient_registry` to store each branch's world-space orientation quaternion:

```python
orient_registry = {}  # branch_id -> Gf.Quatf (world-space orientation)
```

For the trunk (root):
```python
orient_registry[bid] = Gf.Quatf(1, 0, 0, 0)  # identity
```

For branches:
```python
orient_registry[bid] = chain_orientation  # computed world quaternion
```

### 2. Compute Branch Orientation Relative to Parent

```python
# Get parent's world-space orientation
parent_orientation = orient_registry.get(parent_id, Gf.Quatf(1, 0, 0, 0))

# Compute branch rotation in parent's local frame
rot_z    = Gf.Rotation(Gf.Vec3d(0, 0, 1), rot_deg)      # azimuth
rot_tilt = Gf.Rotation(Gf.Vec3d(1, 0, 0), -tilt_deg)    # tilt
branch_rot_in_parent_frame = rot_z * rot_tilt

# Combine with parent's world orientation
parent_rot = Gf.Rotation(Gf.Quatd(parent_orientation))
combined = parent_rot * branch_rot_in_parent_frame

# Branch's world orientation
chain_orientation = Gf.Quatf(combined.GetQuat())
```

### 3. Transform Offset to World Frame

```python
# Compute offset in parent's local frame
radial_distance = p_r_world / 2.0
base_offset_local = Gf.Vec3d(0.0, radial_distance, p_h_world + gap)

# Apply azimuth rotation in parent frame
rot_z_local = Gf.Rotation(Gf.Vec3d(0, 0, 1), rot_deg)
offset_in_parent_frame = rot_z_local.TransformDir(base_offset_local)

# Transform from parent local frame to world frame
parent_rot_matrix = Gf.Matrix3d(parent_orientation)
offset_in_world = parent_rot_matrix * offset_in_parent_frame

# Compute world position
start_pos = attach_base + offset_in_world
```

### 4. Update Joint Frame to be Relative to Parent

LocalPos0 now uses the offset in parent frame:
```python
local_pos0 = Gf.Vec3f(
    offset_in_parent_frame[0],
    offset_in_parent_frame[1],
    offset_in_parent_frame[2]
)
```

LocalRot0 expresses branch orientation relative to parent:
```python
parent_rot_inv = Gf.Rotation(Gf.Quatd(parent_orientation)).GetInverse()
local_rot_gfd  = parent_rot_inv * combined
local_rot0     = Gf.Quatf(local_rot_gfd.GetQuat())
```

## Verification

### Geometric Consistency

For subA1 attaching to branchA_Link_02:

- **Parent position**: (0.0, 1.568, 7.098)
- **Parent orientation**: 45° tilt around X-axis
- **Attachment offset** (parent frame): (-0.15, 0, 1.51) 
  - Radial: 0.15m (half of parent radius 0.3m)
  - Axial: 1.51m (parent height 1.5m + gap 0.01m)
  - Azimuth: 90° → rotates (0, 0.15, 1.51) to (-0.15, 0, 1.51)

- **Transformed to world**: (-0.15, -1.068, 1.068)
  - After applying parent's 45° tilt rotation

- **Child world position**: (-0.15, 0.500, 8.165)
  - Parent pos + world offset = expected child pos ✓

**Position error**: < 0.001mm (within numerical precision)

### Test Results

`test_subbranch_position.py` verifies:
1. World-space position matches geometric calculation
2. Joint LocalPos0 is correctly in parent's local frame
3. Offset magnitude matches expected value (1.517m)

```
Expected offset magnitude: 1.517432m
Actual offset magnitude:   1.517432m
Test result: ✓ PASS
```

### USD Verification

Generated `recursive_tree.usda` shows:

**subA1_Link_01**:
```
quatf xformOp:orient = (0.52133375, -0.47771445, -0.030843617, 0.7064338)
double3 xformOp:translate = (-0.15, 0.5000000500789505, 8.165462493568075)
```

**AttachJoint** (subA1 → branchA):
```
point3f physics:localPos0 = (-0.15, 3.3306692e-17, 1.51)  # in parent frame
quatf physics:localRot0 = (0.66446304, -0.24184476, 0.24184476, 0.66446304)
```

## Impact

- **Before fix**: Sub-branches appeared misaligned in static render, snapping to correct position when simulation started
- **After fix**: Sub-branches appear in correct position immediately in static render, remain stable during simulation
- **No change**: Joint constraints were already correct (which is why simulation worked)

## Files Changed

1. `src/experiments/recursive_tree/generate_recursive_tree_usda.py`
   - Added `orient_registry` tracking
   - Fixed offset transformation to use parent's full orientation
   - Updated joint frame computation to be relative to parent

2. `test_subbranch_position.py` (new)
   - Automated test verifying position consistency

3. `data/usd_models/recursive_tree.usda` (regenerated)
   - Updated with correct positions

## References

Working implementation in `generate_generalized_articulation_usda.py` (line ~298):
```python
rot_total = Gf.Rotation(Gf.Vec3d(1.0, 0.0, 0.0), -tilt_angle_deg) * rot_z
branch_world_base_pos = parent_pos + rot_total.TransformDir(Gf.Vec3d(0.0, 0.0, z_distance))
```

This correctly applies both tilt and azimuth to transform positions from parent's local frame to world frame.
