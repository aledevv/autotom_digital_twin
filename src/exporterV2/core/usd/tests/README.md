# USD Core Tests

Unit tests and demos for core USD geometry and physics primitives.

## Sphere Geometry Tests

### Task 1: Sphere Support Implementation

Tests the `create_sphere_rigid_body()` function for creating spherical rigid bodies.

**Files:**
- `demo_sphere.py` - Generates demo USD with multiple spheres
- `load_sphere_demo.py` - Loads demo in Isaac Sim for visual validation
- `test_sphere_geometry.py` - Unit tests (requires pxr environment)

### Running Tests

#### Generate Demo USD
```bash
cd /home/alessandro/isaacsim/autotom_digital_twin
python -m exporterV2.core.usd.tests.demo_sphere
```

This creates `data/usd_models/sphere_demo.usda` with:
- 4 spheres at different positions and sizes
- Ground plane for reference
- All spheres with collision and rigid body physics

#### Load in Isaac Sim
```bash
# From Isaac Sim Python environment
cd /home/alessandro/isaacsim/autotom_digital_twin/src/exporterV2/core/usd/tests
python load_sphere_demo.py
```

Or open manually:
1. Open Isaac Sim
2. File → Open → `data/usd_models/sphere_demo.usda`
3. Press PLAY to run simulation

#### Expected Behavior
- 4 spheres should be visible at different heights
- Pressing PLAY starts physics simulation
- Spheres fall with gravity and collide with ground
- Physics Inspector shows RigidBodyAPI and MassAPI on each sphere

### Validation Checklist

- [x] `create_sphere_rigid_body()` function implemented in `geometry.py`
- [x] Function creates UsdGeom.Sphere with correct radius
- [x] RigidBodyAPI applied to sphere xform
- [x] MassAPI applied with correct mass value
- [x] CollisionAPI applied to sphere geometry
- [x] Center of mass at sphere center (0,0,0)
- [x] Position set correctly via TranslateOp
- [x] Optional orientation supported via OrientOp
- [x] Demo USD generated successfully
- [ ] Visual validation in Isaac Sim (manual test)
- [ ] Physics simulation runs without errors (manual test)
- [ ] Spheres collide correctly (manual test)

### Next Steps

Task 2: Implement fixed joint attachment for connecting spheres to pedicels.

---

## Fixed Joint Attachment Tests

### Task 2: Fixed Joint for Leaf Nodes

Tests the fixed joint attachment functions for rigidly attaching bodies (like tomatoes) to link tips.

**Files:**
- `demo_fixed_joint.py` - Generates demo USD with pedicels and spheres attached via FixedJoint
- `load_fixed_joint_demo.py` - Loads demo in Isaac Sim for visual validation

### Running Tests

#### Generate Demo USD
```bash
cd /home/alessandro/isaacsim/autotom_digital_twin
python -m exporterV2.core.usd.tests.demo_fixed_joint
```

This creates `data/usd_models/fixed_joint_demo.usda` with:
- Multiple pedicels (cylinders) anchored to world
- Spheres of varying sizes attached to pedicel tips
- One tilted pedicel to test orientation handling
- Fixed joints connecting spheres to pedicels

#### Load in Isaac Sim
```bash
# From Isaac Sim Python environment
cd /home/alessandro/isaacsim/autotom_digital_twin/src/exporterV2/core/usd/tests
python load_fixed_joint_demo.py
```

Or open manually:
1. Open Isaac Sim
2. File → Open → `data/usd_models/fixed_joint_demo.usda`
3. Press PLAY to run simulation

#### Expected Behavior
- Multiple pedicels with spheres at tips should be visible
- Pressing PLAY starts physics simulation
- Spheres remain rigidly attached to pedicels (no relative motion)
- Under gravity, entire structure moves together if not anchored
- Physics Inspector shows FixedJoint for each sphere attachment

### Validation Checklist

- [x] `create_fixed_joint_to_tip()` function implemented in `joints.py`
- [x] Function creates FixedJoint with correct body0/body1 targets
- [x] LocalPos0 at parent tip (0, 0, parent_height)
- [x] LocalPos1 with child_offset support for positioning
- [x] Collision filtering between parent and child
- [x] `create_fixed_joint_attachment()` for general attachment cases
- [x] Demo USD generated with multiple test cases
- [ ] Visual validation in Isaac Sim (manual test)
- [ ] Spheres remain rigidly attached during simulation (manual test)
- [ ] No separation under gravity or external forces (manual test)

### Implementation Notes

Two functions provided:
1. **`create_fixed_joint_to_tip()`**: Simplified interface for attaching to cylinder tips
   - Automatically positions child at tip with offset
   - Common case: tomatoes at pedicel tips
2. **`create_fixed_joint_attachment()`**: General interface for arbitrary attachment
   - Custom position and orientation
   - Used for complex scenarios

Both functions:
- Create FixedJoint (completely rigid, no drives)
- Filter collisions between parent and child
- Exclude child from articulation chain (leaf node behavior)

### Next Steps

Task 3: Create truss_builder.py to generate rachis branch definitions.
