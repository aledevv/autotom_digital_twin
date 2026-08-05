# Task 2: Collision Detection Tests

Tests for the two-stage collision detection system (Task 2).

## Files

### Unit Tests
- **test_collision_detection.py**: Automated tests (12 tests)
  - Vec3 operations
  - Bounding sphere calculation and overlap
  - AABB calculation and overlap
  - Two-stage broad-phase detection
  - Pairwise collision checking

### Visual Tests (Interactive 3D)
- **visual_collision_4_scenarios.py**: 4 representative scenarios
  - Scenario 1: S=False, AB=False → NO COLLISION
  - Scenario 2: S=True, AB=True → COLLISION DETECTED
  - Scenario 3: S=True, AB=False → NO COLLISION (false positive filtered)
  - Scenario 4: S=True, AB=True → COLLISION (edge case)

- **visual_collision_random_test.py**: Random multi-body testing
  - Generate N random cylinders
  - Detect all collisions
  - Color-coded: Blue=safe, Red=colliding
  - Shows AABBs to understand detection

- **visual_collision_3d_interactive.py**: Original 3D interactive test
  - 3 scenarios with interactive rotation/zoom

- **visual_collision_test.py**: Original 2D static plots (deprecated)

### Demo Scripts
- **demo_task2.py**: Full demonstration of collision system
  - 4 real-world scenarios
  - Shows sphere + AABB checks
  - Reports collision results

### Generated Images
- `collision_test1_no_collision.png`
- `collision_test2_collision.png`
- `collision_test3_false_positive.png`

## Running Tests

### Automated Tests
```bash
# Run all unit tests
uv run python src/exporterV2/core/optimizations/tests/2_collision/test_collision_detection.py
```

### Visual Validation

**4 Scenarios (Recommended)**:
```bash
uv run python src/exporterV2/core/optimizations/tests/2_collision/visual_collision_4_scenarios.py
```
Shows the 4 key cases with clear output labels.

**Random Testing**:
```bash
uv run python src/exporterV2/core/optimizations/tests/2_collision/visual_collision_random_test.py
```
Enter number of bodies (e.g., 10), see collisions detected. Great for stress testing!

**Interactive 3D**:
```bash
uv run python src/exporterV2/core/optimizations/tests/2_collision/visual_collision_3d_interactive.py
```
Original interactive version (3 scenarios).

**Demo**:
```bash
uv run python src/exporterV2/core/optimizations/tests/2_collision/demo_task2.py
```

## What Task 2 Tests

### Two-Stage Collision Detection
1. **Stage 1 (Sphere)**: Fast pre-check using bounding spheres
   - O(1) distance calculation
   - Conservative (may have false positives)
   
2. **Stage 2 (AABB)**: Precision check using axis-aligned bounding boxes
   - Only runs if sphere overlap detected
   - Filters false positives from Stage 1

### Key Features
- ✅ Sphere overlap detection with safety margin
- ✅ AABB overlap detection for oriented cylinders
- ✅ Two-stage orchestration (broad-phase)
- ✅ Pairwise collision checking for multiple bodies
- ✅ Collision statistics and reporting

### Visual Validation
The visual tests help verify:
- Bounding volumes correctly enclose cylinders
- Sphere check is conservative (catches potential overlaps)
- AABB check filters false positives
- Final collision detection matches visual expectation

## Usage in Optimization

Collision detection is used in **Task 6 (Stem Collapse)**:
- When remapping attachment points after collapsing stem segments
- Validates that remapped branches don't collide with siblings or parent
- Ensures geometric validity of optimization
