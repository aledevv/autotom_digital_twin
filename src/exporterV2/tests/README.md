# ExporterV2 Test Suite

Automated tests for exporterV2 functionality and collision detection.

## Test Files

### `test_refactoring.sh`
Tests import structure, profiles, and JSON generation after Phase 1-2 refactoring.

**Run from project root:**
```bash
cd /home/alessandro/isaacsim/autotom_digital_twin
./src/exporterV2/tests/test_refactoring.sh
```

**Tests:**
- ✓ Import structure (core, adapters, profiles)
- ✓ JSON generation (day 1: 20 branches, 23 links)
- ✓ Profile system (tomato, simple plant)
- ✓ Directory structure

---

### `test_collision_geometry.py`
Geometric collision verification for lateral branches, leaves, and trunk.

**Run from project root:**
```bash
cd /home/alessandro/isaacsim/autotom_digital_twin
python3 src/exporterV2/tests/test_collision_geometry.py output/day_100/branches_v2_day_100.json
```

**Tests:**
- ✓ Lateral branch inter-collision (same rank, adjacent ranks)
- ✓ Rotation variance (jitter working)
- ✓ Minimum angle separation (≥60°)
- ✓ Bounding box overlap

**Example output:**
```
✅ PASS: Lateral Branch Collision
✅ PASS: Rotation Variance
✅ PASS: Angle Separation
✅ PASS: Bounding Box Overlap
```

---

## Running All Tests

```bash
cd /home/alessandro/isaacsim/autotom_digital_twin

# Test 1: Refactoring verification
./src/exporterV2/tests/test_refactoring.sh

# Test 2: Generate day 100 (requires Isaac Sim)
./run_mainV2.sh --day 100

# Test 3: Collision check
python3 src/exporterV2/tests/test_collision_geometry.py output/day_100/branches_v2_day_100.json
```

---

## Test Coverage

| Test | Coverage | Status |
|------|----------|--------|
| Import structure | Core, adapters, profiles | ✅ |
| JSON generation | Day 1 validation | ✅ |
| Profile system | Tomato, simple plant | ✅ |
| Lateral branch collision | Geometric checks | ✅ |
| Rotation jitter | Variance detection | ✅ |
| Angle separation | Min 60° enforcement | ✅ |

---

## Adding New Tests

1. Create test file in `src/exporterV2/tests/`
2. Make executable: `chmod +x test_name.sh`
3. Document in this README
4. Add to CI pipeline (if applicable)
