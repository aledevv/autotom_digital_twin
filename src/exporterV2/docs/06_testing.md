# Testing

Automated test suite for ExporterV2.

---

## Test Files

Located in `src/exporterV2/tests/`:

1. **test_refactoring.sh** - Import structure & JSON generation
2. **test_collision_geometry.py** - Geometric collision verification

---

## 1. Refactoring Tests

### Purpose
Verify Phase 1-2 refactoring didn't break functionality.

### Run
```bash
cd /home/alessandro/isaacsim/autotom_digital_twin
./src/exporterV2/tests/test_refactoring.sh
```

### Tests

#### Import Structure
```bash
✓ Core modules (tree_config, physics, usd)
✓ Adapters (groimp_csv)
✓ Profiles (tomato_default, simple_plant)
```

#### JSON Generation
```bash
✓ Day 1 output exists
✓ 20 branches in JSON
✓ 23 total links (trunk + branches)
```

### Expected Output
```
✅ All tests passed!
```

---

## 2. Collision Geometry Tests

### Purpose
Verify lateral branches don't collide (prevent simulation explosion).

### Prerequisites
```bash
# Generate day 100 model first
./run_mainV2.sh --day 100
```

### Run
```bash
python3 src/exporterV2/tests/test_collision_geometry.py output/day_100/branches_v2_day_100.json
```

### Tests

#### Lateral Branch Collision
Checks angular separation between branches:
- Same rank
- Adjacent ranks (rank±1)

```python
✓ Parent rank 0: 25.0° / 206.7° (sep: 178.3°)
✓ Parent rank 1: 90.0° / 271.7° (sep: 178.3°)
✓ Parent rank 2: 12.5° / 161.2° (sep: 148.7°)
```

#### Rotation Variance
Verifies jitter is working (not all angles identical).

```python
✓ 8 unique rotation angles (out of 8 branches)
```

#### Angle Separation
Enforces **60° minimum** between all branches.

```python
✓ All separations ≥ 60.0°
```

#### Bounding Box Overlap
Geometric check for cylinder intersections.

```python
✓ No overlapping bounding boxes
```

### Expected Output
```
✅ PASS: Lateral Branch Collision
✅ PASS: Rotation Variance
✅ PASS: Angle Separation
✅ PASS: Bounding Box Overlap
```

---

## Complete Test Suite

Run all tests in sequence:

```bash
cd /home/alessandro/isaacsim/autotom_digital_twin

# 1. Refactoring verification
./src/exporterV2/tests/test_refactoring.sh

# 2. Generate day 100 (requires Isaac Sim)
./run_mainV2.sh --day 100

# 3. Collision check
python3 src/exporterV2/tests/test_collision_geometry.py output/day_100/branches_v2_day_100.json
```

---

## Test Coverage

| Component | Test | Status |
|-----------|------|--------|
| Core imports | Import structure | ✅ |
| Adapter imports | Import structure | ✅ |
| Profile imports | Import structure | ✅ |
| JSON generation | Day 1 validation | ✅ |
| Lateral branches | Collision check | ✅ |
| Random jitter | Variance check | ✅ |
| Angle separation | Min 60° check | ✅ |
| Bounding boxes | Overlap check | ✅ |

---

## Troubleshooting

### Test Fails: "Command not found"
Make test executable:
```bash
chmod +x src/exporterV2/tests/test_refactoring.sh
```

### Test Fails: "Module not found"
Run from project root:
```bash
cd /home/alessandro/isaacsim/autotom_digital_twin
./src/exporterV2/tests/test_refactoring.sh  # ✅
```

### Test Fails: "JSON file not found"
Generate day 100 first:
```bash
./run_mainV2.sh --day 100
```

---

## Adding New Tests

1. Create test file in `tests/`
2. Add execution instructions to `tests/README.md`
3. Document expected output
4. Add to this file

**Example:**
```bash
# tests/test_new_feature.sh
#!/bin/bash
python3 -c "from exporterV2.profiles import NEW_PROFILE; assert NEW_PROFILE"
echo "✅ New feature test passed"
```

---

**See also:**
- [tests/README.md](../tests/README.md) - Quick reference
- [04_collision_checks.md](04_collision_checks.md) - What tests validate
- [07_troubleshooting.md](07_troubleshooting.md) - If tests fail
