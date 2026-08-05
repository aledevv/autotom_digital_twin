# Optimization Tests

Test suite for the joint-budget optimization system.

## Test Organization

Tests are organized by task/feature in numbered folders:

```
tests/
├── 1_infrastructure/       ✅ Task 1 - Base infrastructure tests
├── 2_collision/            ✅ Task 2 - Collision detection tests
├── 3_geometry/             🔴 Task 3 - Geometry remapping (TODO)
├── 4_petiole_lock/         🔴 Task 4 - Petiole lock technique (TODO)
├── 5_lateral_reduce/       🔴 Task 5 - Lateral reduction (TODO)
├── 6_stem_collapse/        🔴 Task 6 - Stem collapse (TODO)
├── 7_truss_static/         🔴 Task 7 - Truss static (TODO)
├── 8_leaf_reduce/          🔴 Task 8 - Leaf reduction (TODO)
├── 9_integration/          🔴 Task 9 - Integration tests (TODO)
└── visual_validation/      🔴 Task 10 - Visual suite (TODO)
```

## Quick Reference

### Task 1: Infrastructure (✅ DONE)
```bash
# Unit tests (6 tests)
uv run python tests/1_infrastructure/test_optimizer_simple.py

# Demo
uv run python tests/1_infrastructure/demo_task1.py
```

**Tests**: Config loading, joint calculation, lower bound, report formatting

---

### Task 2: Collision Detection (✅ DONE)
```bash
# Unit tests (12 tests)
uv run python tests/2_collision/test_collision_detection.py

# Visual: 4 clear scenarios (RECOMMENDED)
uv run python tests/2_collision/visual_collision_4_scenarios.py

# Visual: Random N-body testing
uv run python tests/2_collision/visual_collision_random_test.py

# Demo
uv run python tests/2_collision/demo_task2.py
```

**Tests**: Two-stage collision (sphere + AABB), visual validation with 3D plots

---

### Task 3: Geometry Remapping (✅ DONE)
```bash
# Unit tests (8 tests)
uv run python tests/3_geometry/test_geometry_remapping.py

# Demo
uv run python tests/3_geometry/demo_task3.py
```

**Tests**: Attachment remapping, height preservation, batch remapping

---

### Tasks 4-8: Optimization Techniques (🔴 TODO)
Individual tests for each technique:
- Petiole lock (D6 → Fixed)
- Lateral branch reduction
- Stem collapse + remapping
- Truss static pre-bent
- Leaf branch reduction

---

### Task 9: Integration Tests (🔴 TODO)
Multi-technique composition, budget scenarios, regression tests.

---

### Task 10: Visual Validation (🔴 TODO)
IsaacSim visual tests with manual checklist.

---

## Test Types

### 1. Unit Tests
- Fast, automated tests
- Run with `python` or `pytest`
- Test individual functions/classes
- Example: `test_optimizer_simple.py`, `test_collision_detection.py`

### 2. Demo Scripts
- End-to-end demonstrations
- Show complete workflow
- Example: `demo_task1.py`, `demo_task2.py`

### 3. Visual Tests
- Interactive 3D matplotlib plots
- Manual verification by user
- Example: `visual_collision_4_scenarios.py`, `visual_collision_random_test.py`

### 4. Integration Tests (TODO)
- Multi-component tests
- Real-world scenarios
- Performance benchmarks

### 5. Visual Validation (TODO)
- IsaacSim rendering
- Manual checklist verification
- Before/after comparisons

---

## Running All Tests

```bash
# Run all automated tests (once more are implemented)
cd /home/alessandro/isaacsim/autotom_digital_twin
uv run pytest src/exporterV2/core/optimizations/tests/

# Run specific task tests
uv run pytest src/exporterV2/core/optimizations/tests/1_infrastructure/
uv run pytest src/exporterV2/core/optimizations/tests/2_collision/
```

---

## Test Coverage Status

| Task | Component | Unit Tests | Visual Tests | Status |
|------|-----------|------------|--------------|--------|
| 1 | Infrastructure | ✅ 6 tests | ✅ Demo | ✅ DONE |
| 2 | Collision | ✅ 12 tests | ✅ 4 visual | ✅ DONE |
| 3 | Geometry | ✅ 8 tests | ✅ Demo | ✅ DONE |
| 4 | Petiole Lock | 🔴 TODO | 🔴 TODO | 🔴 TODO |
| 5 | Lateral Reduce | 🔴 TODO | 🔴 TODO | 🔴 TODO |
| 6 | Stem Collapse | 🔴 TODO | 🔴 TODO | 🔴 TODO |
| 7 | Truss Static | 🔴 TODO | 🔴 TODO | 🔴 TODO |
| 8 | Leaf Reduce | 🔴 TODO | 🔴 TODO | 🔴 TODO |
| 9 | Integration | 🔴 TODO | 🔴 TODO | 🔴 TODO |
| 10 | Visual Suite | N/A | 🔴 TODO | 🔴 TODO |

**Progress**: 3/12 tasks complete (25%)

---

## Adding New Tests

When implementing a new task:

1. Create folder: `tests/N_task_name/`
2. Add README.md explaining what's tested
3. Create unit tests: `test_*.py`
4. Create demo: `demo_taskN.py`
5. (Optional) Add visual tests for manual verification
6. Update this README with status

---

## Notes

- All test paths assume running from workspace root
- Use `uv run` for consistent environment
- Visual tests require matplotlib and user interaction
- Integration tests may require IsaacSim (not yet implemented)
