# Task 1: Infrastructure Tests

Tests for the base optimization infrastructure (Task 1).

## Files

### Unit Tests
- **test_optimizer_simple.py**: Standalone unit tests (6 tests)
  - Config loading
  - Joint calculation  
  - Lower bound calculation
  - Optimization skeleton

- **test_optimizer.py**: Pytest-based tests
  - Same coverage as above but pytest compatible

### Demo Scripts
- **demo_task1.py**: Full demonstration of Task 1
  - Shows config loading, joint calc, lower bound, report

## Running Tests

```bash
# Standalone tests
uv run python src/exporterV2/core/optimizations/tests/1_infrastructure/test_optimizer_simple.py

# Pytest
uv run pytest src/exporterV2/core/optimizations/tests/1_infrastructure/test_optimizer.py

# Demo
uv run python src/exporterV2/core/optimizations/tests/1_infrastructure/demo_task1.py
```

## What Task 1 Tests

- ✅ YAML configuration loading
- ✅ Joint count calculation from branches config
- ✅ Lower bound (structural minimum) calculation
- ✅ Report formatting
- ✅ Budget validation logic

**Note**: Task 1 only sets up infrastructure. No optimization techniques are implemented yet (those come in Tasks 4-8).
