# Task 3: Geometry Remapping Tests

Tests for geometry remapping utilities (Task 3).

**Status**: 🔴 TODO - Not yet implemented

## Planned Files

### Unit Tests
- **test_geometry_remapping.py**: Tests for attachment remapping
  - Remap attachment height when collapsing segments
  - Preserve absolute height
  - Handle edge cases (first/last link)
  - Non-uniform segment heights

### Utilities
- **test_bounds.py**: Tests for bounding volume calculation
  - Convert branch config to CylinderGeometry
  - Calculate bounds from link geometry

## What Task 3 Will Test

### Geometry Remapping
When collapsing stem segments (e.g., 5 → 3 → 1 links):
- **Preserve absolute height**: Branch attached at height H should remain at height H
- **Remap to new link index**: Calculate which new link and offset
- **Handle non-uniform segments**: Different link heights

### Example
```
Original: 5 links (heights: 0.2m each)
Branch attached at: link 3, offset 0.1m → absolute height = 0.7m

After collapse to 3 links (heights: 0.33m each):
New attachment: link 2, offset 0.04m → absolute height = 0.7m ✓
```

## Running Tests (Once Implemented)

```bash
# Unit tests
uv run python src/exporterV2/core/optimizations/tests/3_geometry/test_geometry_remapping.py

# Pytest
uv run pytest src/exporterV2/core/optimizations/tests/3_geometry/
```

## Usage in Optimization

Geometry remapping is critical for **Task 6 (Stem Collapse)**:
1. Collapse main stem segments (reduce n_links)
2. Remap all child branch attachment points
3. Validate with collision detection (Task 2)
4. Ensure no geometric artifacts or invalid attachments
