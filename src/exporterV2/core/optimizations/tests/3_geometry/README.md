# Task 3: Geometry Remapping Tests

Tests for geometry remapping utilities (Task 3).

**Status**: ✅ DONE - Task 3 completed

## Files

### Core Modules
- **`geometry/remapping.py`**: Attachment remapping logic
  - `calculate_absolute_height()`: Calculate absolute height from link index + offset
  - `find_new_attachment()`: Find new attachment point for target height
  - `remap_attachment_height()`: Main remapping function
  - `remap_all_children()`: Batch remap all child branches
  
- **`geometry/bounds.py`**: Bounding volume utilities
  - `link_to_cylinder_geometry()`: Convert branch link to CylinderGeometry
  - `branch_to_cylinder_geometries()`: Convert all links in branch
  - `calculate_attachment_position()`: Calculate absolute position of attachment

### Unit Tests
- **`test_geometry_remapping.py`**: Automated tests (8 tests, all passing ✅)
  - Absolute height calculation
  - New attachment finding
  - Simple remapping (5→3 links)
  - Extreme collapse (5→1 link)
  - Edge cases (top, bottom, single link)
  - Non-uniform segments
  - Multiple child branches
  - Invalid inputs handling

### Demo Scripts
- **`demo_task3.py`**: Interactive demonstration
  - Simple stem collapse demo
  - Extreme collapse visualization
  - Multiple branches remapping
  - Comparison table across scenarios

## What Task 3 Implements

### Geometry Remapping
When collapsing stem segments (e.g., 5 → 3 → 1 links):
- **Preserve absolute height**: Branch attached at height H remains at height H
- **Remap to new link index**: Calculate which new link and offset
- **Handle non-uniform segments**: Different link heights supported

### Example
```python
# Original: 5 links (heights: 0.2m each)
# Branch attached at: link 3, offset 0.1m → absolute height = 0.7m

from geometry import remap_attachment_height

result = remap_attachment_height(
    original_link_idx=3,
    original_offset=0.1,
    original_segment_heights=[0.2] * 5,
    new_segment_heights=[0.33, 0.33, 0.34]  # 3 links
)

# After collapse to 3 links (heights: ~0.33m each):
# New attachment: link 2, offset 0.04m → absolute height = 0.7m ✓
print(f"New: link {result.new_link_idx}, offset {result.new_offset:.3f}m")
print(f"Height error: {result.height_error:.4f}m")  # ~0.0000m
```

## Running Tests

### Automated Tests
```bash
# Run all unit tests (8 tests)
uv run python src/exporterV2/core/optimizations/tests/3_geometry/test_geometry_remapping.py
```

### Interactive Demo
```bash
# Run demo with 4 scenarios
uv run python src/exporterV2/core/optimizations/tests/3_geometry/demo_task3.py
```

## Test Coverage

✅ **Absolute height calculation** - Correct for any link/offset  
✅ **New attachment finding** - Preserves target height  
✅ **Simple remapping** - 5→3 links with sub-mm accuracy  
✅ **Extreme collapse** - 5→1 link preserves all attachments  
✅ **Edge cases** - Top, bottom, single link  
✅ **Non-uniform segments** - Variable link heights  
✅ **Multiple children** - Batch remapping  
✅ **Error handling** - Invalid inputs rejected gracefully  

## Usage in Optimization

Geometry remapping is critical for **Task 6 (Stem Collapse)**:
1. Collapse main stem segments (reduce n_links)
2. Remap all child branch attachment points
3. Validate with collision detection (Task 2)
4. Ensure no geometric artifacts or invalid attachments

The remapping ensures that lateral branches, petioles, and trusses remain at their original heights even after the stem topology changes.
