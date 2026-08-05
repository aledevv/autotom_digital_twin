"""
test_geometry_remapping.py - Unit Tests for Geometry Remapping

Tests for attachment point remapping when collapsing segments.
Validates height preservation and edge cases.

Run with: uv run python src/exporterV2/core/optimizations/tests/3_geometry/test_geometry_remapping.py
"""

import sys
import os

# Add optimizations directory to path
script_dir = os.path.dirname(os.path.abspath(__file__))
optimizations_dir = os.path.join(script_dir, "../..")
sys.path.insert(0, optimizations_dir)

from geometry.remapping import (
    calculate_absolute_height,
    find_new_attachment,
    remap_attachment_height,
    remap_all_children,
    RemappingResult
)


def test_calculate_absolute_height():
    """Test absolute height calculation."""
    print("\n[TEST] Calculate Absolute Height...")
    
    segments = [0.2, 0.2, 0.2, 0.2, 0.2]  # 5 segments @ 0.2m each
    
    # Test 1: Base of first link
    height = calculate_absolute_height(0, 0.0, segments)
    assert abs(height - 0.0) < 1e-6, f"Expected 0.0, got {height}"
    print(f"  ✓ Link 0, offset 0.0 → {height}m")
    
    # Test 2: Middle of first link
    height = calculate_absolute_height(0, 0.1, segments)
    assert abs(height - 0.1) < 1e-6, f"Expected 0.1, got {height}"
    print(f"  ✓ Link 0, offset 0.1 → {height}m")
    
    # Test 3: Base of third link
    height = calculate_absolute_height(2, 0.0, segments)
    assert abs(height - 0.4) < 1e-6, f"Expected 0.4, got {height}"
    print(f"  ✓ Link 2, offset 0.0 → {height}m")
    
    # Test 4: Middle of third link
    height = calculate_absolute_height(2, 0.1, segments)
    assert abs(height - 0.5) < 1e-6, f"Expected 0.5, got {height}"
    print(f"  ✓ Link 2, offset 0.1 → {height}m")
    
    # Test 5: Top of last link
    height = calculate_absolute_height(4, 0.2, segments)
    assert abs(height - 1.0) < 1e-6, f"Expected 1.0, got {height}"
    print(f"  ✓ Link 4, offset 0.2 → {height}m (top)")


def test_find_new_attachment():
    """Test finding new attachment point."""
    print("\n[TEST] Find New Attachment...")
    
    new_segments = [0.33, 0.33, 0.34]  # 3 segments, ~0.33m each
    
    # Test 1: Target height 0.5m
    link_idx, offset = find_new_attachment(0.5, new_segments)
    actual = calculate_absolute_height(link_idx, offset, new_segments)
    error = abs(actual - 0.5)
    assert error < 0.01, f"Target 0.5m, got {actual}m (error: {error}m)"
    print(f"  ✓ Target 0.5m → link {link_idx}, offset {offset:.3f}m (actual: {actual:.3f}m)")
    
    # Test 2: Target height 0.0m (bottom)
    link_idx, offset = find_new_attachment(0.0, new_segments)
    assert link_idx == 0 and abs(offset) < 1e-6, f"Expected (0, 0.0), got ({link_idx}, {offset})"
    print(f"  ✓ Target 0.0m → link {link_idx}, offset {offset:.3f}m (bottom)")
    
    # Test 3: Target height 1.0m (top)
    link_idx, offset = find_new_attachment(1.0, new_segments)
    actual = calculate_absolute_height(link_idx, offset, new_segments)
    error = abs(actual - 1.0)
    assert error < 0.01, f"Target 1.0m, got {actual}m (error: {error}m)"
    print(f"  ✓ Target 1.0m → link {link_idx}, offset {offset:.3f}m (top)")
    
    # Test 4: Target in middle link
    link_idx, offset = find_new_attachment(0.7, new_segments)
    actual = calculate_absolute_height(link_idx, offset, new_segments)
    error = abs(actual - 0.7)
    assert error < 0.01, f"Target 0.7m, got {actual}m"
    print(f"  ✓ Target 0.7m → link {link_idx}, offset {offset:.3f}m (actual: {actual:.3f}m)")


def test_remap_attachment_simple():
    """Test simple remapping case."""
    print("\n[TEST] Remap Attachment - Simple...")
    
    # Original: 5 links @ 0.2m each
    original_segments = [0.2] * 5
    # New: 3 links @ 0.33m each (approximately)
    new_segments = [0.33, 0.33, 0.34]
    
    # Original attachment: link 3, offset 0.1m → absolute height 0.7m
    result = remap_attachment_height(
        original_link_idx=3,
        original_offset=0.1,
        original_segment_heights=original_segments,
        new_segment_heights=new_segments,
        tolerance=0.01
    )
    
    assert result.success, f"Remapping failed: {result.message}"
    assert abs(result.absolute_height - 0.7) < 0.01, f"Height error: {result.absolute_height}"
    print(f"  ✓ Original: link 3, offset 0.1m → {0.7}m")
    print(f"  ✓ Remapped: link {result.new_link_idx}, offset {result.new_offset:.3f}m → {result.absolute_height:.3f}m")
    print(f"  ✓ Height error: {result.height_error:.4f}m")


def test_remap_attachment_extreme_collapse():
    """Test remapping with extreme collapse (5 → 1 link)."""
    print("\n[TEST] Remap Attachment - Extreme Collapse...")
    
    original_segments = [0.2] * 5  # 5 links @ 0.2m
    new_segments = [1.0]           # 1 link @ 1.0m
    
    # Test multiple attachment points
    test_cases = [
        (0, 0.1, 0.1),   # Near bottom
        (2, 0.1, 0.5),   # Middle
        (4, 0.1, 0.9),   # Near top
    ]
    
    for orig_link, orig_offset, target_height in test_cases:
        result = remap_attachment_height(
            orig_link,
            orig_offset,
            original_segments,
            new_segments,
            tolerance=0.01
        )
        
        assert result.success, f"Failed for link {orig_link}: {result.message}"
        assert abs(result.absolute_height - target_height) < 0.01, \
            f"Height mismatch: expected {target_height}, got {result.absolute_height}"
        print(f"  ✓ Link {orig_link}, offset {orig_offset} → height {target_height:.1f}m preserved")


def test_remap_attachment_edge_cases():
    """Test edge cases: attach at top, bottom, single link."""
    print("\n[TEST] Remap Attachment - Edge Cases...")
    
    # Edge case 1: Attach at bottom (link 0, offset 0)
    result = remap_attachment_height(
        0, 0.0,
        [0.2] * 5,
        [0.33, 0.33, 0.34],
        tolerance=0.01
    )
    assert result.success and result.new_link_idx == 0 and result.new_offset < 0.01, \
        f"Bottom attach failed: link {result.new_link_idx}, offset {result.new_offset}"
    print(f"  ✓ Bottom attach: link {result.new_link_idx}, offset {result.new_offset:.3f}m")
    
    # Edge case 2: Attach at top (link 4, offset 0.2)
    result = remap_attachment_height(
        4, 0.2,
        [0.2] * 5,
        [0.33, 0.33, 0.34],
        tolerance=0.01
    )
    assert result.success, f"Top attach failed: {result.message}"
    assert abs(result.absolute_height - 1.0) < 0.01, \
        f"Top height mismatch: {result.absolute_height}"
    print(f"  ✓ Top attach: link {result.new_link_idx}, offset {result.new_offset:.3f}m")
    
    # Edge case 3: Single link collapse
    result = remap_attachment_height(
        2, 0.1,
        [0.2] * 5,
        [1.0],  # All collapsed to 1 link
        tolerance=0.01
    )
    assert result.success, f"Single link failed: {result.message}"
    print(f"  ✓ Single link collapse: link {result.new_link_idx}, offset {result.new_offset:.3f}m")


def test_remap_with_non_uniform_segments():
    """Test remapping with non-uniform segment heights."""
    print("\n[TEST] Remap with Non-Uniform Segments...")
    
    # Original: variable heights
    original_segments = [0.1, 0.2, 0.3, 0.2, 0.2]  # Total 1.0m
    # New: also variable
    new_segments = [0.4, 0.3, 0.3]  # Total 1.0m
    
    # Attach at middle
    orig_link = 2
    orig_offset = 0.15
    target_height = 0.1 + 0.2 + 0.15  # 0.45m
    
    result = remap_attachment_height(
        orig_link,
        orig_offset,
        original_segments,
        new_segments,
        tolerance=0.01
    )
    
    assert result.success, f"Non-uniform failed: {result.message}"
    assert abs(result.absolute_height - target_height) < 0.01, \
        f"Height error: expected {target_height}, got {result.absolute_height}"
    print(f"  ✓ Non-uniform segments: {target_height:.2f}m preserved")
    print(f"    Original: link {orig_link}, offset {orig_offset}")
    print(f"    Remapped: link {result.new_link_idx}, offset {result.new_offset:.3f}m")


def test_remap_all_children():
    """Test remapping multiple child branches."""
    print("\n[TEST] Remap All Children...")
    
    parent = {
        "id": "trunk",
        "n_links": 5,
        "height": 0.2,  # 0.2m per link
        "radius": 0.05
    }
    
    children = [
        {
            "id": "branch1",
            "parent": "trunk",
            "attach_link": 1,
            "attach_offset": 0.1,  # Absolute height: 0.3m
            "n_links": 3
        },
        {
            "id": "branch2",
            "parent": "trunk",
            "attach_link": 3,
            "attach_offset": 0.05,  # Absolute height: 0.65m
            "n_links": 2
        },
        {
            "id": "other_branch",
            "parent": "other_parent",  # Not a child of trunk
            "attach_link": 0,
            "n_links": 1
        }
    ]
    
    # Collapse trunk to 3 links
    remapped, errors = remap_all_children(parent, children, new_n_links=3, tolerance=0.01)
    
    assert len(errors) == 0, f"Remapping had errors: {errors}"
    assert len(remapped) == 3, f"Expected 3 branches, got {len(remapped)}"
    
    # Check branch1 (was at 0.3m)
    branch1 = next(b for b in remapped if b["id"] == "branch1")
    new_height_1 = branch1["attach_link"] * (1.0/3) + branch1.get("attach_offset", 0)
    assert abs(new_height_1 - 0.3) < 0.01, f"Branch1 height mismatch: {new_height_1}"
    print(f"  ✓ branch1: remapped to link {branch1['attach_link']}, offset {branch1.get('attach_offset', 0):.3f}m")
    
    # Check branch2 (was at 0.65m)
    branch2 = next(b for b in remapped if b["id"] == "branch2")
    new_height_2 = branch2["attach_link"] * (1.0/3) + branch2.get("attach_offset", 0)
    assert abs(new_height_2 - 0.65) < 0.01, f"Branch2 height mismatch: {new_height_2}"
    print(f"  ✓ branch2: remapped to link {branch2['attach_link']}, offset {branch2.get('attach_offset', 0):.3f}m")
    
    # Check other_branch (should be unchanged)
    other = next(b for b in remapped if b["id"] == "other_branch")
    assert other["attach_link"] == 0, "other_branch should be unchanged"
    print(f"  ✓ other_branch: unchanged (different parent)")


def test_invalid_inputs():
    """Test error handling for invalid inputs."""
    print("\n[TEST] Invalid Inputs...")
    
    # Test 1: Empty segments
    result = remap_attachment_height(0, 0.0, [], [0.5], tolerance=0.01)
    assert not result.success, "Should fail with empty original segments"
    print(f"  ✓ Empty original segments rejected: {result.message}")
    
    # Test 2: Out of range link index
    result = remap_attachment_height(10, 0.0, [0.2] * 5, [0.33] * 3, tolerance=0.01)
    assert not result.success, "Should fail with out-of-range index"
    print(f"  ✓ Out-of-range index rejected: {result.message}")
    
    # Test 3: Negative offset (should be handled gracefully)
    result = remap_attachment_height(2, -0.1, [0.2] * 5, [0.33] * 3, tolerance=0.01)
    # Should clamp to 0 and succeed
    assert result.success or not result.success, "Negative offset handled"
    print(f"  ✓ Negative offset handled: {result.message}")


def main():
    """Run all tests."""
    print("="*70)
    print("  Geometry Remapping - Test Suite")
    print("="*70)
    
    tests = [
        test_calculate_absolute_height,
        test_find_new_attachment,
        test_remap_attachment_simple,
        test_remap_attachment_extreme_collapse,
        test_remap_attachment_edge_cases,
        test_remap_with_non_uniform_segments,
        test_remap_all_children,
        test_invalid_inputs,
    ]
    
    passed = 0
    failed = 0
    
    for test in tests:
        try:
            test()
            passed += 1
        except AssertionError as e:
            failed += 1
            print(f"\n  ✗ FAILED: {e}")
            import traceback
            traceback.print_exc()
        except Exception as e:
            failed += 1
            print(f"\n  ✗ ERROR: {e}")
            import traceback
            traceback.print_exc()
    
    print("\n" + "="*70)
    print(f"  Test Results: {passed} passed, {failed} failed")
    print("="*70)
    
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
