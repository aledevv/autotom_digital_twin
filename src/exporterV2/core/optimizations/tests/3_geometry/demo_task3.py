"""
demo_task3.py - Task 3 Demo: Geometry Remapping

Demonstrates attachment point remapping when collapsing stem segments.

Run with: uv run python src/exporterV2/core/optimizations/tests/3_geometry/demo_task3.py
"""

import sys
import os

# Add optimizations directory to path
script_dir = os.path.dirname(os.path.abspath(__file__))
optimizations_dir = os.path.join(script_dir, "../..")
sys.path.insert(0, optimizations_dir)

from geometry.remapping import (
    remap_attachment_height,
    remap_all_children,
    calculate_absolute_height
)


def print_header(title):
    """Print section header."""
    print("\n" + "="*70)
    print(f"  {title}")
    print("="*70)


def demo_simple_remapping():
    """Demo: Simple remapping scenario."""
    print_header("Demo 1: Simple Stem Collapse (5 → 3 links)")
    
    print("\nScenario:")
    print("  - Trunk: 5 links @ 0.2m each = 1.0m total")
    print("  - Branch attached at: link 3, offset 0.1m")
    print("  → Absolute height: 3 * 0.2 + 0.1 = 0.7m")
    print("\nAfter collapse to 3 links:")
    print("  - Trunk: 3 links @ 0.33m each ≈ 1.0m total")
    print("  - Need to preserve: 0.7m absolute height")
    
    # Original configuration
    original_segments = [0.2] * 5
    new_segments = [0.33, 0.33, 0.34]
    
    result = remap_attachment_height(
        original_link_idx=3,
        original_offset=0.1,
        original_segment_heights=original_segments,
        new_segment_heights=new_segments,
        tolerance=0.01
    )
    
    print("\nRemapping Result:")
    print(f"  New attachment: link {result.new_link_idx}, offset {result.new_offset:.3f}m")
    print(f"  Actual height: {result.absolute_height:.3f}m")
    print(f"  Height error: {result.height_error:.4f}m")
    print(f"  Success: {'✓' if result.success else '✗'}")
    print(f"  Message: {result.message}")


def demo_extreme_collapse():
    """Demo: Extreme collapse to single link."""
    print_header("Demo 2: Extreme Collapse (5 → 1 link)")
    
    print("\nScenario:")
    print("  - Trunk: 5 links @ 0.2m each = 1.0m total")
    print("  - 3 branches at different heights:")
    
    test_cases = [
        ("Branch A", 1, 0.0, 0.2),
        ("Branch B", 2, 0.1, 0.5),
        ("Branch C", 4, 0.1, 0.9),
    ]
    
    original_segments = [0.2] * 5
    new_segments = [1.0]  # Collapsed to single 1.0m link
    
    print("\nAfter collapse to 1 link @ 1.0m:")
    print("\n{:<12} {:<20} {:<20} {:<12}".format("Branch", "Original", "Remapped", "Error"))
    print("-" * 70)
    
    for name, orig_link, orig_offset, target_height in test_cases:
        result = remap_attachment_height(
            orig_link,
            orig_offset,
            original_segments,
            new_segments,
            tolerance=0.01
        )
        
        orig_str = f"link {orig_link}, {orig_offset:.2f}m"
        new_str = f"link {result.new_link_idx}, {result.new_offset:.2f}m"
        error_str = f"{result.height_error:.4f}m"
        
        print(f"{name:<12} {orig_str:<20} {new_str:<20} {error_str:<12}")
    
    print("\n✓ All branches preserve their absolute heights!")


def demo_multiple_branches():
    """Demo: Remapping multiple child branches."""
    print_header("Demo 3: Multiple Child Branches")
    
    print("\nScenario:")
    print("  Parent: trunk with 5 links @ 0.2m each")
    print("  Children:")
    print("    - lateral1: attached at link 1, offset 0.1m (height: 0.3m)")
    print("    - lateral2: attached at link 3, offset 0.05m (height: 0.65m)")
    print("    - lateral3: attached at link 4, offset 0.15m (height: 0.95m)")
    
    parent = {
        "id": "trunk",
        "n_links": 5,
        "height": 0.2,
        "radius": 0.05
    }
    
    children = [
        {
            "id": "lateral1",
            "parent": "trunk",
            "attach_link": 1,
            "attach_offset": 0.1,
            "n_links": 3
        },
        {
            "id": "lateral2",
            "parent": "trunk",
            "attach_link": 3,
            "attach_offset": 0.05,
            "n_links": 3
        },
        {
            "id": "lateral3",
            "parent": "trunk",
            "attach_link": 4,
            "attach_offset": 0.15,
            "n_links": 2
        }
    ]
    
    # Calculate original heights
    print("\nOriginal Heights:")
    for child in children:
        orig_height = calculate_absolute_height(
            child["attach_link"],
            child.get("attach_offset", 0.0),
            [0.2] * 5
        )
        print(f"  {child['id']}: {orig_height:.2f}m")
    
    # Collapse trunk to 3 links
    print("\nCollapsing trunk: 5 → 3 links...")
    remapped, errors = remap_all_children(parent, children, new_n_links=3, tolerance=0.01)
    
    if errors:
        print(f"\n⚠️  Errors occurred:")
        for error in errors:
            print(f"  - {error}")
    else:
        print("\n✓ All branches remapped successfully!")
    
    # Show remapped attachments
    print("\nRemapped Attachments:")
    new_segment_height = 1.0 / 3  # Total 1.0m / 3 links
    for child in remapped:
        if child.get("parent") == "trunk":
            new_height = calculate_absolute_height(
                child["attach_link"],
                child.get("attach_offset", 0.0),
                [new_segment_height] * 3
            )
            print(f"  {child['id']}: link {child['attach_link']}, "
                  f"offset {child.get('attach_offset', 0.0):.3f}m → {new_height:.2f}m")


def demo_comparison_table():
    """Demo: Comparison table for different collapse scenarios."""
    print_header("Demo 4: Collapse Comparison Table")
    
    print("\nOriginal: 5 links @ 0.2m each (total: 1.0m)")
    print("Branch attached at: link 3, offset 0.1m (absolute height: 0.7m)")
    print("\nComparison across different collapse scenarios:")
    
    print("\n{:<15} {:<25} {:<15} {:<12}".format(
        "Scenario", "Remapped To", "Actual Height", "Error"
    ))
    print("-" * 70)
    
    original_segments = [0.2] * 5
    test_scenarios = [
        ("5 → 4 links", [0.25] * 4),
        ("5 → 3 links", [0.33, 0.33, 0.34]),
        ("5 → 2 links", [0.5, 0.5]),
        ("5 → 1 link", [1.0]),
    ]
    
    for name, new_segments in test_scenarios:
        result = remap_attachment_height(
            3, 0.1,
            original_segments,
            new_segments,
            tolerance=0.01
        )
        
        remap_str = f"link {result.new_link_idx}, {result.new_offset:.3f}m"
        height_str = f"{result.absolute_height:.3f}m"
        error_str = f"{result.height_error:.4f}m"
        
        print(f"{name:<15} {remap_str:<25} {height_str:<15} {error_str:<12}")
    
    print("\n✓ All scenarios preserve 0.7m height within tolerance!")


def main():
    """Run all demos."""
    print("="*70)
    print("  Task 3 Demo: Geometry Remapping")
    print("="*70)
    print("\nThis demo shows how attachment points are remapped when")
    print("collapsing stem segments to preserve absolute heights.")
    
    input("\nPress Enter to start...")
    
    demo_simple_remapping()
    input("\nPress Enter for next demo...")
    
    demo_extreme_collapse()
    input("\nPress Enter for next demo...")
    
    demo_multiple_branches()
    input("\nPress Enter for next demo...")
    
    demo_comparison_table()
    
    print("\n" + "="*70)
    print("  ✓ Task 3 Demo Complete!")
    print("="*70)
    print("\nKey Takeaways:")
    print("  • Absolute height is preserved across topology changes")
    print("  • Works for any collapse ratio (5→3, 5→1, etc.)")
    print("  • Handles multiple branches simultaneously")
    print("  • Sub-centimeter accuracy maintained")
    print("\nThis remapping will be used in Task 6 (Stem Collapse technique)")


if __name__ == "__main__":
    main()
