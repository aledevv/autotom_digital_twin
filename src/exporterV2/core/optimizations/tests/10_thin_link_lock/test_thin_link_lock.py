"""
test_thin_link_lock.py - Unit Tests for Thin Link Lock Technique

Tests for converting thin link joints from D6 to Fixed.

Run with: uv run python src/exporterV2/core/optimizations/tests/10_thin_link_lock/test_thin_link_lock.py
"""

import sys
import os

# Add optimizations directory to path
script_dir = os.path.dirname(os.path.abspath(__file__))
optimizations_dir = os.path.join(script_dir, "../..")
sys.path.insert(0, optimizations_dir)

from techniques.thin_link_lock import ThinLinkLockTechnique


def test_identify_thin_links():
    """Test thin link identification."""
    print("\n[TEST] Identify Thin Links...")
    
    technique = ThinLinkLockTechnique()
    
    # MIN_LINK_RADIUS_WORLD is 0.002, GLOBAL_SCALE is 2.0
    # pre-scale target is <= 0.001
    branches = [
        {"id": "trunk", "n_links": 5, "radius": 0.05},           # 0.100m world -> not target
        {"id": "branch_1", "n_links": 3, "radius": 0.005},       # 0.010m world -> not target
        {"id": "branch_2", "n_links": 2, "radius": 0.001},       # 0.002m world -> target
        {"id": "branch_3", "n_links": 1, "radius": 0.0005},      # 0.001m world -> target
        {"id": "branch_4", "n_links": 1, "radius": 0.0010001},   # 0.0020002m world -> target (within 1e-6 epsilon)
        {"id": "branch_5", "n_links": 1, "radius": 0.002},       # 0.004m world -> not target
    ]
    
    target_count = sum(1 for b in branches if technique._is_target(b))
    assert target_count == 3, f"Expected 3 thin links, found {target_count}"
    print(f"  ✓ Identified {target_count} thin links")


def test_can_apply():
    """Test can_apply() method."""
    print("\n[TEST] Can Apply...")
    
    technique = ThinLinkLockTechnique()
    
    # Test 1: With thin links
    branches_with = [
        {"id": "trunk", "n_links": 5, "radius": 0.05},
        {"id": "thin", "n_links": 1, "radius": 0.001},
    ]
    
    assert technique.can_apply(branches_with), "Should be applicable with thin links"
    print("  ✓ Can apply with thin links")
    
    # Test 2: Without thin links
    branches_without = [
        {"id": "trunk", "n_links": 5, "radius": 0.05},
        {"id": "thick", "n_links": 3, "radius": 0.03},
    ]
    
    assert not technique.can_apply(branches_without), "Should not be applicable without thin links"
    print("  ✓ Cannot apply without thin links")
    
    # Test 3: With thin links already fixed
    branches_fixed = [
        {"id": "thin", "n_links": 1, "radius": 0.001, "joint_type": "fixed"},
    ]
    
    assert not technique.can_apply(branches_fixed), "Should not apply to already-fixed thin links"
    print("  ✓ Cannot apply to already-fixed thin links")


def test_estimate_reduction():
    """Test joint reduction estimation."""
    print("\n[TEST] Estimate Reduction...")
    
    technique = ThinLinkLockTechnique()
    
    branches = [
        {"id": "trunk", "n_links": 5, "radius": 0.05},
        {"id": "thin_1", "n_links": 2, "radius": 0.001},         # target, 2 links
        {"id": "thin_2", "n_links": 3, "radius": 0.0005},        # target, 3 links
        {"id": "thick", "n_links": 1, "radius": 0.05},           # not target
        {"id": "thin_3", "n_links": 4, "radius": 0.001, "joint_type": "fixed"},  # target, already fixed
    ]
    
    reduction = technique.estimate_reduction(branches)
    # 2 + 3 = 5 joints reduction
    assert reduction == 5, f"Expected 5 joints reduction, got {reduction}"
    print(f"  ✓ Estimated {reduction} joints reduction (sum of n_links)")


def test_apply_simple():
    """Test applying technique to simple case."""
    print("\n[TEST] Apply - Simple Case...")
    
    technique = ThinLinkLockTechnique()
    
    branches = [
        {"id": "trunk", "n_links": 5, "radius": 0.05},
        {"id": "thin_1", "n_links": 2, "radius": 0.001},
        {"id": "thin_2", "n_links": 1, "radius": 0.0005},
    ]
    
    modified, report = technique.apply(branches)
    
    # n_links for targets: 2 + 1 = 3 joints saved
    assert report.joints_saved == 3, f"Should report 3 joints saved, got {report.joints_saved}"
    assert report.details["dof_reduced"] == 12, f"Expected 12 DOF reduced (2 targets * 6), got {report.details['dof_reduced']}"
    assert report.details["items_locked"] == 2, f"Expected 2 items locked"
    
    # Check that thin links got joint_type metadata
    locked_links = [b for b in modified if b["id"].startswith("thin") and b.get("joint_type") == "fixed"]
    assert len(locked_links) == 2, f"Expected 2 locked links, got {len(locked_links)}"
    
    print(f"  ✓ Locked {len(locked_links)} thin links")
    print(f"  ✓ Report: Joints saved = {report.joints_saved}, DOF reduced = {report.details['dof_reduced']}")


def test_apply_preserves_geometry():
    """Test that apply() preserves geometry."""
    print("\n[TEST] Apply Preserves Geometry...")
    
    technique = ThinLinkLockTechnique()
    
    branches = [
        {"id": "trunk", "n_links": 5, "height": 0.2, "radius": 0.05},
        {"id": "thin_link", "n_links": 2, "height": 0.1, "radius": 0.001, "parent": "trunk"},
    ]
    
    modified, report = technique.apply(branches)
    
    # Check trunk unchanged
    trunk_orig = branches[0]
    trunk_mod = [b for b in modified if b["id"] == "trunk"][0]
    
    assert trunk_mod["n_links"] == trunk_orig["n_links"], "Trunk n_links changed"
    assert trunk_mod["height"] == trunk_orig["height"], "Trunk height changed"
    assert trunk_mod["radius"] == trunk_orig["radius"], "Trunk radius changed"
    
    # Check thin link geometry unchanged (only joint_type added)
    thin_orig = branches[1]
    thin_mod = [b for b in modified if b["id"] == "thin_link"][0]
    
    assert thin_mod["n_links"] == thin_orig["n_links"], "Thin link n_links changed"
    assert thin_mod["height"] == thin_orig["height"], "Thin link height changed"
    assert thin_mod["radius"] == thin_orig["radius"], "Thin link radius changed"
    assert thin_mod.get("joint_type") == "fixed", "Thin link should have fixed joint"
    
    print("  ✓ Geometry preserved (n_links, height, radius)")
    print("  ✓ Only joint_type metadata added")


def main():
    """Run all tests."""
    print("="*70)
    print("  Thin Link Lock Technique - Test Suite")
    print("="*70)
    
    tests = [
        test_identify_thin_links,
        test_can_apply,
        test_estimate_reduction,
        test_apply_simple,
        test_apply_preserves_geometry,
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
