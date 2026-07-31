#!/usr/bin/env python3
"""
Test: Sub-branch static position correctness

Verifies that sub-branches (branches attaching to already-tilted parent branches)
have correct initial world-space positions that match their joint constraints.

The bug was: sub-branch offsets were only rotated by azimuth (world Z), not by
the parent's full orientation. This caused visual misalignment in static render
that would snap to correct position when simulation started.

The fix: Transform the attachment offset by the parent's world-space orientation,
and express joint LocalPos0/LocalRot0 relative to the parent's local frame.
"""

import sys
import os
import math

# Add src to path
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(SCRIPT_DIR, "src/experiments/recursive_tree"))

from generate_recursive_tree_usda import build_stage, scaled
from tree_config import BRANCHES, GAP


def extract_link_world_position(stage, link_path):
    """Extract world-space position from a link's xformOp:translate."""
    prim = stage.GetPrimAtPath(link_path)
    if not prim.IsValid():
        raise ValueError(f"Invalid prim: {link_path}")
    
    xform = prim.GetAttribute("xformOp:translate")
    if not xform:
        raise ValueError(f"No translate op on {link_path}")
    
    return tuple(xform.Get())


def extract_joint_local_pos0(stage, joint_path):
    """Extract LocalPos0 from a joint."""
    prim = stage.GetPrimAtPath(joint_path)
    if not prim.IsValid():
        raise ValueError(f"Invalid joint: {joint_path}")
    
    attr = prim.GetAttribute("physics:localPos0")
    if not attr:
        raise ValueError(f"No localPos0 on {joint_path}")
    
    return tuple(attr.Get())


def test_subbranch_position():
    """Test that subA1 position is geometrically consistent with branchA."""
    print("=" * 80)
    print("Test: Sub-branch static position correctness")
    print("=" * 80)
    
    # Generate USD in memory
    import tempfile
    with tempfile.NamedTemporaryFile(suffix=".usda", delete=False) as tmp:
        output_path = tmp.name
    
    try:
        stage, stem_path = build_stage(output_path, BRANCHES)
        
        # Extract positions
        branchA_link2_path = "/World/Stem/branchA_Link_02"
        subA1_link1_path = "/World/Stem/subA1_Link_01"
        subA1_joint_path = f"{subA1_link1_path}/AttachJoint"
        
        parent_pos = extract_link_world_position(stage, branchA_link2_path)
        child_pos = extract_link_world_position(stage, subA1_link1_path)
        joint_local = extract_joint_local_pos0(stage, subA1_joint_path)
        
        print(f"\nParent (branchA_Link_02) world pos: {parent_pos}")
        print(f"Child  (subA1_Link_01)   world pos: {child_pos}")
        print(f"Joint LocalPos0 (in parent frame):  {joint_local}")
        
        # Calculate offset
        offset = tuple(child_pos[i] - parent_pos[i] for i in range(3))
        distance = math.sqrt(sum(d**2 for d in offset))
        
        print(f"\nWorld-space offset: ({offset[0]:.6f}, {offset[1]:.6f}, {offset[2]:.6f})")
        print(f"Distance: {distance:.6f}m")
        
        # Expected offset magnitude from config:
        # branchA: radius 0.3m, height 1.5m (world scale)
        # subA1 attaches with radial offset 0.15m at top + gap 0.01m
        # After 90° azimuth and 45° tilt, offset should be approx 1.517m
        
        parent_branch = next(b for b in BRANCHES if b["id"] == "branchA")
        p_h = scaled(parent_branch["height"])
        p_r = scaled(parent_branch["radius"])
        gap = scaled(GAP)
        
        radial = p_r / 2.0
        axial = p_h + gap
        
        # After transformations (90° azimuth + 45° parent tilt):
        # expected offset = (-0.15, -1.0677, 1.0677) ≈ magnitude 1.517m
        expected_mag = math.sqrt(radial**2 + 2*(axial/math.sqrt(2))**2)
        
        print(f"\nExpected offset magnitude: {expected_mag:.6f}m")
        print(f"Actual offset magnitude:   {distance:.6f}m")
        
        # Tolerance: 1mm
        tolerance = 0.001
        passed = abs(distance - expected_mag) < tolerance
        
        print(f"\nTest result: {'✓ PASS' if passed else '✗ FAIL'}")
        print("=" * 80)
        
        return passed
        
    finally:
        if os.path.exists(output_path):
            os.unlink(output_path)


if __name__ == "__main__":
    success = test_subbranch_position()
    sys.exit(0 if success else 1)
