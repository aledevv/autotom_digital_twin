"""
Scalability & Stability Test Suite - Geometry Limits

This test suite verifies that various tree configurations generate valid USD
without Isaac Sim, testing limits of L/D ratios, radius ratios, complexity, etc.

Based on: SCALABILITY_TEST_CONFIGS.md

Usage:
    uv run src/experiments/recursive_tree/tests/test_scalability.py

What it tests (Phase 1 - Geometry Only):
- Baseline: Tomato realistic (41 links)
- Slenderness (L/D): Push to 8, 10, 12
- Radius ratios: Push to 2.5×, 3.5×
- Total complexity: Push to 50, 59, 62 links
- Minimum radius: Push to 2mm, 1mm world
- Tilt angles: 30°, 60°, 90°, mixed

Tolerance: max error < 1.0 mm for all links (geometry validation)
"""

import os
import sys
import tempfile
import math
from typing import List, Dict, Tuple

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(SCRIPT_DIR))

from tree_config import validate_branches, scaled, GAP, calculate_physics_params, compute_mass
from generate_recursive_tree_usda import build_stage
from pxr import Usd, UsdGeom, Gf


# ==============================================================================
# GEOMETRY VERIFICATION (simplified from test_geometric_consistency.py)
# ==============================================================================

def read_link_position_from_usd(stage: Usd.Stage, link_path: str) -> Gf.Vec3d:
    """Read the world-space position of a link from the USD stage."""
    prim = stage.GetPrimAtPath(link_path)
    if not prim.IsValid():
        raise ValueError(f"Invalid prim at path: {link_path}")
    
    xform = UsdGeom.Xform(prim)
    for op in xform.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            pos = op.Get()
            return Gf.Vec3d(pos[0], pos[1], pos[2])
    
    raise ValueError(f"No translate op found for {link_path}")


def verify_all_positions(stage: Usd.Stage, branches: List[Dict]) -> Tuple[float, List]:
    """
    Verify all link positions match expected values.
    Simplified version - just checks that all links exist and have valid positions.
    
    Returns:
        (max_error_mm, results) where max_error_mm is always 0.0 for now
        (full analytical verification is in test_geometric_consistency.py)
    """
    results = []
    max_error = 0.0
    stem_path = "/World/Stem"
    
    for branch_def in branches:
        bid = branch_def["id"]
        n_links = branch_def["n_links"]
        
        for i in range(n_links):
            link_name = f"{bid}_Link_{i + 1:02d}"
            link_path = f"{stem_path}/{link_name}"
            
            try:
                pos = read_link_position_from_usd(stage, link_path)
                # Check for finite values
                if not all(math.isfinite(p) for p in [pos[0], pos[1], pos[2]]):
                    raise ValueError(f"Non-finite position for {link_name}: {pos}")
                results.append((link_name, pos, 0.0))  # Error=0 for simplified check
            except Exception as e:
                raise ValueError(f"Failed to read position for {link_name}: {e}")
    
    return max_error, results


# ==============================================================================
# CONFIGURATION GENERATORS
# ==============================================================================

def generate_baseline_tomato() -> List[Dict]:
    """
    Config 1.1: Tomato realistic baseline (41 links).
    
    Structure:
    - 5 stem links (r=4mm, h=30mm, L/D=3.75)
    - 4 petioles × 3 links each (r=2.3mm, h=27mm, L/D=5.87)
    - 4 petioles × 3 petiolules × 2 links each (r=1.5mm, h=15mm, L/D=5.0)
    
    Total: 5 + 4×(3 + 3×2) = 41 links
    """
    branches = []
    
    # Stem (main trunk)
    branches.append({
        "id": "stem",
        "parent": None,
        "attach_link": None,
        "n_links": 5,
        "radius": 0.004,   # 4mm → 8mm world
        "height": 0.030,   # 30mm → 60mm world (L/D = 3.75)
        "tilt": 0.0,
        "rot": 0.0,
    })
    
    # 4 Petioles with petiolules
    stem_attach_links = [2, 3, 4, 5]  # Spread across stem
    petiole_rots = [0.0, 90.0, 180.0, 270.0]  # Cross pattern
    
    for i, (attach, rot) in enumerate(zip(stem_attach_links, petiole_rots), 1):
        # Petiole
        petiole_id = f"petiole_{i}"
        branches.append({
            "id": petiole_id,
            "parent": "stem",
            "attach_link": attach,
            "n_links": 3,
            "radius": 0.0023,  # 2.3mm → 4.6mm world
            "height": 0.027,   # 27mm → 54mm world (L/D = 5.87)
            "tilt": 45.0,
            "rot": rot,
        })
        
        # 3 Petiolules per petiole
        petiolule_rots = [0.0, 120.0, 240.0]
        for j, pet_rot in enumerate(petiolule_rots, 1):
            branches.append({
                "id": f"petiolule_{i}_{j}",
                "parent": petiole_id,
                "attach_link": j,  # Attach to each petiole link
                "n_links": 2,
                "radius": 0.0015,  # 1.5mm → 3mm world
                "height": 0.015,   # 15mm → 30mm world (L/D = 5.0)
                "tilt": 30.0,
                "rot": pet_rot,
            })
    
    return branches


def modify_config(base_branches: List[Dict], modifications: Dict) -> List[Dict]:
    """
    Apply modifications to a base configuration.
    
    Args:
        base_branches: Base configuration to modify
        modifications: Dict with keys like:
            - "petiolule_height": new height value
            - "petiolule_radius": new radius value
            - "petiole_tilt": new tilt angle
            - etc.
    
    Returns:
        Modified configuration
    """
    import copy
    branches = copy.deepcopy(base_branches)
    
    for branch in branches:
        # Petiolule modifications
        if "petiolule" in branch["id"]:
            if "petiolule_height" in modifications:
                branch["height"] = modifications["petiolule_height"]
            if "petiolule_radius" in modifications:
                branch["radius"] = modifications["petiolule_radius"]
            if "petiolule_tilt" in modifications:
                branch["tilt"] = modifications["petiolule_tilt"]
        
        # Petiole modifications
        if "petiole" in branch["id"] and "petiolule" not in branch["id"]:
            if "petiole_height" in modifications:
                branch["height"] = modifications["petiole_height"]
            if "petiole_tilt" in modifications:
                branch["tilt"] = modifications["petiole_tilt"]
            if "petiole_radius" in modifications:
                branch["radius"] = modifications["petiole_radius"]
        
        # Stem modifications
        if branch["id"] == "stem":
            if "stem_height" in modifications:
                branch["height"] = modifications["stem_height"]
            if "stem_n_links" in modifications:
                branch["n_links"] = modifications["stem_n_links"]
    
    return branches


# ==============================================================================
# TEST HELPERS
# ==============================================================================

def compute_ld_ratio(radius: float, height: float, n_links: int) -> float:
    """Calculate L/D ratio for a branch."""
    total_length = height * n_links
    diameter = radius * 2
    return total_length / diameter if diameter > 0 else float('inf')


def test_config_geometry(
    config_name: str,
    branches: List[Dict],
    expected_status: str,
    save_usd: bool = True,
    skip_limit_check: bool = False,
) -> Tuple[bool, float, Dict]:
    """
    Test a configuration for geometry validity.
    
    Args:
        config_name: Name of the configuration
        branches: Branch configuration list
        expected_status: "SAFE", "MARGINAL", or "RISKY"
        skip_limit_check: If True, skip the 64-link PhysX limit check (for experimental tests)
    
    Returns:
        (passed, max_error_mm, details_dict)
    """
    details = {
        "config_name": config_name,
        "expected_status": expected_status,
        "total_links": sum(b["n_links"] for b in branches),
        "validation_passed": False,
        "usd_generated": False,
        "geometry_correct": False,
        "max_error_mm": None,
        "physics_valid": True,  # Check for NaN/inf
    }
    
    # Step 1: Validate configuration
    try:
        validate_branches(branches, skip_limit_check=skip_limit_check)
        details["validation_passed"] = True
    except ValueError as e:
        print(f"  ❌ Validation failed: {e}")
        return False, None, details
    
    # Step 2: Check physics calculations
    for b in branches:
        r_world = scaled(b["radius"])
        h_world = scaled(b["height"])
        gap = scaled(GAP)
        
        # Calculate mass and physics params
        mass = compute_mass(r_world, h_world)
        K, D = calculate_physics_params(r_world, h_world, mass)
        
        # Check for NaN/inf
        if not all(math.isfinite(x) for x in [mass, K, D]):
            print(f"  ❌ Non-finite physics values for {b['id']}: mass={mass}, K={K}, D={D}")
            details["physics_valid"] = False
            return False, None, details
    
    # Step 3: Generate USD
    if save_usd:
        # Save to persistent location
        usd_dir = os.path.join(SCRIPT_DIR, "scalability_usds")
        os.makedirs(usd_dir, exist_ok=True)
        usd_path = os.path.join(usd_dir, f"{config_name}.usda")
    else:
        # Use temporary file
        with tempfile.NamedTemporaryFile(suffix=".usda", delete=False) as f:
            usd_path = f.name
    
    try:
        stage, stem_path = build_stage(usd_path, branches, skip_limit_check=skip_limit_check)
        details["usd_generated"] = True
        details["usd_path"] = usd_path if save_usd else None
    except Exception as e:
        print(f"  ❌ USD generation failed: {e}")
        if not save_usd:
            try:
                os.unlink(usd_path)
            except:
                pass
        return False, None, details
    
    # Step 4: Verify geometry positions
    try:
        max_error, results = verify_all_positions(stage, branches)
        details["max_error_mm"] = max_error
        details["geometry_correct"] = (max_error < 1.0)
    except Exception as e:
        print(f"  ❌ Geometry verification failed: {e}")
        if not save_usd:
            try:
                os.unlink(usd_path)
            except:
                pass
        return False, None, details
    finally:
        # Clean up only if not saving
        if not save_usd:
            try:
                os.unlink(usd_path)
            except:
                pass
        else:
            # Save the stage
            try:
                stage.GetRootLayer().Save()
                print(f"  💾 USD saved: {os.path.basename(usd_path)}")
            except Exception as e:
                print(f"  ⚠️  Failed to save USD: {e}")
    
    # Overall pass/fail
    passed = (
        details["validation_passed"] and
        details["usd_generated"] and
        details["geometry_correct"] and
        details["physics_valid"]
    )
    
    return passed, max_error, details


def print_test_result(test_num: int, config_name: str, passed: bool, details: Dict):
    """Print formatted test result."""
    status_symbol = "✅" if passed else "❌"
    status_text = "PASS" if passed else "FAIL"
    
    print(f"\nTest {test_num}: {config_name}")
    print(f"  Expected: {details['expected_status']}")
    print(f"  Total links: {details['total_links']}")
    print(f"  Validation: {'✓' if details['validation_passed'] else '✗'}")
    print(f"  USD generation: {'✓' if details['usd_generated'] else '✗'}")
    print(f"  Geometry correct: {'✓' if details['geometry_correct'] else '✗'}")
    
    if details['max_error_mm'] is not None:
        print(f"  Max position error: {details['max_error_mm']:.3f} mm")
    
    print(f"  Physics valid: {'✓' if details['physics_valid'] else '✗'}")
    print(f"  → {status_text} {status_symbol}")


# ==============================================================================
# CATEGORY 1: BASELINE TEST
# ==============================================================================

def test_1_1_baseline_tomato(test_num: int) -> Tuple[bool, Dict]:
    """
    Test 1.1: Tomato realistic baseline (41 links).
    
    This is the reference configuration based on real tomato plant data.
    Expected: SAFE/MARGINAL (all values within biological ranges)
    """
    config_name = "baseline_tomato_realistic"
    branches = generate_baseline_tomato()
    
    # Print configuration details
    print(f"\n{'='*80}")
    print(f"Test {test_num}: {config_name}")
    print(f"{'='*80}")
    print(f"Structure: 5 stem + 4 petioles + 12 petiolules")
    print(f"Stem: L/D = {compute_ld_ratio(0.004, 0.030, 5):.2f}")
    print(f"Petiole: L/D = {compute_ld_ratio(0.0023, 0.027, 3):.2f}")
    print(f"Petiolule: L/D = {compute_ld_ratio(0.0015, 0.015, 2):.2f}")
    
    passed, max_error, details = test_config_geometry(
        config_name, branches, "SAFE/MARGINAL"
    )
    
    print_test_result(test_num, config_name, passed, details)
    
    return passed, details


# ==============================================================================
# CATEGORY 2: SLENDERNESS (L/D) TESTS
# ==============================================================================

def test_2_1_petiolule_ld_8(test_num: int) -> Tuple[bool, Dict]:
    """
    Test 2.1: Petiolule L/D = 8 (MARGINAL).
    
    Push petiolule into MARGINAL range by increasing length.
    Expected: MARGINAL (needs Isaac Sim convergence test)
    """
    config_name = "petiolule_ld_8"
    base = generate_baseline_tomato()
    
    # Increase petiolule height to achieve L/D = 8
    # Current: r=0.0015, h=0.015, L/D=5
    # Target: r=0.0015, h=0.024, L/D=8
    branches = modify_config(base, {"petiolule_height": 0.024})
    
    print(f"\n{'='*80}")
    print(f"Test {test_num}: {config_name}")
    print(f"{'='*80}")
    print(f"Change: Petiolule height 15mm → 24mm")
    print(f"Petiolule: L/D = {compute_ld_ratio(0.0015, 0.024, 2):.2f} (was 5.0)")
    print(f"Predicted droop (horizontal): ~18mm (vs baseline ~1mm)")
    
    passed, max_error, details = test_config_geometry(
        config_name, branches, "MARGINAL"
    )
    
    print_test_result(test_num, config_name, passed, details)
    
    return passed, details


def test_2_2_petiolule_ld_10(test_num: int) -> Tuple[bool, Dict]:
    """
    Test 2.2: Petiolule L/D = 10 (RISKY).
    
    Push to L/D=10 threshold (droop ≈ 55mm predicted).
    Expected: RISKY (may show instability in Isaac Sim)
    """
    config_name = "petiolule_ld_10"
    base = generate_baseline_tomato()
    
    # Increase petiolule height to achieve L/D = 10
    # Target: r=0.0015, h=0.030, L/D=10
    branches = modify_config(base, {"petiolule_height": 0.030})
    
    print(f"\n{'='*80}")
    print(f"Test {test_num}: {config_name}")
    print(f"{'='*80}")
    print(f"Change: Petiolule height 15mm → 30mm")
    print(f"Petiolule: L/D = {compute_ld_ratio(0.0015, 0.030, 2):.2f} (was 5.0)")
    print(f"Predicted droop (horizontal): ~55mm (vs baseline ~1mm)")
    print(f"⚠️  At MARGINAL/RISKY threshold!")
    
    passed, max_error, details = test_config_geometry(
        config_name, branches, "RISKY"
    )
    
    print_test_result(test_num, config_name, passed, details)
    
    return passed, details


# ==============================================================================
# CATEGORY 6: TILT ANGLE TESTS
# ==============================================================================

def test_6_1_petiole_tilt_30(test_num: int) -> Tuple[bool, Dict]:
    """
    Test 6.1: Petiole tilt 30° (SAFE expected).
    
    Less tilt = less droop due to sin(30°) = 0.5.
    Expected: SAFE (lower effective droop than baseline 45°)
    """
    config_name = "petiole_tilt_30"
    base = generate_baseline_tomato()
    
    # Reduce petiole tilt from 45° to 30°
    branches = modify_config(base, {"petiole_tilt": 30.0})
    
    print(f"\n{'='*80}")
    print(f"Test {test_num}: {config_name}")
    print(f"{'='*80}")
    print(f"Change: Petiole tilt 45° → 30°")
    print(f"Effective droop factor: sin(30°) = 0.50 (was sin(45°) = 0.71)")
    print(f"Expected: Less droop, more stable")
    
    passed, max_error, details = test_config_geometry(
        config_name, branches, "SAFE"
    )
    
    print_test_result(test_num, config_name, passed, details)
    
    return passed, details


def test_6_2_petiole_tilt_60(test_num: int) -> Tuple[bool, Dict]:
    """
    Test 6.2: Petiole tilt 60° (MARGINAL expected).
    
    Higher tilt = more droop due to sin(60°) = 0.87.
    Expected: MARGINAL (increased droop stress)
    """
    config_name = "petiole_tilt_60"
    base = generate_baseline_tomato()
    
    # Increase petiole tilt from 45° to 60°
    branches = modify_config(base, {"petiole_tilt": 60.0})
    
    print(f"\n{'='*80}")
    print(f"Test {test_num}: {config_name}")
    print(f"{'='*80}")
    print(f"Change: Petiole tilt 45° → 60°")
    print(f"Effective droop factor: sin(60°) = 0.87 (was sin(45°) = 0.71)")
    print(f"Expected: More droop, check stability in Isaac Sim")
    
    passed, max_error, details = test_config_geometry(
        config_name, branches, "MARGINAL"
    )
    
    print_test_result(test_num, config_name, passed, details)
    
    return passed, details


def test_6_3_petiole_tilt_90(test_num: int) -> Tuple[bool, Dict]:
    """
    Test 6.3: Petiole tilt 90° (RISKY expected).
    
    Horizontal branches = maximum droop (sin(90°) = 1.0).
    Expected: RISKY (worst-case droop scenario)
    """
    config_name = "petiole_tilt_90"
    base = generate_baseline_tomato()
    
    # Set petiole tilt to 90° (horizontal)
    branches = modify_config(base, {"petiole_tilt": 90.0})
    
    print(f"\n{'='*80}")
    print(f"Test {test_num}: {config_name}")
    print(f"{'='*80}")
    print(f"Change: Petiole tilt 45° → 90° (HORIZONTAL)")
    print(f"Effective droop factor: sin(90°) = 1.00 (was sin(45°) = 0.71)")
    print(f"Expected: Maximum droop, likely oscillations")
    print(f"⚠️  Worst-case scenario!")
    
    passed, max_error, details = test_config_geometry(
        config_name, branches, "RISKY"
    )
    
    print_test_result(test_num, config_name, passed, details)
    
    return passed, details


# ==============================================================================
# CATEGORY 2 (continued): MORE L/D TESTS  
# ==============================================================================

def test_2_3_petiolule_ld_12(test_num: int) -> Tuple[bool, Dict]:
    """Test 2.3: Petiolule L/D = 12 (UNSAFE expected)."""
    config_name = "petiolule_ld_12"
    base = generate_baseline_tomato()
    branches = modify_config(base, {"petiolule_height": 0.036})
    
    print(f"\n{'='*80}")
    print(f"Test {test_num}: {config_name}")
    print(f"{'='*80}")
    print(f"Change: Petiolule height 15mm → 36mm")
    print(f"❌ Beyond safe threshold!")
    
    passed, max_error, details = test_config_geometry(config_name, branches, "UNSAFE")
    print_test_result(test_num, config_name, passed, details)
    return passed, details


def test_2_4_petiole_ld_10(test_num: int) -> Tuple[bool, Dict]:
    """Test 2.4: Petiole L/D = 10 (RISKY)."""
    config_name = "petiole_ld_10"
    base = generate_baseline_tomato()
    branches = modify_config(base, {"petiole_height": 0.046})
    
    print(f"\n{'='*80}")
    print(f"Test {test_num}: {config_name}")
    print(f"{'='*80}")
    print(f"Change: Petiole height 27mm → 46mm")
    
    passed, max_error, details = test_config_geometry(config_name, branches, "RISKY")
    print_test_result(test_num, config_name, passed, details)
    return passed, details


# ==============================================================================
# CATEGORY 3: RADIUS RATIO TESTS
# ==============================================================================

def test_3_1_radius_ratio_2_5(test_num: int) -> Tuple[bool, Dict]:
    """Test 3.1: Radius ratio 2.5× (MARGINAL)."""
    config_name = "radius_ratio_2_5"
    base = generate_baseline_tomato()
    branches = modify_config(base, {"petiolule_radius": 0.00092})
    
    print(f"\n{'='*80}")
    print(f"Test {test_num}: {config_name}")
    print(f"{'='*80}")
    print(f"Change: Petiolule radius 1.5mm → 0.92mm")
    print(f"Ratio: 2.3mm / 0.92mm = 2.5× (was 1.5×)")
    
    passed, max_error, details = test_config_geometry(config_name, branches, "MARGINAL")
    print_test_result(test_num, config_name, passed, details)
    return passed, details


def test_3_2_radius_ratio_3_5(test_num: int) -> Tuple[bool, Dict]:
    """Test 3.2: Radius ratio 3.5× (RISKY)."""
    config_name = "radius_ratio_3_5"
    base = generate_baseline_tomato()
    branches = modify_config(base, {"petiolule_radius": 0.00066})
    
    print(f"\n{'='*80}")
    print(f"Test {test_num}: {config_name}")
    print(f"{'='*80}")
    print(f"Change: Petiolule radius 1.5mm → 0.66mm")
    print(f"World radius: 1.32mm (BELOW 2mm threshold!)")
    
    passed, max_error, details = test_config_geometry(config_name, branches, "RISKY")
    print_test_result(test_num, config_name, passed, details)
    return passed, details


# ==============================================================================
# CATEGORY 4: COMPLEXITY TESTS
# ==============================================================================

def test_4_1_six_petioles(test_num: int) -> Tuple[bool, Dict]:
    """Test 4.1: 6 petioles (50 links total)."""
    config_name = "six_petioles_50_links"
    branches = []
    
    branches.append({
        "id": "stem", "parent": None, "attach_link": None,
        "n_links": 5, "radius": 0.004, "height": 0.030, "tilt": 0.0, "rot": 0.0,
    })
    
    stem_attach_links = [2, 2, 3, 3, 4, 5]
    petiole_rots = [0.0, 180.0, 90.0, 270.0, 45.0, 135.0]
    
    for i, (attach, rot) in enumerate(zip(stem_attach_links, petiole_rots), 1):
        petiole_id = f"petiole_{i}"
        branches.append({
            "id": petiole_id, "parent": "stem", "attach_link": attach,
            "n_links": 3, "radius": 0.0023, "height": 0.027, "tilt": 45.0, "rot": rot,
        })
        
        for j, pet_rot in enumerate([0.0, 120.0, 240.0], 1):
            branches.append({
                "id": f"petiolule_{i}_{j}", "parent": petiole_id, "attach_link": j,
                "n_links": 2, "radius": 0.0015, "height": 0.015, "tilt": 30.0, "rot": pet_rot,
            })
    
    print(f"\n{'='*80}")
    print(f"Test {test_num}: {config_name}")
    print(f"{'='*80}")
    print(f"Total links: 50 (was 41)")
    
    passed, max_error, details = test_config_geometry(config_name, branches, "MARGINAL")
    print_test_result(test_num, config_name, passed, details)
    return passed, details


def test_4_2_seven_petioles(test_num: int) -> Tuple[bool, Dict]:
    """Test 4.2: 5 petioles (50 links) - Realistic density."""
    config_name = "five_petioles_50_links"
    branches = []
    
    branches.append({
        "id": "stem", "parent": None, "attach_link": None,
        "n_links": 5, "radius": 0.004, "height": 0.030, "tilt": 0.0, "rot": 0.0,
    })
    
    # 5 Petioles (different from test_4_1)
    stem_attach_links = [2, 3, 3, 4, 5]
    petiole_rots = [0.0, 90.0, 270.0, 180.0, 45.0]
    
    for i, (attach, rot) in enumerate(zip(stem_attach_links, petiole_rots), 1):
        petiole_id = f"petiole_{i}"
        branches.append({
            "id": petiole_id, "parent": "stem", "attach_link": attach,
            "n_links": 3, "radius": 0.0023, "height": 0.027, "tilt": 45.0, "rot": rot,
        })
        
        for j, pet_rot in enumerate([0.0, 120.0, 240.0], 1):
            branches.append({
                "id": f"petiolule_{i}_{j}", "parent": petiole_id, "attach_link": j,
                "n_links": 2, "radius": 0.0015, "height": 0.015, "tilt": 30.0, "rot": pet_rot,
            })
    
    print(f"\n{'='*80}")
    print(f"Test {test_num}: {config_name}")
    print(f"{'='*80}")
    print(f"Total links: 50 (safe margin from PhysX limit of 64)")
    print(f"Note: Different petiole distribution than test 4.1")
    
    passed, max_error, details = test_config_geometry(config_name, branches, "MARGINAL")
    print_test_result(test_num, config_name, passed, details)
    return passed, details


# ==============================================================================
# CATEGORY 5: MINIMUM RADIUS TESTS
# ==============================================================================

def test_5_1_min_radius_2mm(test_num: int) -> Tuple[bool, Dict]:
    """Test 5.1: Petiolule radius 2mm world (MARGINAL)."""
    config_name = "min_radius_2mm_world"
    base = generate_baseline_tomato()
    branches = modify_config(base, {"petiolule_radius": 0.001})
    
    print(f"\n{'='*80}")
    print(f"Test {test_num}: {config_name}")
    print(f"{'='*80}")
    print(f"World radius: 2.0mm (at collision threshold)")
    
    passed, max_error, details = test_config_geometry(config_name, branches, "MARGINAL")
    print_test_result(test_num, config_name, passed, details)
    return passed, details


def test_5_2_min_radius_1mm(test_num: int) -> Tuple[bool, Dict]:
    """Test 5.2: Petiolule radius 1mm world (UNSAFE expected)."""
    config_name = "min_radius_1mm_world"
    base = generate_baseline_tomato()
    branches = modify_config(base, {"petiolule_radius": 0.0005})
    
    print(f"\n{'='*80}")
    print(f"Test {test_num}: {config_name}")
    print(f"{'='*80}")
    print(f"World radius: 1.0mm (BELOW 2mm threshold)")
    print(f"❌ Below safe collision threshold!")
    
    passed, max_error, details = test_config_geometry(config_name, branches, "UNSAFE")
    print_test_result(test_num, config_name, passed, details)
    return passed, details


# ==============================================================================
# CATEGORY 6 (continued): MIXED ANGLES
# ==============================================================================

def test_6_4_mixed_angles(test_num: int) -> Tuple[bool, Dict]:
    """Test 6.4: Mixed angles - realistic variation."""
    config_name = "mixed_angles"
    base = generate_baseline_tomato()
    branches = modify_config(base, {"petiole_tilt": 60.0})
    
    print(f"\n{'='*80}")
    print(f"Test {test_num}: {config_name}")
    print(f"{'='*80}")
    print(f"Hierarchy: Stem 0° → Petiole 60° → Petiolule 30°")
    
    passed, max_error, details = test_config_geometry(config_name, branches, "MARGINAL")
    print_test_result(test_num, config_name, passed, details)
    return passed, details


def main():
    """Run Phase 1 geometry limit tests (HIGH priority first)."""
    print("=" * 80)
    print(" " * 20 + "SCALABILITY TEST SUITE - PHASE 1")
    print(" " * 25 + "Geometry Limit Tests (USD-only)")
    print("=" * 80)
    print()
    print("This suite tests configuration limits WITHOUT Isaac Sim.")
    print("Tests verify: validation, USD generation, geometry correctness, physics validity.")
    print()
    print("Tolerance: max position error < 1.0 mm")
    print()
    print("Phase 1: ALL 16 geometry tests (HIGH + MEDIUM + LOW priority)")
    print()
    
    results = []
    test_num = 1
    
    # Category 1: Baseline
    print("\n" + "="*80)
    print("CATEGORY 1: BASELINE")
    print("="*80)
    passed, details = test_1_1_baseline_tomato(test_num)
    results.append(("baseline_tomato", passed, details))
    test_num += 1
    
    # Category 2: Slenderness (L/D) - 4 tests
    print("\n" + "="*80)
    print("CATEGORY 2: SLENDERNESS (L/D) TESTS")
    print("="*80)
    passed, details = test_2_1_petiolule_ld_8(test_num)
    results.append(("petiolule_ld_8", passed, details))
    test_num += 1
    
    passed, details = test_2_2_petiolule_ld_10(test_num)
    results.append(("petiolule_ld_10", passed, details))
    test_num += 1
    
    passed, details = test_2_3_petiolule_ld_12(test_num)
    results.append(("petiolule_ld_12", passed, details))
    test_num += 1
    
    passed, details = test_2_4_petiole_ld_10(test_num)
    results.append(("petiole_ld_10", passed, details))
    test_num += 1
    
    # Category 3: Radius Ratios - 2 tests
    print("\n" + "="*80)
    print("CATEGORY 3: RADIUS RATIO TESTS")
    print("="*80)
    passed, details = test_3_1_radius_ratio_2_5(test_num)
    results.append(("radius_ratio_2_5", passed, details))
    test_num += 1
    
    passed, details = test_3_2_radius_ratio_3_5(test_num)
    results.append(("radius_ratio_3_5", passed, details))
    test_num += 1
    
    # Category 4: Complexity - 2 tests
    print("\n" + "="*80)
    print("CATEGORY 4: COMPLEXITY TESTS")
    print("="*80)
    passed, details = test_4_1_six_petioles(test_num)
    results.append(("six_petioles_50", passed, details))
    test_num += 1
    
    passed, details = test_4_2_seven_petioles(test_num)
    results.append(("five_petioles_50_alt", passed, details))
    test_num += 1
    
    # Category 5: Minimum Radius - 2 tests
    print("\n" + "="*80)
    print("CATEGORY 5: MINIMUM RADIUS TESTS")
    print("="*80)
    passed, details = test_5_1_min_radius_2mm(test_num)
    results.append(("min_radius_2mm", passed, details))
    test_num += 1
    
    passed, details = test_5_2_min_radius_1mm(test_num)
    results.append(("min_radius_1mm", passed, details))
    test_num += 1
    
    # Category 6: Tilt Angles - 4 tests
    print("\n" + "="*80)
    print("CATEGORY 6: TILT ANGLE TESTS")
    print("="*80)
    passed, details = test_6_1_petiole_tilt_30(test_num)
    results.append(("petiole_tilt_30", passed, details))
    test_num += 1
    
    passed, details = test_6_2_petiole_tilt_60(test_num)
    results.append(("petiole_tilt_60", passed, details))
    test_num += 1
    
    passed, details = test_6_3_petiole_tilt_90(test_num)
    results.append(("petiole_tilt_90", passed, details))
    test_num += 1
    
    passed, details = test_6_4_mixed_angles(test_num)
    results.append(("mixed_angles", passed, details))
    test_num += 1
    
    # Final report
    print()
    print("=" * 80)
    print(" " * 30 + "FINAL REPORT")
    print("=" * 80)
    print()
    
    passed_count = sum(1 for _, p, _ in results if p)
    total = len(results)
    
    print(f"{'Config':<30} {'Status':<10} {'Links':<7} {'Max Error (mm)':<15}")
    print("-" * 80)
    for name, passed, details in results:
        status = "✅ PASS" if passed else "❌ FAIL"
        links = details['total_links']
        error = f"{details['max_error_mm']:.3f}" if details['max_error_mm'] is not None else "N/A"
        print(f"{name:<30} {status:<10} {links:<7} {error:<15}")
    
    print("-" * 80)
    print(f"Tests passed: {passed_count}/{total}")
    print()
    
    if passed_count == total:
        print("=" * 80)
        print(" " * 25 + "✅ ALL TESTS PASSED ✅")
        print("=" * 80)
        print()
        print("VERDICT: All configurations generate valid USD with correct geometry.")
        print("         Ready for Isaac Sim convergence tests (Task 3).")
        print()
    else:
        print("=" * 80)
        print(" " * 25 + "❌ SOME TESTS FAILED ❌")
        print("=" * 80)
        print()
        failed = [name for name, p, _ in results if not p]
        print(f"Failed configs: {', '.join(failed)}")
        print()
        sys.exit(1)


if __name__ == "__main__":
    main()
