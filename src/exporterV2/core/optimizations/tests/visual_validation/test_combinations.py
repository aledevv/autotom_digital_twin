"""
test_combinations.py - Non-visual tests for technique combinations

Tests joint counts, structural integrity, and attach_frac correctness for
all 8 technique combinations.
"""

import sys
from pathlib import Path

# Path setup
script_dir = Path(__file__).parent.resolve()
sys.path.insert(0, str(script_dir / "../../../../../../src"))
sys.path.insert(0, str(script_dir / "../.."))

from generate_combinations_usd import create_baseline_plant, apply_combination, COMBINATIONS
from techniques.base import count_d6_joints


def test_baseline_plant():
    """Verify baseline plant structure."""
    baseline = create_baseline_plant()
    joints = count_d6_joints(baseline)
    
    assert len(baseline) == 46, f"Expected 46 branches, got {len(baseline)}"
    assert joints == 99, f"Expected 99 D6 joints, got {joints}"
    
    # Check structure
    trunk = [b for b in baseline if b["id"] == "trunk"]
    assert len(trunk) == 1, "Expected 1 trunk"
    assert trunk[0]["n_links"] == 10, "Trunk should have 10 links"
    
    laterals = [b for b in baseline if "Branch_r" in b["id"]]
    assert len(laterals) == 5, f"Expected 5 lateral branches, got {len(laterals)}"
    
    petioles = [b for b in baseline if "_petiole" in b["id"]]
    assert len(petioles) == 8, f"Expected 8 petioles, got {len(petioles)}"
    
    rachis = [b for b in baseline if "_rachis" in b["id"]]
    assert len(rachis) == 8, f"Expected 8 rachis, got {len(rachis)}"
    
    petiolules = [b for b in baseline if "Petiolule_" in b["id"]]
    assert len(petiolules) == 24, f"Expected 24 petiolules, got {len(petiolules)}"


def test_combo_0_baseline():
    """Combo 0: Baseline (no techniques)."""
    baseline = create_baseline_plant()
    modified = apply_combination(baseline, [])
    
    joints = count_d6_joints(modified)
    assert joints == 99, f"Baseline should have 99 D6 joints, got {joints}"
    assert len(modified) == 46, f"Baseline should have 46 branches, got {len(modified)}"


def test_combo_1_petiole_lock():
    """Combo 1: Petiole Lock only."""
    baseline = create_baseline_plant()
    modified = apply_combination(baseline, ["petiole_lock"])
    
    joints = count_d6_joints(modified)
    assert joints == 75, f"Expected 75 D6 joints (99-24 petiolules), got {joints}"
    
    # Check petiolules are Fixed
    petiolules = [b for b in modified if "Petiolule_" in b["id"]]
    assert len(petiolules) == 24, "Should still have 24 petiolules"
    for p in petiolules:
        assert p.get("joint_type") == "fixed", f"{p['id']} should be Fixed"
    
    # Check structure unchanged
    assert len(modified) == 46, "Branch count should not change"


def test_combo_2_lateral_reduce():
    """Combo 2: Lateral Reduce only."""
    baseline = create_baseline_plant()
    modified = apply_combination(baseline, ["lateral_reduce"])
    
    joints = count_d6_joints(modified)
    assert joints == 79, f"Expected 79 D6 joints (99-20 lateral), got {joints}"
    
    # Check laterals reduced to 1 segment
    laterals = [b for b in modified if "Branch_r" in b["id"]]
    assert len(laterals) == 5, "Should still have 5 laterals"
    for lat in laterals:
        assert lat["n_links"] == 1, f"{lat['id']} should have 1 link, got {lat['n_links']}"


def test_combo_3_stem_collapse():
    """Combo 3: Stem Collapse only."""
    baseline = create_baseline_plant()
    modified = apply_combination(baseline, ["stem_collapse"])
    
    joints = count_d6_joints(modified)
    assert joints == 92, f"Expected 92 D6 joints (99-7 trunk), got {joints}"
    
    # Check trunk reduced to 3 segments
    trunk_before = [b for b in baseline if b["id"] == "trunk"][0]
    trunk_after = [b for b in modified if b["id"] == "trunk"][0]
    
    assert trunk_after["n_links"] == 3, f"Trunk should have 3 links, got {trunk_after['n_links']}"
    
    # Check total height preserved
    total_height_before = trunk_before["n_links"] * trunk_before["height"]
    total_height_after = trunk_after["n_links"] * trunk_after["height"]
    assert abs(total_height_before - total_height_after) < 0.01, (
        f"Trunk total height changed: {total_height_before:.3f}m → {total_height_after:.3f}m"
    )
    
    # Check children have attach_frac
    trunk_children = [b for b in modified if b.get("parent") == "trunk"]
    for child in trunk_children:
        assert "attach_frac" in child, f"{child['id']} missing attach_frac"
        assert 0 <= child["attach_frac"] <= 1, f"{child['id']} invalid attach_frac: {child['attach_frac']}"


def test_combo_4_leaf_reduce():
    """Combo 4: Leaf Reduce only."""
    baseline = create_baseline_plant()
    modified = apply_combination(baseline, ["leaf_reduce"])
    
    joints = count_d6_joints(modified)
    assert joints == 67, f"Expected 67 D6 joints (99-32 rachis), got {joints}"
    
    # Check petiole+rachis merged
    merged = [b for b in modified if "_merged" in b["id"]]
    assert len(merged) == 8, f"Expected 8 merged leaves, got {len(merged)}"
    
    # Check no separate rachis
    rachis = [b for b in modified if "_rachis" in b["id"]]
    assert len(rachis) == 0, "Should have no separate rachis branches"
    
    # Check petiolules attached to merged
    petiolules = [b for b in modified if "Petiolule_" in b["id"]]
    for p in petiolules:
        assert "_merged" in p["parent"], f"{p['id']} should be attached to merged leaf"
        assert "attach_frac" in p, f"{p['id']} missing attach_frac"


def test_combo_5_p_plus_l():
    """Combo 5: Petiole Lock + Lateral Reduce."""
    baseline = create_baseline_plant()
    modified = apply_combination(baseline, ["petiole_lock", "lateral_reduce"])
    
    joints = count_d6_joints(modified)
    assert joints == 55, f"Expected 55 D6 joints (99-24-20), got {joints}"
    
    # Check petiolules Fixed
    petiolules = [b for b in modified if "Petiolule_" in b["id"]]
    for p in petiolules:
        assert p.get("joint_type") == "fixed", f"{p['id']} should be Fixed"
    
    # Check laterals reduced
    laterals = [b for b in modified if "Branch_r" in b["id"]]
    for lat in laterals:
        assert lat["n_links"] == 1, f"{lat['id']} should have 1 link"


def test_combo_6_p_plus_f():
    """Combo 6: Petiole Lock + Leaf Reduce."""
    baseline = create_baseline_plant()
    modified = apply_combination(baseline, ["petiole_lock", "leaf_reduce"])
    
    joints = count_d6_joints(modified)
    assert joints == 43, f"Expected 43 D6 joints (99-24-32), got {joints}"
    
    # Check petiolules Fixed
    petiolules = [b for b in modified if "Petiolule_" in b["id"]]
    for p in petiolules:
        assert p.get("joint_type") == "fixed", f"{p['id']} should be Fixed"
    
    # Check leaves merged
    merged = [b for b in modified if "_merged" in b["id"]]
    assert len(merged) == 8, "Should have 8 merged leaves"


def test_combo_7_full():
    """Combo 7: Full Optimization (all techniques)."""
    baseline = create_baseline_plant()
    modified = apply_combination(baseline, [
        "petiole_lock",
        "lateral_reduce",
        "stem_collapse",
        "leaf_reduce",
    ])
    
    joints = count_d6_joints(modified)
    assert joints == 16, f"Expected 16 D6 joints (highly optimized), got {joints}"
    
    # Check all techniques applied
    
    # Petiolules Fixed
    petiolules = [b for b in modified if "Petiolule_" in b["id"]]
    for p in petiolules:
        assert p.get("joint_type") == "fixed", f"{p['id']} should be Fixed"
    
    # Laterals reduced
    laterals = [b for b in modified if "Branch_r" in b["id"]]
    for lat in laterals:
        assert lat["n_links"] == 1, f"{lat['id']} should have 1 link"
    
    # Trunk reduced
    trunk = [b for b in modified if b["id"] == "trunk"]
    assert trunk[0]["n_links"] == 3, "Trunk should have 3 links"
    
    # Leaves merged
    merged = [b for b in modified if "_merged" in b["id"]]
    assert len(merged) == 8, "Should have 8 merged leaves"


def test_no_orphaned_branches():
    """Verify no technique creates orphaned branches."""
    baseline = create_baseline_plant()
    
    for combo in COMBINATIONS:
        if combo["id"] == 0:
            continue  # Skip baseline
        
        modified = apply_combination(baseline, combo["techniques"])
        
        # Build parent map
        branch_ids = {b["id"] for b in modified}
        
        for branch in modified:
            parent_id = branch.get("parent")
            if parent_id is not None:  # trunk has no parent
                assert parent_id in branch_ids, (
                    f"Combo {combo['id']} ({combo['label']}): "
                    f"Branch {branch['id']} has orphaned parent {parent_id}"
                )


def test_attach_frac_values():
    """Verify attach_frac is valid where present."""
    baseline = create_baseline_plant()
    
    for combo in COMBINATIONS:
        modified = apply_combination(baseline, combo["techniques"])
        
        for branch in modified:
            if "attach_frac" in branch:
                frac = branch["attach_frac"]
                assert 0 <= frac <= 1, (
                    f"Combo {combo['id']}: {branch['id']} has invalid "
                    f"attach_frac: {frac} (should be 0-1)"
                )


def test_joint_count_never_increases():
    """Verify techniques never increase joint count."""
    baseline = create_baseline_plant()
    baseline_joints = count_d6_joints(baseline)
    
    for combo in COMBINATIONS:
        modified = apply_combination(baseline, combo["techniques"])
        joints = count_d6_joints(modified)
        
        assert joints <= baseline_joints, (
            f"Combo {combo['id']} ({combo['label']}) increased joints: "
            f"{baseline_joints} → {joints}"
        )
