#!/usr/bin/env python3
"""
Incremental Debug Test - Find when jitter starts

Test progression:
1. Stem only (5 links)
2. Stem + 1 petiole (8 links)
3. Stem + 1 petiole + 1 petiolule (10 links)
4. Stem + 1 petiole + 3 petiolules (14 links)
5. Stem + 2 petioles + petiolules (23 links)
6. Stem + 4 petioles + petiolules (41 links)

Goal: Identify at which complexity level the jitter begins.
"""
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).parent.resolve()
RECURSIVE_TREE_DIR = SCRIPT_DIR.parent
sys.path.insert(0, str(RECURSIVE_TREE_DIR))
sys.path.insert(0, str(SCRIPT_DIR))

from tree_config import validate_branches
from test_scalability import test_config_geometry

def test_1_stem_only():
    """Test 1: Stem only - 5 links (simplest possible)."""
    branches = [{
        "id": "stem",
        "parent": None,
        "attach_link": None,
        "n_links": 5,
        "radius": 0.004,
        "height": 0.030,
        "tilt": 0.0,
        "rot": 0.0,
    }]
    return "test1_stem_only", branches, 5

def test_2_stem_1_petiole():
    """Test 2: Stem + 1 petiole - 8 links."""
    branches = [
        {
            "id": "stem",
            "parent": None,
            "attach_link": None,
            "n_links": 5,
            "radius": 0.004,
            "height": 0.030,
            "tilt": 0.0,
            "rot": 0.0,
        },
        {
            "id": "petiole_1",
            "parent": "stem",
            "attach_link": 3,
            "n_links": 3,
            "radius": 0.0023,
            "height": 0.027,
            "tilt": 45.0,
            "rot": 0.0,
        },
    ]
    return "test2_stem_1petiole", branches, 8

def test_3_stem_1petiole_1petiolule():
    """Test 3: Stem + 1 petiole + 1 petiolule - 10 links."""
    branches = [
        {
            "id": "stem",
            "parent": None,
            "attach_link": None,
            "n_links": 5,
            "radius": 0.004,
            "height": 0.030,
            "tilt": 0.0,
            "rot": 0.0,
        },
        {
            "id": "petiole_1",
            "parent": "stem",
            "attach_link": 3,
            "n_links": 3,
            "radius": 0.0023,
            "height": 0.027,
            "tilt": 45.0,
            "rot": 0.0,
        },
        {
            "id": "petiolule_1_1",
            "parent": "petiole_1",
            "attach_link": 2,
            "n_links": 2,
            "radius": 0.0015,
            "height": 0.015,
            "tilt": 30.0,
            "rot": 0.0,
        },
    ]
    return "test3_stem_1p_1pet", branches, 10

def test_4_stem_1petiole_3petiolules():
    """Test 4: Stem + 1 petiole + 3 petiolules - 14 links."""
    branches = [
        {
            "id": "stem",
            "parent": None,
            "attach_link": None,
            "n_links": 5,
            "radius": 0.004,
            "height": 0.030,
            "tilt": 0.0,
            "rot": 0.0,
        },
        {
            "id": "petiole_1",
            "parent": "stem",
            "attach_link": 3,
            "n_links": 3,
            "radius": 0.0023,
            "height": 0.027,
            "tilt": 45.0,
            "rot": 0.0,
        },
    ]

    # 3 petiolules
    for j, rot in enumerate([0.0, 120.0, 240.0], 1):
        branches.append({
            "id": f"petiolule_1_{j}",
            "parent": "petiole_1",
            "attach_link": j,
            "n_links": 2,
            "radius": 0.0015,
            "height": 0.015,
            "tilt": 30.0,
            "rot": rot,
        })

    return "test4_stem_1p_3pet", branches, 14

def test_5_stem_2petioles():
    """Test 5: Stem + 2 petioles + petiolules - 23 links."""
    branches = [
        {
            "id": "stem",
            "parent": None,
            "attach_link": None,
            "n_links": 5,
            "radius": 0.004,
            "height": 0.030,
            "tilt": 0.0,
            "rot": 0.0,
        },
    ]

    # 2 petioles
    for i, (attach, rot) in enumerate([(2, 0.0), (3, 180.0)], 1):
        petiole_id = f"petiole_{i}"
        branches.append({
            "id": petiole_id,
            "parent": "stem",
            "attach_link": attach,
            "n_links": 3,
            "radius": 0.0023,
            "height": 0.027,
            "tilt": 45.0,
            "rot": rot,
        })

        # 3 petiolules per petiole
        for j, pet_rot in enumerate([0.0, 120.0, 240.0], 1):
            branches.append({
                "id": f"petiolule_{i}_{j}",
                "parent": petiole_id,
                "attach_link": j,
                "n_links": 2,
                "radius": 0.0015,
                "height": 0.015,
                "tilt": 30.0,
                "rot": pet_rot,
            })

    return "test5_stem_2p", branches, 23

def test_6_stem_4petioles():
    """Test 6: Stem + 4 petioles + petiolules - 41 links (full baseline)."""
    branches = [
        {
            "id": "stem",
            "parent": None,
            "attach_link": None,
            "n_links": 5,
            "radius": 0.004,
            "height": 0.030,
            "tilt": 0.0,
            "rot": 0.0,
        },
    ]

    # 4 petioles
    for i, (attach, rot) in enumerate([(2, 0.0), (3, 90.0), (4, 180.0), (5, 270.0)], 1):
        petiole_id = f"petiole_{i}"
        branches.append({
            "id": petiole_id,
            "parent": "stem",
            "attach_link": attach,
            "n_links": 3,
            "radius": 0.0023,
            "height": 0.027,
            "tilt": 45.0,
            "rot": rot,
        })

        # 3 petiolules per petiole
        for j, pet_rot in enumerate([0.0, 120.0, 240.0], 1):
            branches.append({
                "id": f"petiolule_{i}_{j}",
                "parent": petiole_id,
                "attach_link": j,
                "n_links": 2,
                "radius": 0.0015,
                "height": 0.015,
                "tilt": 30.0,
                "rot": pet_rot,
            })

    return "test6_stem_4p_full", branches, 41

def main():
    print("="*80)
    print("INCREMENTAL DEBUG TEST - Find when jitter starts")
    print("="*80)
    print()
    print("Generating 6 test configs with increasing complexity...")
    print()

    tests = [
        test_1_stem_only,
        test_2_stem_1_petiole,
        test_3_stem_1petiole_1petiolule,
        test_4_stem_1petiole_3petiolules,
        test_5_stem_2petioles,
        test_6_stem_4petioles,
    ]

    for i, test_func in enumerate(tests, 1):
        name, branches, expected_links = test_func()

        print(f"[{i}/6] {name} ({expected_links} links)")

        # Validate
        try:
            validate_branches(branches)
        except ValueError as e:
            print(f"  ❌ Validation failed: {e}")
            continue

        # Generate USD
        passed, max_error, details = test_config_geometry(
            name, branches, "DEBUG", save_usd=True
        )

        if passed:
            print(f"  ✅ Generated: {name}.usda")
        else:
            print(f"  ❌ Generation failed")

        print()

    print("="*80)
    print("TESTING INSTRUCTIONS")
    print("="*80)
    print()
    print("Run manual test:")
    print("  python3 src/experiments/recursive_tree/tests/test_manual_cli.py")
    print()
    print("Test each config in order:")
    print("  1. test1_stem_only      (5 links)  - Should be stable")
    print("  2. test2_stem_1petiole  (8 links)  - First branch added")
    print("  3. test3_stem_1p_1pet   (10 links) - Depth-3 added")
    print("  4. test4_stem_1p_3pet   (14 links) - Full petiole")
    print("  5. test5_stem_2p        (23 links) - 2 petioles")
    print("  6. test6_stem_4p_full   (41 links) - Full plant")
    print()
    print("Goal: Find at which step jitter starts")
    print()
    print("Expected patterns:")
    print("  - If test1 jitters → problem is basic PhysX settings")
    print("  - If test2-3 jitter → problem is branching/joints")
    print("  - If test4-5 jitter → problem is complexity/multiple branches")
    print("  - If only test6 jitters → problem is total link count")
    print()

if __name__ == "__main__":
    main()
