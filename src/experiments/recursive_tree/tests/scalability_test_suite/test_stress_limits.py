#!/usr/bin/env python3
"""
Stress Test Suite - Find Breaking Point

Progressive stress tests to find the limits of the current physics setup.
All tests use realistic tomato plant parameters but push complexity/geometry limits.

Categories:
1. Link Count Stress (approaching PhysX 64-link limit)
2. Depth Stress (deep branching hierarchies)
3. Density Stress (many branches from same attachment point)
4. Extreme Geometry (very thin, very tilted, very long)

Usage:
    uv run src/experiments/recursive_tree/tests/test_stress_limits.py
"""
import sys
import os
from pathlib import Path

SCRIPT_DIR = Path(__file__).parent.resolve()
RECURSIVE_TREE_DIR = SCRIPT_DIR.parent.parent
sys.path.insert(0, str(RECURSIVE_TREE_DIR))

from tree_config import validate_branches
from test_scalability import test_config_geometry


# ==============================================================================
# CATEGORY 1: LINK COUNT STRESS (approaching 64-link PhysX limit)
# ==============================================================================

def stress_1_max_links_59():
    """Stress 1: Maximum safe links (59) - just below 64 limit."""
    branches = []
    
    # Stem: 5 links
    branches.append({
        "id": "stem", "parent": None, "attach_link": None,
        "n_links": 5, "radius": 0.004, "height": 0.030, "tilt": 0.0, "rot": 0.0,
    })
    
    # 6 petioles × 3 links = 18 links
    # 6 petioles × 3 petiolules × 2 links = 36 links
    # Total: 5 + 18 + 36 = 59 links
    
    stem_attach = [2, 2, 3, 3, 4, 5]
    petiole_rots = [0, 180, 90, 270, 45, 135]
    
    for i, (attach, rot) in enumerate(zip(stem_attach, petiole_rots), 1):
        petiole_id = f"petiole_{i}"
        branches.append({
            "id": petiole_id, "parent": "stem", "attach_link": attach,
            "n_links": 3, "radius": 0.0023, "height": 0.027, "tilt": 45.0, "rot": rot,
        })
        
        for j, pet_rot in enumerate([0, 120, 240], 1):
            branches.append({
                "id": f"petiolule_{i}_{j}", "parent": petiole_id, "attach_link": j,
                "n_links": 2, "radius": 0.0015, "height": 0.015, "tilt": 30.0, "rot": pet_rot,
            })
    
    return "stress_max_links_59", branches, 59


def stress_2_extreme_links_63():
    """Stress 2: EXTREME - 63 links (1 below hard limit)."""
    branches = []
    
    # Stem: 7 links (longer stem)
    branches.append({
        "id": "stem", "parent": None, "attach_link": None,
        "n_links": 7, "radius": 0.004, "height": 0.030, "tilt": 0.0, "rot": 0.0,
    })
    
    # 7 petioles × 2 links = 14 links
    # 7 petioles × 3 petiolules × 2 links = 42 links
    # Total: 7 + 14 + 42 = 63 links
    
    stem_attach = [2, 3, 3, 4, 5, 6, 7]
    petiole_rots = [0, 90, 180, 270, 45, 135, 225]
    
    for i, (attach, rot) in enumerate(zip(stem_attach, petiole_rots), 1):
        petiole_id = f"petiole_{i}"
        branches.append({
            "id": petiole_id, "parent": "stem", "attach_link": attach,
            "n_links": 2, "radius": 0.0023, "height": 0.027, "tilt": 45.0, "rot": rot,
        })
        
        for j, pet_rot in enumerate([0, 120, 240], 1):
            branches.append({
                "id": f"petiolule_{i}_{j}", "parent": petiole_id, "attach_link": j if j <= 2 else 2,
                "n_links": 2, "radius": 0.0015, "height": 0.015, "tilt": 30.0, "rot": pet_rot,
            })
    
    return "stress_extreme_links_63", branches, 63


# ==============================================================================
# CATEGORY 2: DEPTH STRESS (deep hierarchies)
# ==============================================================================

def stress_3_depth_5_levels():
    """Stress 3: 5-level depth hierarchy (stem → petiole → petiolule → sub1 → sub2)."""
    branches = []
    
    # Level 1: Stem
    branches.append({
        "id": "stem", "parent": None, "attach_link": None,
        "n_links": 4, "radius": 0.004, "height": 0.030, "tilt": 0.0, "rot": 0.0,
    })
    
    # Level 2: 2 Petioles
    for i in [1, 2]:
        petiole_id = f"petiole_{i}"
        branches.append({
            "id": petiole_id, "parent": "stem", "attach_link": 2 + i - 1,
            "n_links": 3, "radius": 0.0023, "height": 0.027, "tilt": 45.0, "rot": i * 90.0,
        })
        
        # Level 3: 2 Petiolules per petiole
        for j in [1, 2]:
            petiolule_id = f"petiolule_{i}_{j}"
            branches.append({
                "id": petiolule_id, "parent": petiole_id, "attach_link": j + 1,
                "n_links": 2, "radius": 0.0015, "height": 0.015, "tilt": 30.0, "rot": j * 120.0,
            })
            
            # Level 4: Sub-branch 1
            sub1_id = f"sub1_{i}_{j}"
            branches.append({
                "id": sub1_id, "parent": petiolule_id, "attach_link": 2,
                "n_links": 2, "radius": 0.001, "height": 0.010, "tilt": 25.0, "rot": 90.0,
            })
            
            # Level 5: Sub-branch 2 (deepest level)
            sub2_id = f"sub2_{i}_{j}"
            branches.append({
                "id": sub2_id, "parent": sub1_id, "attach_link": 2,
                "n_links": 1, "radius": 0.0008, "height": 0.008, "tilt": 20.0, "rot": 0.0,
            })
    
    return "stress_depth_5_levels", branches, sum(b["n_links"] for b in branches)


# ==============================================================================
# CATEGORY 3: DENSITY STRESS (many branches from same point)
# ==============================================================================

def stress_4_dense_attachment():
    """Stress 4: 6 branches from single attachment point (density stress)."""
    branches = []
    
    # Stem
    branches.append({
        "id": "stem", "parent": None, "attach_link": None,
        "n_links": 5, "radius": 0.004, "height": 0.030, "tilt": 0.0, "rot": 0.0,
    })
    
    # Single petiole
    branches.append({
        "id": "main_petiole", "parent": "stem", "attach_link": 3,
        "n_links": 3, "radius": 0.0023, "height": 0.027, "tilt": 45.0, "rot": 0.0,
    })
    
    # 6 petiolules ALL from link 2 of the petiole (high density!)
    for i in range(1, 7):
        branches.append({
            "id": f"petiolule_{i}", "parent": "main_petiole", "attach_link": 2,
            "n_links": 2, "radius": 0.0015, "height": 0.015, "tilt": 30.0, "rot": i * 60.0,
        })
    
    return "stress_dense_6_from_1", branches, sum(b["n_links"] for b in branches)


def stress_5_radial_explosion():
    """Stress 5: 8 petioles from stem in radial pattern (45° spacing)."""
    branches = []
    
    # Stem
    branches.append({
        "id": "stem", "parent": None, "attach_link": None,
        "n_links": 5, "radius": 0.004, "height": 0.030, "tilt": 0.0, "rot": 0.0,
    })
    
    # 8 petioles in radial pattern (every 45°)
    for i in range(8):
        branches.append({
            "id": f"petiole_{i+1}", "parent": "stem", "attach_link": 2 + (i // 2),
            "n_links": 2, "radius": 0.0023, "height": 0.027, "tilt": 45.0, "rot": i * 45.0,
        })
    
    return "stress_radial_8_petioles", branches, sum(b["n_links"] for b in branches)


# ==============================================================================
# CATEGORY 4: EXTREME GEOMETRY
# ==============================================================================

def stress_6_ultra_thin():
    """Stress 6: Ultra-thin branches (radius 0.5mm world = 0.05mm pre-scale)."""
    branches = []
    
    # Stem (normal)
    branches.append({
        "id": "stem", "parent": None, "attach_link": None,
        "n_links": 5, "radius": 0.004, "height": 0.030, "tilt": 0.0, "rot": 0.0,
    })
    
    # 2 petioles (thin)
    for i in [1, 2]:
        branches.append({
            "id": f"petiole_{i}", "parent": "stem", "attach_link": 2 + i,
            "n_links": 3, "radius": 0.001, "height": 0.027, "tilt": 45.0, "rot": i * 180.0,
        })
        
        # 3 petiolules (ULTRA thin - 0.5mm world)
        for j in [1, 2, 3]:
            branches.append({
                "id": f"petiolule_{i}_{j}", "parent": f"petiole_{i}", "attach_link": j,
                "n_links": 2, "radius": 0.00005, "height": 0.015, "tilt": 30.0, "rot": j * 120.0,
            })
    
    return "stress_ultra_thin_0.5mm", branches, sum(b["n_links"] for b in branches)


def stress_7_extreme_slenderness():
    """Stress 7: Extreme slenderness ratio L/D = 15 (beyond safe zone)."""
    branches = []
    
    # Stem (normal)
    branches.append({
        "id": "stem", "parent": None, "attach_link": None,
        "n_links": 5, "radius": 0.004, "height": 0.030, "tilt": 0.0, "rot": 0.0,
    })
    
    # 2 petioles with extreme L/D
    for i in [1, 2]:
        # r=0.002, h=0.050, n=3 → per link L/D = 12.5
        branches.append({
            "id": f"petiole_{i}", "parent": "stem", "attach_link": 2 + i,
            "n_links": 3, "radius": 0.002, "height": 0.050, "tilt": 45.0, "rot": i * 180.0,
        })
    
    return "stress_extreme_slenderness_LD15", branches, sum(b["n_links"] for b in branches)


def stress_8_horizontal_cantilever():
    """Stress 8: Completely horizontal branches (90° tilt) - maximum droop."""
    branches = []
    
    # Stem
    branches.append({
        "id": "stem", "parent": None, "attach_link": None,
        "n_links": 5, "radius": 0.004, "height": 0.030, "tilt": 0.0, "rot": 0.0,
    })
    
    # 4 horizontal petioles (worst case for droop)
    for i in range(4):
        branches.append({
            "id": f"petiole_{i+1}", "parent": "stem", "attach_link": 2 + i,
            "n_links": 3, "radius": 0.0023, "height": 0.027, "tilt": 90.0, "rot": i * 90.0,
        })
        
        # 2 horizontal petiolules
        for j in [1, 2]:
            branches.append({
                "id": f"petiolule_{i+1}_{j}", "parent": f"petiole_{i+1}", "attach_link": j + 1,
                "n_links": 2, "radius": 0.0015, "height": 0.015, "tilt": 90.0, "rot": j * 180.0,
            })
    
    return "stress_horizontal_90deg", branches, sum(b["n_links"] for b in branches)


# ==============================================================================
# CATEGORY 5: BEYOND 64 LIMIT (experimental - will likely fail or need workarounds)
# ==============================================================================

def stress_9_monster_70_links():
    """Stress 9: MONSTER - 70 links (exceeds PhysX 64-link hard limit)."""
    branches = []
    
    # Stem: 10 links (long trunk)
    branches.append({
        "id": "stem", "parent": None, "attach_link": None,
        "n_links": 10, "radius": 0.004, "height": 0.030, "tilt": 0.0, "rot": 0.0,
    })
    
    # 10 petioles × 2 links = 20 links
    # 10 petioles × 2 petiolules × 2 links = 40 links
    # Total: 10 + 20 + 40 = 70 links
    
    for i in range(10):
        petiole_id = f"petiole_{i+1}"
        attach_link = 2 + (i % 9)  # Distribute along stem (links 2-10)
        
        branches.append({
            "id": petiole_id, "parent": "stem", "attach_link": attach_link,
            "n_links": 2, "radius": 0.0023, "height": 0.027, "tilt": 45.0, "rot": (i * 36.0) % 360,
        })
        
        # 2 petiolules per petiole
        for j in [1, 2]:
            branches.append({
                "id": f"petiolule_{i+1}_{j}", "parent": petiole_id, "attach_link": j,
                "n_links": 2, "radius": 0.0015, "height": 0.015, "tilt": 30.0, "rot": j * 180.0,
            })
    
    return "stress_monster_70_links", branches, 70


def stress_10_megamonster_85_links():
    """Stress 10: MEGA-MONSTER - 85 links (way beyond limit)."""
    branches = []
    
    # Stem: 5 links
    branches.append({
        "id": "stem", "parent": None, "attach_link": None,
        "n_links": 5, "radius": 0.004, "height": 0.030, "tilt": 0.0, "rot": 0.0,
    })
    
    # 10 main petioles × 3 links = 30 links
    # Each petiole has 5 petiolules × 2 links = 50 links
    # Total: 5 + 30 + 50 = 85 links
    
    for i in range(10):
        petiole_id = f"petiole_{i+1}"
        attach_link = 2 + (i // 3)  # Multiple petioles per stem link
        
        branches.append({
            "id": petiole_id, "parent": "stem", "attach_link": attach_link,
            "n_links": 3, "radius": 0.0023, "height": 0.027, "tilt": 45.0, "rot": (i * 36.0) % 360,
        })
        
        # 5 petiolules per petiole (dense!)
        for j in range(1, 6):
            pet_attach = 1 + (j // 2)  # Distribute along petiole
            branches.append({
                "id": f"petiolule_{i+1}_{j}", "parent": petiole_id, "attach_link": pet_attach,
                "n_links": 1, "radius": 0.0015, "height": 0.015, "tilt": 30.0, "rot": (j * 72.0) % 360,
            })
    
    return "stress_megamonster_85_links", branches, 85


def stress_11_ultramonster_100_links():
    """Stress 11: ULTRA-MONSTER - 100 links (extreme test)."""
    branches = []
    
    # Stem: 5 links
    branches.append({
        "id": "stem", "parent": None, "attach_link": None,
        "n_links": 5, "radius": 0.004, "height": 0.030, "tilt": 0.0, "rot": 0.0,
    })
    
    # Strategy: Many small branches to reach 100 links
    # 19 petioles × 3 links = 57 links
    # 19 petioles × 2 petiolules × 1 link = 38 links
    # Total: 5 + 57 + 38 = 100 links
    
    for i in range(19):
        petiole_id = f"petiole_{i+1}"
        attach_link = 2 + (i // 5)  # Pack multiple petioles per stem link
        
        branches.append({
            "id": petiole_id, "parent": "stem", "attach_link": attach_link,
            "n_links": 3, "radius": 0.0023, "height": 0.027, "tilt": 45.0, "rot": (i * 18.95) % 360,
        })
        
        # 2 petiolules per petiole
        for j in [1, 2]:
            branches.append({
                "id": f"petiolule_{i+1}_{j}", "parent": petiole_id, "attach_link": j + 1,
                "n_links": 1, "radius": 0.0015, "height": 0.015, "tilt": 30.0, "rot": j * 180.0,
            })
    
    return "stress_ultramonster_100_links", branches, 100


def stress_12_apocalypse_200_links():
    """Stress 12: APOCALYPSE - 200 links (DENSE hierarchy, multi-level branching)."""
    branches = []
    
    # Stem: 8 links (tall trunk for distributing branches)
    branches.append({
        "id": "stem", "parent": None, "attach_link": None,
        "n_links": 8, "radius": 0.004, "height": 0.030, "tilt": 0.0, "rot": 0.0,
    })
    
    total = 8
    
    # Strategy: Dense multi-level branching
    # 12 main petioles × 4 links = 48 links
    # Each petiole has 4 petiolules × 2 links = 96 links  
    # Each petiolule has 2 sub-branches × 1 link = 48 links
    # Total: 8 + 48 + 96 + 48 = 200 links
    
    for i in range(12):
        petiole_id = f"petiole_{i+1}"
        # Dense attachment: pack multiple petioles on each stem link
        attach_link = 2 + (i // 2)  # 2 petioles per stem link
        
        branches.append({
            "id": petiole_id, "parent": "stem", "attach_link": attach_link,
            "n_links": 4, "radius": 0.0023, "height": 0.027, "tilt": 45.0, "rot": (i * 30.0) % 360,
        })
        total += 4
        
        # Level 2: 4 petiolules per petiole (DENSE!)
        for j in range(1, 5):
            petiolule_id = f"petiolule_{i+1}_{j}"
            pet_attach = 1 + (j // 2)  # 2 petiolules per petiole link
            
            branches.append({
                "id": petiolule_id, "parent": petiole_id, "attach_link": pet_attach,
                "n_links": 2, "radius": 0.0015, "height": 0.015, "tilt": 30.0, "rot": (j * 90.0) % 360,
            })
            total += 2
            
            # Level 3: 2 sub-branches per petiolule (SUPER DENSE!)
            for k in [1, 2]:
                sub_id = f"sub_{i+1}_{j}_{k}"
                
                branches.append({
                    "id": sub_id, "parent": petiolule_id, "attach_link": k,
                    "n_links": 1, "radius": 0.001, "height": 0.010, "tilt": 25.0, "rot": k * 180.0,
                })
                total += 1
    
    return "stress_apocalypse_200_links", branches, total


def stress_13_extreme_150_links():
    """Stress 13: EXTREME - 150 links (complex but stable, thicker branches)."""
    branches = []
    
    # Stem: 6 links (good height for distribution)
    branches.append({
        "id": "stem", "parent": None, "attach_link": None,
        "n_links": 6, "radius": 0.004, "height": 0.030, "tilt": 0.0, "rot": 0.0,
    })
    
    total = 6
    
    # Strategy: Complex 3-level hierarchy with thicker branches
    # 10 main petioles × 3 links = 30 links
    # Each petiole has 4 petiolules × 3 links = 120 links
    # Total: 6 + 30 + 120 = 156 links (~150 target)
    
    for i in range(10):
        petiole_id = f"petiole_{i+1}"
        # Distribute petioles: 2 per stem link
        attach_link = 2 + (i // 2)
        
        branches.append({
            "id": petiole_id, "parent": "stem", "attach_link": attach_link,
            "n_links": 3, 
            "radius": 0.0025,  # Slightly thicker than before (2.5mm pre-scale)
            "height": 0.028, 
            "tilt": 45.0, 
            "rot": (i * 36.0) % 360,
        })
        total += 3
        
        # Level 2: 4 petiolules per petiole (dense but manageable)
        for j in range(1, 5):
            petiolule_id = f"petiolule_{i+1}_{j}"
            # Distribute: 2 on link 1, 2 on link 2
            pet_attach = 1 + ((j - 1) // 2)
            
            branches.append({
                "id": petiolule_id, "parent": petiole_id, "attach_link": pet_attach,
                "n_links": 3,  # 3 links each for complexity
                "radius": 0.0018,  # Thicker than apocalypse (1.8mm pre-scale)
                "height": 0.016, 
                "tilt": 35.0, 
                "rot": ((j - 1) * 90.0) % 360,
            })
            total += 3
    
    return "stress_extreme_150_links", branches, total


def stress_14_ragnarok_1000_links():
    """Stress 14: ⚡ RAGNAROK ⚡ - 265 links (Final limit test - between 254 and 276)."""
    branches = []
    
    # Stem: 6 links
    branches.append({
        "id": "stem", "parent": None, "attach_link": None,
        "n_links": 6, "radius": 0.0045, "height": 0.032, "tilt": 0.0, "rot": 0.0,
    })
    
    total = 6
    
    # Strategy: 3-level hierarchy, ~265 links (between working 254 and crashing 276)
    # Current: 9 main × 3 sec × 3 tert × 2 = 6 + 27 + 81 + 162 = 276 (crash)
    # Previous: 8 main × 4 sec × 2 tert × 2 = 6 + 24 + 96 + 128 = 254 (works)
    # Target: 9 main × 3 sec × 2.7 tert avg
    # Try: 9 main × 3 sec, first 2 have 3 tert, last 1 has 2 tert
    # = 6 + 27 + 81 + (9×3×(2×3+1×2)×2)/3 = complex
    # Simpler: 8 main × 4 sec × 2 tert, but add 5 more tertiaries strategically
    # = 254 + 10 = 264 ✓
    # Or: 9 main × 3 sec × 2.5 tert = 6 + 27 + 81 + 135 = 249 (too low)
    # Better: 9 main, 3 sec each, some with 2 tert, some with 3 tert
    # 5 mains with 3×3 tert = 5×3×3×2 = 90
    # 4 mains with 3×2 tert = 4×3×2×2 = 48
    # Total = 6 + 27 + 81 + 90 + 48 = 252 (too low)
    # Try: 8 main, 4 sec, alternate 2/3 tertiaries
    # First 4 mains: 4×4×2×2 = 64
    # Last 4 mains: 4×4×3×2 = 96
    # Total = 6 + 24 + 96 + 64 + 96 = 286 (too high)
    # Better: 8 main, 4 sec, mostly 2 tert with some 3
    # 6 mains × 4 sec × 2 tert = 6×4×2×2 = 96
    # 2 mains × 4 sec × 3 tert = 2×4×3×2 = 48
    # Total = 6 + 24 + 96 + 96 + 48 = 270 ✓
    
    n_main = 8
    n_secondary_per_main = 4
    
    for i in range(n_main):
        # Level 1: Main petioles
        main_id = f"main_{i+1}"
        attach_link = 2 + (i // 2)
        
        branches.append({
            "id": main_id, "parent": "stem", "attach_link": attach_link,
            "n_links": 3, 
            "radius": 0.0032,
            "height": 0.030, 
            "tilt": 45.0, 
            "rot": (i * 45.0) % 360,
        })
        total += 3
        
        # Level 2: Secondary branches (4 per main)
        for j in range(n_secondary_per_main):
            sec_id = f"sec_{i+1}_{j+1}"
            sec_attach = 1 + (j // 2)
            
            branches.append({
                "id": sec_id, "parent": main_id, "attach_link": sec_attach,
                "n_links": 3,
                "radius": 0.0028,
                "height": 0.026,
                "tilt": 42.0,
                "rot": (j * 90.0) % 360,
            })
            total += 3
            
            # Level 3: Tertiary - varies by main index
            # First 6 mains: 2 tertiaries per secondary
            # Last 2 mains: 3 tertiaries per secondary
            n_tert = 3 if i >= 6 else 2
            
            for k in range(n_tert):
                tert_id = f"tert_{i+1}_{j+1}_{k+1}"
                tert_attach = 1 + (k // 2)
                
                branches.append({
                    "id": tert_id, "parent": sec_id, "attach_link": tert_attach,
                    "n_links": 2,
                    "radius": 0.0026,
                    "height": 0.020,
                    "tilt": 38.0,
                    "rot": (k * 120.0) % 360,
                })
                total += 2
    
    return "stress_ragnarok_final", branches, total


# ==============================================================================
# MAIN TEST RUNNER
# ==============================================================================

def main():
    print("=" * 80)
    print(" " * 25 + "STRESS TEST SUITE")
    print(" " * 20 + "Find the Breaking Point")
    print("=" * 80)
    print()
    print("Testing configurations that push the limits of:")
    print("  1. Link count (approaching 64 PhysX limit)")
    print("  2. Hierarchy depth (5 levels)")
    print("  3. Attachment density (many branches from one point)")
    print("  4. Extreme geometry (thin, slender, horizontal)")
    print("  5. BEYOND 64 LIMIT (70, 85, 100 links - EXPERIMENTAL!)")
    print()
    print("Goal: Find where the system breaks (instability/jitter/collapse)")
    print()
    print("⚠️  WARNING: Tests 9-11 EXCEED PhysX 64-link hard limit!")
    print("   These will likely fail or require articulation splitting.")
    print()
    
    tests = [
        # Category 1: Link Count
        ("LINK COUNT", [
            stress_1_max_links_59,
            stress_2_extreme_links_63,
        ]),
        
        # Category 2: Depth
        ("HIERARCHY DEPTH", [
            stress_3_depth_5_levels,
        ]),
        
        # Category 3: Density
        ("ATTACHMENT DENSITY", [
            stress_4_dense_attachment,
            stress_5_radial_explosion,
        ]),
        
        # Category 4: Geometry
        ("EXTREME GEOMETRY", [
            stress_6_ultra_thin,
            stress_7_extreme_slenderness,
            stress_8_horizontal_cantilever,
        ]),
        
        # Category 5: Beyond 64 Limit (EXPERIMENTAL!)
        ("BEYOND 64 LIMIT (experimental)", [
            stress_9_monster_70_links,
            stress_10_megamonster_85_links,
            stress_11_ultramonster_100_links,
            stress_13_extreme_150_links,
            stress_12_apocalypse_200_links,
            stress_14_ragnarok_1000_links,
        ]),
    ]
    
    all_results = []
    test_num = 1
    
    for category_name, category_tests in tests:
        print()
        print("=" * 80)
        print(f"CATEGORY: {category_name}")
        print("=" * 80)
        
        # Check if this is the experimental "beyond 64 limit" category
        skip_limit = "BEYOND 64" in category_name
        
        for test_func in category_tests:
            config_name, branches, total_links = test_func()
            
            print(f"\nTest {test_num}: {config_name} ({total_links} links)")
            
            # Validate (skip 64-link check for experimental tests)
            try:
                validate_branches(branches, skip_limit_check=skip_limit)
            except ValueError as e:
                print(f"  ❌ Validation failed: {e}")
                all_results.append((config_name, False, None))
                test_num += 1
                continue
            
            # Generate USD
            passed, max_error, details = test_config_geometry(
                config_name, branches, "STRESS_TEST", save_usd=True, skip_limit_check=skip_limit
            )
            
            if passed:
                print(f"  ✅ USD generated: {config_name}.usda")
            else:
                print(f"  ❌ Generation failed")
            
            all_results.append((config_name, passed, details))
            test_num += 1
    
    # Final report
    print()
    print("=" * 80)
    print(" " * 30 + "FINAL REPORT")
    print("=" * 80)
    print()
    
    passed_count = sum(1 for _, p, _ in all_results if p)
    total = len(all_results)
    
    print(f"{'Config':<35} {'Status':<10} {'Links':<7}")
    print("-" * 80)
    for name, passed, details in all_results:
        status = "✅ PASS" if passed else "❌ FAIL"
        links = details['total_links'] if details else "N/A"
        print(f"{name:<35} {status:<10} {links:<7}")
    
    print("-" * 80)
    print(f"Tests passed: {passed_count}/{total}")
    print()
    
    if passed_count == total:
        print("=" * 80)
        print(" " * 20 + "✅ ALL STRESS TESTS GENERATED ✅")
        print("=" * 80)
        print()
        print("Now test each configuration in Isaac Sim to find breaking point.")
        print()
        print("Expected outcomes:")
        print("  - Link count tests: Should be stable until near 64 limit")
        print("  - Depth tests: May show slower performance but should be stable")
        print("  - Density tests: High collision workload, may be unstable")
        print("  - Geometry tests: Likely to show instability (droop, jitter)")
        print()
    else:
        print("Some tests failed validation or generation.")
    
    return 0 if passed_count == total else 1


if __name__ == "__main__":
    sys.exit(main())
