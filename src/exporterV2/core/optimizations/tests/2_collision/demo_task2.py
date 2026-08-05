"""
demo_task2.py - Demo Script for Task 2 (Collision Detection)

Demonstrates:
- Bounding sphere calculation
- AABB calculation  
- Two-stage broad-phase collision detection
- Collision resolution scenarios

Run with: uv run python src/exporterV2/core/optimizations/tests/demo_task2.py
"""

import sys
import os

# Add optimizations directory to path
script_dir = os.path.dirname(os.path.abspath(__file__))
optimizations_dir = os.path.join(script_dir, "../..")
sys.path.insert(0, optimizations_dir)

from collision import (
    Vec3, CylinderGeometry,
    calculate_bounding_sphere, check_sphere_overlap,
    calculate_aabb, check_aabb_overlap,
    check_attachment_collision, check_pairwise_collisions
)


def main():
    print("=" * 70)
    print("  Task 2 Demo: Collision Detection System")
    print("=" * 70)
    
    # Create test geometry
    print("\n[Scenario 1] No Collision - Safe Attachment")
    print("-" * 70)
    
    new_link = CylinderGeometry(
        base=Vec3(0, 0, 1.0),
        axis=Vec3(0, 0, 1),
        height=0.5,
        radius=0.05
    )
    
    siblings = [
        ("branch1", CylinderGeometry(
            base=Vec3(0.5, 0, 1.0),
            axis=Vec3(0, 0, 1),
            height=0.5,
            radius=0.05
        )),
        ("branch2", CylinderGeometry(
            base=Vec3(-0.5, 0, 1.0),
            axis=Vec3(0, 0, 1),
            height=0.5,
            radius=0.05
        ))
    ]
    
    parent = CylinderGeometry(
        base=Vec3(0, 0, 0),
        axis=Vec3(0, 0, 1),
        height=1.0,
        radius=0.1
    )
    
    result = check_attachment_collision(new_link, siblings, parent, margin=0.01)
    
    print(f"New link: base={new_link.base}, height={new_link.height}, radius={new_link.radius}")
    print(f"Siblings: {len(siblings)} branches")
    print(f"\nResult:")
    print(f"  Collision detected: {result.collision_detected}")
    print(f"  Stage: {result.stage_detected}")
    print(f"  Details: {result.details}")
    
    # Scenario 2: Collision detected
    print("\n[Scenario 2] Collision Detected - Overlapping Siblings")
    print("-" * 70)
    
    new_link2 = CylinderGeometry(
        base=Vec3(0, 0, 1.0),
        axis=Vec3(0, 0, 1),
        height=0.5,
        radius=0.05
    )
    
    siblings2 = [
        ("branch1", CylinderGeometry(
            base=Vec3(0.08, 0, 1.0),  # Very close - will collide
            axis=Vec3(0, 0, 1),
            height=0.5,
            radius=0.05
        ))
    ]
    
    result2 = check_attachment_collision(new_link2, siblings2, parent, margin=0.01, check_parent=False)
    
    print(f"New link: base={new_link2.base}")
    print(f"Sibling: base={siblings2[0][1].base} (distance: 0.08m)")
    print(f"\nResult:")
    print(f"  Collision detected: {result2.collision_detected}")
    print(f"  Colliding with: {result2.colliding_with}")
    print(f"  Stage: {result2.stage_detected}")
    
    # Scenario 3: Sphere overlap but AABB separated
    print("\n[Scenario 3] Conservative Sphere Check - False Positive")
    print("-" * 70)
    
    # Two thin cylinders oriented perpendicular - sphere overlaps but AABB doesn't
    link_a = CylinderGeometry(
        base=Vec3(0, 0, 0),
        axis=Vec3(1, 0, 0),  # Horizontal along X
        height=2.0,
        radius=0.05
    )
    
    link_b = CylinderGeometry(
        base=Vec3(0, 0, 0.3),  # Offset vertically
        axis=Vec3(0, 1, 0),  # Horizontal along Y
        height=2.0,
        radius=0.05
    )
    
    # Check sphere overlap
    sphere_a = calculate_bounding_sphere(link_a)
    sphere_b = calculate_bounding_sphere(link_b)
    sphere_overlap = check_sphere_overlap(sphere_a, sphere_b)
    
    # Check AABB overlap
    aabb_a = calculate_aabb(link_a)
    aabb_b = calculate_aabb(link_b)
    aabb_overlap = check_aabb_overlap(aabb_a, aabb_b)
    
    print(f"Link A: horizontal along X, radius={link_a.radius}, height={link_a.height}")
    print(f"Link B: horizontal along Y, radius={link_b.radius}, height={link_b.height}, offset Z=0.3")
    print(f"\nStage 1 - Sphere overlap: {sphere_overlap}")
    print(f"Stage 2 - AABB overlap: {aabb_overlap}")
    print(f"\nResult: Sphere detected potential collision, but AABB confirms separation")
    print(f"  → This demonstrates the value of two-stage checking!")
    
    # Scenario 4: Pairwise collision checking
    print("\n[Scenario 4] Pairwise Collision Check - Multiple Links")
    print("-" * 70)
    
    links = [
        ("trunk", CylinderGeometry(Vec3(0, 0, 0), Vec3(0, 0, 1), 1.0, 0.1)),
        ("branch1", CylinderGeometry(Vec3(0.15, 0, 0.5), Vec3(1, 0, 0), 0.5, 0.05)),
        ("branch2", CylinderGeometry(Vec3(-0.15, 0, 0.5), Vec3(-1, 0, 0), 0.5, 0.05)),
        ("branch3", CylinderGeometry(Vec3(0, 0.15, 0.5), Vec3(0, 1, 0), 0.5, 0.05)),
    ]
    
    collisions = check_pairwise_collisions(links, margin=0.01)
    
    print(f"Checking {len(links)} links:")
    for link_id, geom in links:
        print(f"  - {link_id}: base={geom.base}, axis={geom.axis}, r={geom.radius}")
    
    print(f"\nCollisions found: {len(collisions)}")
    if collisions:
        for id1, id2 in collisions:
            print(f"  - {id1} <-> {id2}")
    else:
        print("  (None - all links properly spaced)")
    
    # Summary
    print("\n" + "=" * 70)
    print("  ✓ Task 2 Complete: Collision Detection Working!")
    print("=" * 70)
    print("\nKey Features Demonstrated:")
    print("  • Stage 1: Fast sphere pre-check")
    print("  • Stage 2: Precise AABB validation")
    print("  • Hybrid approach: Conservative but efficient")
    print("  • Safety margins for numerical stability")
    print("  • Pairwise checking for complex configurations")
    
    print("\nNext Steps:")
    print("  - Task 3: Implement geometry remapping for attachment points")
    print("  - Task 4-8: Implement optimization techniques (will use collision system)")


if __name__ == "__main__":
    main()
