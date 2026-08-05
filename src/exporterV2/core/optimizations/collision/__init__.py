"""
Collision Detection System

Two-stage broad-phase collision detection for validating attachment remapping.

Components:
    - Sphere Overlap (Stage 1): Fast pre-check using bounding spheres
    - AABB Overlap (Stage 2): Precision check using axis-aligned bounding boxes
    - Broad Phase: Orchestrator for two-stage checking

Example:
    >>> from exporterV2.core.optimizations.collision import check_attachment_collision
    >>> result = check_attachment_collision(new_link, siblings, parent)
    >>> if result.collision_detected:
    ...     print(f"Collision with: {result.colliding_with}")
"""

# Exports (will be implemented in Task 2)
# from .sphere import calculate_bounding_sphere, check_sphere_overlap
# from .aabb import calculate_aabb, check_aabb_overlap
# from .broad_phase import check_attachment_collision, CollisionResult

__all__ = [
    # "calculate_bounding_sphere",
    # "check_sphere_overlap",
    # "calculate_aabb",
    # "check_aabb_overlap",
    # "check_attachment_collision",
    # "CollisionResult",
]
