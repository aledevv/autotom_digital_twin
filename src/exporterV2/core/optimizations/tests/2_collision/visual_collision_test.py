"""
visual_collision_test.py - Visual Validation for Collision Detection

Creates matplotlib plots showing:
1. Cylinder geometries with bounding spheres and AABBs
2. Collision vs no-collision scenarios
3. Two-stage detection process visualization

This allows manual verification that collision detection works correctly.

Run with: uv run python src/exporterV2/core/optimizations/tests/visual_collision_test.py
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# Add optimizations directory to path
script_dir = os.path.dirname(os.path.abspath(__file__))
optimizations_dir = os.path.join(script_dir, "../..")
sys.path.insert(0, optimizations_dir)

from collision import (
    Vec3, CylinderGeometry,
    calculate_bounding_sphere, check_sphere_overlap,
    calculate_aabb, check_aabb_overlap,
    check_attachment_collision
)


def plot_cylinder(ax, geom, color='blue', alpha=0.6, label=None):
    """Plot a cylinder in 3D."""
    # Create cylinder mesh
    n_points = 20
    theta = np.linspace(0, 2*np.pi, n_points)
    
    # Bottom circle
    x_bottom = geom.base.x + geom.radius * np.cos(theta) * (1 if geom.axis.y == 0 else 0)
    y_bottom = geom.base.y + geom.radius * np.sin(theta) * (1 if geom.axis.x == 0 else 0)
    z_bottom = np.full_like(theta, geom.base.z)
    
    # Top circle
    top = Vec3(
        geom.base.x + geom.axis.x * geom.height,
        geom.base.y + geom.axis.y * geom.height,
        geom.base.z + geom.axis.z * geom.height
    )
    x_top = top.x + geom.radius * np.cos(theta) * (1 if geom.axis.y == 0 else 0)
    y_top = top.y + geom.radius * np.sin(theta) * (1 if geom.axis.x == 0 else 0)
    z_top = np.full_like(theta, top.z)
    
    # Plot cylinder surface
    for i in range(n_points - 1):
        xs = [x_bottom[i], x_bottom[i+1], x_top[i+1], x_top[i]]
        ys = [y_bottom[i], y_bottom[i+1], y_top[i+1], y_top[i]]
        zs = [z_bottom[i], z_bottom[i+1], z_top[i+1], z_top[i]]
        verts = [list(zip(xs, ys, zs))]
        ax.add_collection3d(Poly3DCollection(verts, alpha=alpha, facecolor=color, edgecolor='black', linewidth=0.5))
    
    # Plot axis line
    ax.plot([geom.base.x, top.x], [geom.base.y, top.y], [geom.base.z, top.z], 
            'k--', linewidth=1, alpha=0.5)
    
    if label:
        # Add label at midpoint
        mid_x = (geom.base.x + top.x) / 2
        mid_y = (geom.base.y + top.y) / 2
        mid_z = (geom.base.z + top.z) / 2
        ax.text(mid_x, mid_y, mid_z, label, fontsize=8)


def plot_sphere(ax, center, radius, color='red', alpha=0.2):
    """Plot a bounding sphere."""
    u = np.linspace(0, 2 * np.pi, 20)
    v = np.linspace(0, np.pi, 20)
    x = center.x + radius * np.outer(np.cos(u), np.sin(v))
    y = center.y + radius * np.outer(np.sin(u), np.sin(v))
    z = center.z + radius * np.outer(np.ones(np.size(u)), np.cos(v))
    ax.plot_surface(x, y, z, color=color, alpha=alpha)


def plot_aabb(ax, min_pt, max_pt, color='green', alpha=0.15):
    """Plot an AABB box."""
    # Define 8 corners
    corners = [
        [min_pt.x, min_pt.y, min_pt.z],
        [max_pt.x, min_pt.y, min_pt.z],
        [max_pt.x, max_pt.y, min_pt.z],
        [min_pt.x, max_pt.y, min_pt.z],
        [min_pt.x, min_pt.y, max_pt.z],
        [max_pt.x, min_pt.y, max_pt.z],
        [max_pt.x, max_pt.y, max_pt.z],
        [min_pt.x, max_pt.y, max_pt.z],
    ]
    
    # Define 6 faces
    faces = [
        [corners[0], corners[1], corners[2], corners[3]],  # bottom
        [corners[4], corners[5], corners[6], corners[7]],  # top
        [corners[0], corners[1], corners[5], corners[4]],  # front
        [corners[2], corners[3], corners[7], corners[6]],  # back
        [corners[0], corners[3], corners[7], corners[4]],  # left
        [corners[1], corners[2], corners[6], corners[5]],  # right
    ]
    
    ax.add_collection3d(Poly3DCollection(faces, alpha=alpha, facecolor=color, edgecolor='black', linewidth=1))


def test_scenario_1_no_collision():
    """Scenario 1: No collision - well separated links."""
    print("\n[Visual Test 1] No Collision - Safe Spacing")
    
    fig = plt.figure(figsize=(12, 5))
    
    # Setup geometry
    link1 = CylinderGeometry(Vec3(0, 0, 0), Vec3(0, 0, 1), 1.0, 0.1)
    link2 = CylinderGeometry(Vec3(0.5, 0, 0.3), Vec3(1, 0, 0), 0.8, 0.08)
    
    sphere1 = calculate_bounding_sphere(link1)
    sphere2 = calculate_bounding_sphere(link2)
    sphere_overlap = check_sphere_overlap(sphere1, sphere2, margin=0.01)
    
    aabb1 = calculate_aabb(link1)
    aabb2 = calculate_aabb(link2)
    aabb_overlap = check_aabb_overlap(aabb1, aabb2)
    
    # Plot 1: Geometry + Spheres
    ax1 = fig.add_subplot(121, projection='3d')
    plot_cylinder(ax1, link1, 'blue', label='Link 1')
    plot_cylinder(ax1, link2, 'orange', label='Link 2')
    plot_sphere(ax1, sphere1[0], sphere1[1], 'red', alpha=0.1)
    plot_sphere(ax1, sphere2[0], sphere2[1], 'red', alpha=0.1)
    
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')
    ax1.set_title(f'Stage 1: Sphere Check\nOverlap: {sphere_overlap}')
    ax1.set_box_aspect([1,1,1])
    
    # Plot 2: Geometry + AABBs
    ax2 = fig.add_subplot(122, projection='3d')
    plot_cylinder(ax2, link1, 'blue', label='Link 1')
    plot_cylinder(ax2, link2, 'orange', label='Link 2')
    plot_aabb(ax2, aabb1[0], aabb1[1], 'green', alpha=0.1)
    plot_aabb(ax2, aabb2[0], aabb2[1], 'green', alpha=0.1)
    
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_zlabel('Z')
    ax2.set_title(f'Stage 2: AABB Check\nOverlap: {aabb_overlap}')
    ax2.set_box_aspect([1,1,1])
    
    plt.suptitle('Scenario 1: NO COLLISION - Safe Spacing', fontsize=14, fontweight='bold')
    plt.tight_layout()
    
    # Save
    output_path = os.path.join(script_dir, 'collision_test1_no_collision.png')
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"  ✓ Saved: {output_path}")
    print(f"    Sphere overlap: {sphere_overlap}, AABB overlap: {aabb_overlap}")
    
    return fig


def test_scenario_2_collision():
    """Scenario 2: Collision detected - overlapping links."""
    print("\n[Visual Test 2] Collision Detected - Overlapping")
    
    fig = plt.figure(figsize=(12, 5))
    
    # Setup geometry (overlapping)
    link1 = CylinderGeometry(Vec3(0, 0, 0), Vec3(0, 0, 1), 1.0, 0.1)
    link2 = CylinderGeometry(Vec3(0.15, 0, 0.3), Vec3(1, 0, 0), 0.6, 0.08)  # Overlaps link1
    
    sphere1 = calculate_bounding_sphere(link1)
    sphere2 = calculate_bounding_sphere(link2)
    sphere_overlap = check_sphere_overlap(sphere1, sphere2, margin=0.01)
    
    aabb1 = calculate_aabb(link1)
    aabb2 = calculate_aabb(link2)
    aabb_overlap = check_aabb_overlap(aabb1, aabb2)
    
    # Plot 1: Geometry + Spheres
    ax1 = fig.add_subplot(121, projection='3d')
    plot_cylinder(ax1, link1, 'blue', label='Link 1')
    plot_cylinder(ax1, link2, 'red', alpha=0.5, label='Link 2')
    plot_sphere(ax1, sphere1[0], sphere1[1], 'red', alpha=0.15)
    plot_sphere(ax1, sphere2[0], sphere2[1], 'red', alpha=0.15)
    
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')
    ax1.set_title(f'Stage 1: Sphere Check\nOverlap: {sphere_overlap} ⚠️')
    ax1.set_box_aspect([1,1,1])
    
    # Plot 2: Geometry + AABBs
    ax2 = fig.add_subplot(122, projection='3d')
    plot_cylinder(ax2, link1, 'blue', label='Link 1')
    plot_cylinder(ax2, link2, 'red', alpha=0.5, label='Link 2')
    plot_aabb(ax2, aabb1[0], aabb1[1], 'green', alpha=0.15)
    plot_aabb(ax2, aabb2[0], aabb2[1], 'red', alpha=0.2)
    
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_zlabel('Z')
    ax2.set_title(f'Stage 2: AABB Check\nOverlap: {aabb_overlap} ⚠️')
    ax2.set_box_aspect([1,1,1])
    
    plt.suptitle('Scenario 2: COLLISION DETECTED - Overlapping Links', fontsize=14, fontweight='bold', color='red')
    plt.tight_layout()
    
    # Save
    output_path = os.path.join(script_dir, 'collision_test2_collision.png')
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"  ✓ Saved: {output_path}")
    print(f"    Sphere overlap: {sphere_overlap}, AABB overlap: {aabb_overlap}")
    
    return fig


def test_scenario_3_false_positive():
    """Scenario 3: Sphere overlap but AABB separated (conservative sphere)."""
    print("\n[Visual Test 3] Conservative Sphere - False Positive")
    
    fig = plt.figure(figsize=(12, 5))
    
    # Two perpendicular cylinders - sphere overlaps but AABB doesn't
    link1 = CylinderGeometry(Vec3(-0.8, 0, 0), Vec3(1, 0, 0), 1.6, 0.05)
    link2 = CylinderGeometry(Vec3(0, -0.8, 0.25), Vec3(0, 1, 0), 1.6, 0.05)
    
    sphere1 = calculate_bounding_sphere(link1)
    sphere2 = calculate_bounding_sphere(link2)
    sphere_overlap = check_sphere_overlap(sphere1, sphere2, margin=0.01)
    
    aabb1 = calculate_aabb(link1)
    aabb2 = calculate_aabb(link2)
    aabb_overlap = check_aabb_overlap(aabb1, aabb2)
    
    # Plot 1: Geometry + Spheres
    ax1 = fig.add_subplot(121, projection='3d')
    plot_cylinder(ax1, link1, 'blue', label='Link 1')
    plot_cylinder(ax1, link2, 'orange', label='Link 2')
    plot_sphere(ax1, sphere1[0], sphere1[1], 'red', alpha=0.15)
    plot_sphere(ax1, sphere2[0], sphere2[1], 'red', alpha=0.15)
    
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')
    ax1.set_title(f'Stage 1: Sphere Check\nOverlap: {sphere_overlap} (False Positive)')
    ax1.set_box_aspect([1,1,1])
    
    # Plot 2: Geometry + AABBs
    ax2 = fig.add_subplot(122, projection='3d')
    plot_cylinder(ax2, link1, 'blue', label='Link 1')
    plot_cylinder(ax2, link2, 'orange', label='Link 2')
    plot_aabb(ax2, aabb1[0], aabb1[1], 'green', alpha=0.15)
    plot_aabb(ax2, aabb2[0], aabb2[1], 'green', alpha=0.15)
    
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_zlabel('Z')
    ax2.set_title(f'Stage 2: AABB Check\nOverlap: {aabb_overlap} ✓ (Actually Safe)')
    ax2.set_box_aspect([1,1,1])
    
    plt.suptitle('Scenario 3: Conservative Sphere (False Positive) - AABB Corrects', fontsize=14, fontweight='bold')
    plt.tight_layout()
    
    # Save
    output_path = os.path.join(script_dir, 'collision_test3_false_positive.png')
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"  ✓ Saved: {output_path}")
    print(f"    Sphere overlap: {sphere_overlap}, AABB overlap: {aabb_overlap}")
    print(f"    → Demonstrates value of two-stage checking!")
    
    return fig


def main():
    print("=" * 70)
    print("  Visual Collision Detection Tests")
    print("=" * 70)
    print("\nGenerating 3D visualizations...")
    
    test_scenario_1_no_collision()
    test_scenario_2_collision()
    test_scenario_3_false_positive()
    
    print("\n" + "=" * 70)
    print("  ✓ Visual tests complete!")
    print("=" * 70)
    print("\nGenerated images:")
    print("  1. collision_test1_no_collision.png")
    print("  2. collision_test2_collision.png")
    print("  3. collision_test3_false_positive.png")
    print("\nPlease review the images to verify:")
    print("  • Cylinders are rendered correctly")
    print("  • Bounding spheres enclose cylinders")
    print("  • AABBs enclose cylinders")
    print("  • Collision detection works as expected")
    print("\nPress Enter to close plots...")
    
    # Show plots (non-blocking)
    plt.show(block=False)
    input()
    plt.close('all')


if __name__ == "__main__":
    main()
