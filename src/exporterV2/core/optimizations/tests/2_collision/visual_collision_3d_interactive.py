"""
visual_collision_3d_interactive.py - Interactive 3D Collision Visualization

Opens interactive 3D matplotlib windows that you can rotate/zoom with mouse.
This allows proper verification of collision detection from all angles.

Controls:
- Left mouse: Rotate view
- Right mouse: Zoom
- Middle mouse: Pan
- Close window to move to next scenario

Run with: uv run python src/exporterV2/core/optimizations/tests/visual_collision_3d_interactive.py
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
    """Plot a cylinder with better 3D representation."""
    n_theta = 30
    n_z = 2
    
    # Create mesh along cylinder axis
    theta = np.linspace(0, 2*np.pi, n_theta)
    z = np.linspace(0, geom.height, n_z)
    
    # Create meshgrid
    theta_grid, z_grid = np.meshgrid(theta, z)
    
    # Calculate points in cylinder local coordinates
    # We need to handle arbitrary axis orientation
    axis_norm = Vec3(geom.axis.x, geom.axis.y, geom.axis.z)
    axis_len = np.sqrt(axis_norm.x**2 + axis_norm.y**2 + axis_norm.z**2)
    if axis_len < 1e-10:
        axis_norm = Vec3(0, 0, 1)
    else:
        axis_norm = Vec3(axis_norm.x/axis_len, axis_norm.y/axis_len, axis_norm.z/axis_len)
    
    # Find perpendicular vectors
    if abs(axis_norm.z) < 0.9:
        perp1 = Vec3(0, 0, 1)
    else:
        perp1 = Vec3(1, 0, 0)
    
    # Cross product for first perpendicular
    cross1_x = perp1.y * axis_norm.z - perp1.z * axis_norm.y
    cross1_y = perp1.z * axis_norm.x - perp1.x * axis_norm.z
    cross1_z = perp1.x * axis_norm.y - perp1.y * axis_norm.x
    perp1_len = np.sqrt(cross1_x**2 + cross1_y**2 + cross1_z**2)
    if perp1_len > 1e-10:
        perp1 = Vec3(cross1_x/perp1_len, cross1_y/perp1_len, cross1_z/perp1_len)
    
    # Cross product for second perpendicular
    cross2_x = axis_norm.y * perp1.z - axis_norm.z * perp1.y
    cross2_y = axis_norm.z * perp1.x - axis_norm.x * perp1.z
    cross2_z = axis_norm.x * perp1.y - axis_norm.y * perp1.x
    perp2_len = np.sqrt(cross2_x**2 + cross2_y**2 + cross2_z**2)
    if perp2_len > 1e-10:
        perp2 = Vec3(cross2_x/perp2_len, cross2_y/perp2_len, cross2_z/perp2_len)
    
    # Calculate cylinder surface points
    x = np.zeros_like(theta_grid)
    y = np.zeros_like(theta_grid)
    z_out = np.zeros_like(theta_grid)
    
    for i in range(n_z):
        for j in range(n_theta):
            # Radial offset
            offset_x = geom.radius * (perp1.x * np.cos(theta[j]) + perp2.x * np.sin(theta[j]))
            offset_y = geom.radius * (perp1.y * np.cos(theta[j]) + perp2.y * np.sin(theta[j]))
            offset_z = geom.radius * (perp1.z * np.cos(theta[j]) + perp2.z * np.sin(theta[j]))
            
            # Axial position
            axial_x = axis_norm.x * z_grid[i, j]
            axial_y = axis_norm.y * z_grid[i, j]
            axial_z = axis_norm.z * z_grid[i, j]
            
            # Final position
            x[i, j] = geom.base.x + axial_x + offset_x
            y[i, j] = geom.base.y + axial_y + offset_y
            z_out[i, j] = geom.base.z + axial_z + offset_z
    
    # Plot surface
    ax.plot_surface(x, y, z_out, color=color, alpha=alpha, edgecolor='black', linewidth=0.3)
    
    # Plot end caps
    cap_theta = np.linspace(0, 2*np.pi, n_theta)
    
    # Bottom cap
    cap_x = geom.base.x + geom.radius * (perp1.x * np.cos(cap_theta) + perp2.x * np.sin(cap_theta))
    cap_y = geom.base.y + geom.radius * (perp1.y * np.cos(cap_theta) + perp2.y * np.sin(cap_theta))
    cap_z = geom.base.z + geom.radius * (perp1.z * np.cos(cap_theta) + perp2.z * np.sin(cap_theta))
    verts_bottom = [list(zip(cap_x, cap_y, cap_z))]
    ax.add_collection3d(Poly3DCollection(verts_bottom, alpha=alpha, facecolor=color, edgecolor='black'))
    
    # Top cap
    top_x = geom.base.x + axis_norm.x * geom.height
    top_y = geom.base.y + axis_norm.y * geom.height
    top_z = geom.base.z + axis_norm.z * geom.height
    cap_x_top = top_x + geom.radius * (perp1.x * np.cos(cap_theta) + perp2.x * np.sin(cap_theta))
    cap_y_top = top_y + geom.radius * (perp1.y * np.cos(cap_theta) + perp2.y * np.sin(cap_theta))
    cap_z_top = top_z + geom.radius * (perp1.z * np.cos(cap_theta) + perp2.z * np.sin(cap_theta))
    verts_top = [list(zip(cap_x_top, cap_y_top, cap_z_top))]
    ax.add_collection3d(Poly3DCollection(verts_top, alpha=alpha, facecolor=color, edgecolor='black'))
    
    # Plot axis line
    ax.plot([geom.base.x, top_x], [geom.base.y, top_y], [geom.base.z, top_z], 
            'k--', linewidth=2, alpha=0.7)
    
    if label:
        mid_x = (geom.base.x + top_x) / 2
        mid_y = (geom.base.y + top_y) / 2
        mid_z = (geom.base.z + top_z) / 2
        ax.text(mid_x, mid_y, mid_z, label, fontsize=10, fontweight='bold')


def plot_sphere_wireframe(ax, center, radius, color='red', alpha=0.3):
    """Plot a wireframe bounding sphere."""
    u = np.linspace(0, 2 * np.pi, 30)
    v = np.linspace(0, np.pi, 20)
    x = center.x + radius * np.outer(np.cos(u), np.sin(v))
    y = center.y + radius * np.outer(np.sin(u), np.sin(v))
    z = center.z + radius * np.outer(np.ones(np.size(u)), np.cos(v))
    ax.plot_wireframe(x, y, z, color=color, alpha=alpha, linewidth=0.5)


def plot_aabb_wireframe(ax, min_pt, max_pt, color='green', linewidth=2):
    """Plot AABB as wireframe box."""
    # 8 corners
    corners = np.array([
        [min_pt.x, min_pt.y, min_pt.z],
        [max_pt.x, min_pt.y, min_pt.z],
        [max_pt.x, max_pt.y, min_pt.z],
        [min_pt.x, max_pt.y, min_pt.z],
        [min_pt.x, min_pt.y, max_pt.z],
        [max_pt.x, min_pt.y, max_pt.z],
        [max_pt.x, max_pt.y, max_pt.z],
        [min_pt.x, max_pt.y, max_pt.z],
    ])
    
    # 12 edges
    edges = [
        [0, 1], [1, 2], [2, 3], [3, 0],  # bottom
        [4, 5], [5, 6], [6, 7], [7, 4],  # top
        [0, 4], [1, 5], [2, 6], [3, 7],  # vertical
    ]
    
    for edge in edges:
        points = corners[edge]
        ax.plot3D(*points.T, color=color, linewidth=linewidth, alpha=0.8)


def set_axes_equal(ax):
    """Set 3D plot axes to equal scale."""
    limits = np.array([ax.get_xlim3d(), ax.get_ylim3d(), ax.get_zlim3d()])
    center = np.mean(limits, axis=1)
    radius = 0.5 * np.max(np.abs(limits[:, 1] - limits[:, 0]))
    ax.set_xlim3d([center[0] - radius, center[0] + radius])
    ax.set_ylim3d([center[1] - radius, center[1] + radius])
    ax.set_zlim3d([center[2] - radius, center[2] + radius])


def test_scenario_1():
    """Scenario 1: No collision - well separated."""
    print("\n[Interactive 3D Test 1] No Collision")
    print("  Controls: Left-click drag to rotate, right-click to zoom")
    print("  Close window to continue...")
    
    fig = plt.figure(figsize=(14, 7))
    
    link1 = CylinderGeometry(Vec3(0, 0, 0), Vec3(0, 0, 1), 1.0, 0.1)
    link2 = CylinderGeometry(Vec3(0.5, 0, 0.3), Vec3(1, 0, 0), 0.8, 0.08)
    
    sphere1 = calculate_bounding_sphere(link1)
    sphere2 = calculate_bounding_sphere(link2)
    sphere_overlap = check_sphere_overlap(sphere1, sphere2, margin=0.01)
    
    aabb1 = calculate_aabb(link1)
    aabb2 = calculate_aabb(link2)
    aabb_overlap = check_aabb_overlap(aabb1, aabb2)
    
    # Plot with spheres
    ax1 = fig.add_subplot(121, projection='3d')
    plot_cylinder(ax1, link1, 'blue', 0.7, 'Link 1')
    plot_cylinder(ax1, link2, 'orange', 0.7, 'Link 2')
    plot_sphere_wireframe(ax1, sphere1[0], sphere1[1], 'red', 0.4)
    plot_sphere_wireframe(ax1, sphere2[0], sphere2[1], 'red', 0.4)
    
    ax1.set_xlabel('X (m)', fontsize=10)
    ax1.set_ylabel('Y (m)', fontsize=10)
    ax1.set_zlabel('Z (m)', fontsize=10)
    ax1.set_title(f'Stage 1: Bounding Spheres\nOverlap: {sphere_overlap}', fontsize=12)
    set_axes_equal(ax1)
    ax1.grid(True, alpha=0.3)
    
    # Plot with AABBs
    ax2 = fig.add_subplot(122, projection='3d')
    plot_cylinder(ax2, link1, 'blue', 0.7, 'Link 1')
    plot_cylinder(ax2, link2, 'orange', 0.7, 'Link 2')
    plot_aabb_wireframe(ax2, aabb1[0], aabb1[1], 'green', 2)
    plot_aabb_wireframe(ax2, aabb2[0], aabb2[1], 'green', 2)
    
    ax2.set_xlabel('X (m)', fontsize=10)
    ax2.set_ylabel('Y (m)', fontsize=10)
    ax2.set_zlabel('Z (m)', fontsize=10)
    ax2.set_title(f'Stage 2: AABBs\nOverlap: {aabb_overlap}', fontsize=12)
    set_axes_equal(ax2)
    ax2.grid(True, alpha=0.3)
    
    fig.suptitle('Scenario 1: NO COLLISION ✓', fontsize=16, fontweight='bold', color='green')
    plt.tight_layout()
    plt.show()


def test_scenario_2():
    """Scenario 2: Collision detected."""
    print("\n[Interactive 3D Test 2] Collision Detected")
    print("  Controls: Left-click drag to rotate, right-click to zoom")
    print("  Close window to continue...")
    
    fig = plt.figure(figsize=(14, 7))
    
    link1 = CylinderGeometry(Vec3(0, 0, 0), Vec3(0, 0, 1), 1.0, 0.1)
    link2 = CylinderGeometry(Vec3(0.15, 0, 0.3), Vec3(1, 0, 0), 0.6, 0.08)
    
    sphere1 = calculate_bounding_sphere(link1)
    sphere2 = calculate_bounding_sphere(link2)
    sphere_overlap = check_sphere_overlap(sphere1, sphere2, margin=0.01)
    
    aabb1 = calculate_aabb(link1)
    aabb2 = calculate_aabb(link2)
    aabb_overlap = check_aabb_overlap(aabb1, aabb2)
    
    # Plot with spheres
    ax1 = fig.add_subplot(121, projection='3d')
    plot_cylinder(ax1, link1, 'blue', 0.7, 'Link 1')
    plot_cylinder(ax1, link2, 'red', 0.5, 'Link 2')
    plot_sphere_wireframe(ax1, sphere1[0], sphere1[1], 'red', 0.4)
    plot_sphere_wireframe(ax1, sphere2[0], sphere2[1], 'red', 0.4)
    
    ax1.set_xlabel('X (m)', fontsize=10)
    ax1.set_ylabel('Y (m)', fontsize=10)
    ax1.set_zlabel('Z (m)', fontsize=10)
    ax1.set_title(f'Stage 1: Bounding Spheres\nOverlap: {sphere_overlap} ⚠️', fontsize=12)
    set_axes_equal(ax1)
    ax1.grid(True, alpha=0.3)
    
    # Plot with AABBs
    ax2 = fig.add_subplot(122, projection='3d')
    plot_cylinder(ax2, link1, 'blue', 0.7, 'Link 1')
    plot_cylinder(ax2, link2, 'red', 0.5, 'Link 2')
    plot_aabb_wireframe(ax2, aabb1[0], aabb1[1], 'green', 2)
    plot_aabb_wireframe(ax2, aabb2[0], aabb2[1], 'red', 2)
    
    ax2.set_xlabel('X (m)', fontsize=10)
    ax2.set_ylabel('Y (m)', fontsize=10)
    ax2.set_zlabel('Z (m)', fontsize=10)
    ax2.set_title(f'Stage 2: AABBs\nOverlap: {aabb_overlap} ⚠️', fontsize=12)
    set_axes_equal(ax2)
    ax2.grid(True, alpha=0.3)
    
    fig.suptitle('Scenario 2: COLLISION DETECTED ⚠️', fontsize=16, fontweight='bold', color='red')
    plt.tight_layout()
    plt.show()


def test_scenario_3():
    """Scenario 3: False positive - conservative sphere."""
    print("\n[Interactive 3D Test 3] Conservative Sphere Check")
    print("  Controls: Left-click drag to rotate, right-click to zoom")
    print("  Close window to finish...")
    
    fig = plt.figure(figsize=(14, 7))
    
    link1 = CylinderGeometry(Vec3(-0.8, 0, 0), Vec3(1, 0, 0), 1.6, 0.05)
    link2 = CylinderGeometry(Vec3(0, -0.8, 0.25), Vec3(0, 1, 0), 1.6, 0.05)
    
    sphere1 = calculate_bounding_sphere(link1)
    sphere2 = calculate_bounding_sphere(link2)
    sphere_overlap = check_sphere_overlap(sphere1, sphere2, margin=0.01)
    
    aabb1 = calculate_aabb(link1)
    aabb2 = calculate_aabb(link2)
    aabb_overlap = check_aabb_overlap(aabb1, aabb2)
    
    # Plot with spheres
    ax1 = fig.add_subplot(121, projection='3d')
    plot_cylinder(ax1, link1, 'blue', 0.7, 'Link 1')
    plot_cylinder(ax1, link2, 'orange', 0.7, 'Link 2')
    plot_sphere_wireframe(ax1, sphere1[0], sphere1[1], 'red', 0.4)
    plot_sphere_wireframe(ax1, sphere2[0], sphere2[1], 'red', 0.4)
    
    ax1.set_xlabel('X (m)', fontsize=10)
    ax1.set_ylabel('Y (m)', fontsize=10)
    ax1.set_zlabel('Z (m)', fontsize=10)
    ax1.set_title(f'Stage 1: Bounding Spheres\nOverlap: {sphere_overlap} (Conservative)', fontsize=12)
    set_axes_equal(ax1)
    ax1.grid(True, alpha=0.3)
    
    # Plot with AABBs
    ax2 = fig.add_subplot(122, projection='3d')
    plot_cylinder(ax2, link1, 'blue', 0.7, 'Link 1')
    plot_cylinder(ax2, link2, 'orange', 0.7, 'Link 2')
    plot_aabb_wireframe(ax2, aabb1[0], aabb1[1], 'green', 2)
    plot_aabb_wireframe(ax2, aabb2[0], aabb2[1], 'green', 2)
    
    ax2.set_xlabel('X (m)', fontsize=10)
    ax2.set_ylabel('Y (m)', fontsize=10)
    ax2.set_zlabel('Z (m)', fontsize=10)
    ax2.set_title(f'Stage 2: AABBs\nOverlap: {aabb_overlap} ✓ (Actually Safe)', fontsize=12)
    set_axes_equal(ax2)
    ax2.grid(True, alpha=0.3)
    
    fig.suptitle('Scenario 3: False Positive - Two-Stage Corrects ✓', fontsize=16, fontweight='bold', color='blue')
    plt.tight_layout()
    plt.show()


def main():
    print("=" * 70)
    print("  Interactive 3D Collision Visualization")
    print("=" * 70)
    print("\nThis will open 3 interactive 3D windows sequentially.")
    print("Use your mouse to:")
    print("  • Left-click + drag: Rotate view")
    print("  • Right-click + drag: Zoom in/out")
    print("  • Middle-click + drag: Pan")
    print("\nClose each window to proceed to the next scenario.")
    print("\nPress Enter to start...")
    input()
    
    test_scenario_1()
    test_scenario_2()
    test_scenario_3()
    
    print("\n" + "=" * 70)
    print("  ✓ All interactive visualizations complete!")
    print("=" * 70)
    print("\nDid the collision detection work correctly? (y/n): ", end='')
    response = input().strip().lower()
    
    if response == 'y':
        print("\n✓ Collision detection validated!")
    else:
        print("\n⚠ Please report what looked incorrect.")


if __name__ == "__main__":
    main()
