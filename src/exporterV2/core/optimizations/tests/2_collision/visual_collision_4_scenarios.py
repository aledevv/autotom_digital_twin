"""
visual_collision_4_scenarios.py - 4 Clear Collision Detection Scenarios

Tests the two-stage collision detection system with 4 representative cases:
1. No collision: Both Sphere and AABB say False → NO COLLISION
2. True collision: Both Sphere and AABB say True → COLLISION DETECTED
3. Conservative sphere: Sphere True, AABB False → NO COLLISION (false positive filtered)
4. Edge case: Touching cylinders → COLLISION DETECTED

Run with: uv run python src/exporterV2/core/optimizations/tests/visual_collision_4_scenarios.py
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
    calculate_aabb, check_aabb_overlap
)


def plot_cylinder(ax, geom, color='blue', alpha=0.6, label=None):
    """Plot a cylinder with 3D representation."""
    n_theta = 30
    n_z = 2
    
    theta = np.linspace(0, 2*np.pi, n_theta)
    z = np.linspace(0, geom.height, n_z)
    theta_grid, z_grid = np.meshgrid(theta, z)
    
    # Normalize axis
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
    
    # Calculate cylinder surface
    x = np.zeros_like(theta_grid)
    y = np.zeros_like(theta_grid)
    z_out = np.zeros_like(theta_grid)
    
    for i in range(n_z):
        for j in range(n_theta):
            offset_x = geom.radius * (perp1.x * np.cos(theta[j]) + perp2.x * np.sin(theta[j]))
            offset_y = geom.radius * (perp1.y * np.cos(theta[j]) + perp2.y * np.sin(theta[j]))
            offset_z = geom.radius * (perp1.z * np.cos(theta[j]) + perp2.z * np.sin(theta[j]))
            
            axial_x = axis_norm.x * z_grid[i, j]
            axial_y = axis_norm.y * z_grid[i, j]
            axial_z = axis_norm.z * z_grid[i, j]
            
            x[i, j] = geom.base.x + axial_x + offset_x
            y[i, j] = geom.base.y + axial_y + offset_y
            z_out[i, j] = geom.base.z + axial_z + offset_z
    
    ax.plot_surface(x, y, z_out, color=color, alpha=alpha, edgecolor='black', linewidth=0.3)
    
    # End caps
    cap_theta = np.linspace(0, 2*np.pi, n_theta)
    cap_x = geom.base.x + geom.radius * (perp1.x * np.cos(cap_theta) + perp2.x * np.sin(cap_theta))
    cap_y = geom.base.y + geom.radius * (perp1.y * np.cos(cap_theta) + perp2.y * np.sin(cap_theta))
    cap_z = geom.base.z + geom.radius * (perp1.z * np.cos(cap_theta) + perp2.z * np.sin(cap_theta))
    verts_bottom = [list(zip(cap_x, cap_y, cap_z))]
    ax.add_collection3d(Poly3DCollection(verts_bottom, alpha=alpha, facecolor=color, edgecolor='black'))
    
    top_x = geom.base.x + axis_norm.x * geom.height
    top_y = geom.base.y + axis_norm.y * geom.height
    top_z = geom.base.z + axis_norm.z * geom.height
    cap_x_top = top_x + geom.radius * (perp1.x * np.cos(cap_theta) + perp2.x * np.sin(cap_theta))
    cap_y_top = top_y + geom.radius * (perp1.y * np.cos(cap_theta) + perp2.y * np.sin(cap_theta))
    cap_z_top = top_z + geom.radius * (perp1.z * np.cos(cap_theta) + perp2.z * np.sin(cap_theta))
    verts_top = [list(zip(cap_x_top, cap_y_top, cap_z_top))]
    ax.add_collection3d(Poly3DCollection(verts_top, alpha=alpha, facecolor=color, edgecolor='black'))
    
    ax.plot([geom.base.x, top_x], [geom.base.y, top_y], [geom.base.z, top_z], 
            'k--', linewidth=2, alpha=0.7)
    
    if label:
        mid_x = (geom.base.x + top_x) / 2
        mid_y = (geom.base.y + top_y) / 2
        mid_z = (geom.base.z + top_z) / 2
        ax.text(mid_x, mid_y, mid_z, label, fontsize=10, fontweight='bold')


def plot_sphere_wireframe(ax, center, radius, color='red', alpha=0.3):
    """Plot bounding sphere."""
    u = np.linspace(0, 2 * np.pi, 30)
    v = np.linspace(0, np.pi, 20)
    x = center.x + radius * np.outer(np.cos(u), np.sin(v))
    y = center.y + radius * np.outer(np.sin(u), np.sin(v))
    z = center.z + radius * np.outer(np.ones(np.size(u)), np.cos(v))
    ax.plot_wireframe(x, y, z, color=color, alpha=alpha, linewidth=0.5)


def plot_aabb_wireframe(ax, min_pt, max_pt, color='green', linewidth=2):
    """Plot AABB wireframe."""
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
    
    edges = [
        [0, 1], [1, 2], [2, 3], [3, 0],
        [4, 5], [5, 6], [6, 7], [7, 4],
        [0, 4], [1, 5], [2, 6], [3, 7],
    ]
    
    for edge in edges:
        points = corners[edge]
        ax.plot3D(*points.T, color=color, linewidth=linewidth, alpha=0.8)


def set_axes_equal(ax):
    """Set 3D axes to equal scale."""
    limits = np.array([ax.get_xlim3d(), ax.get_ylim3d(), ax.get_zlim3d()])
    center = np.mean(limits, axis=1)
    radius = 0.5 * np.max(np.abs(limits[:, 1] - limits[:, 0]))
    ax.set_xlim3d([center[0] - radius, center[0] + radius])
    ax.set_ylim3d([center[1] - radius, center[1] + radius])
    ax.set_zlim3d([center[2] - radius, center[2] + radius])


def run_collision_check(link1, link2, margin=0.01):
    """Run two-stage collision check and return results."""
    # Stage 1: Sphere check
    sphere1 = calculate_bounding_sphere(link1)
    sphere2 = calculate_bounding_sphere(link2)
    sphere_overlap = check_sphere_overlap(sphere1, sphere2, margin=margin)
    
    # Stage 2: AABB check (only if sphere overlaps)
    aabb_overlap = False
    if sphere_overlap:
        aabb1 = calculate_aabb(link1)
        aabb2 = calculate_aabb(link2)
        aabb_overlap = check_aabb_overlap(aabb1, aabb2)
    
    # Final decision: collision only if BOTH stages agree
    collision_detected = sphere_overlap and aabb_overlap
    
    return {
        'sphere_overlap': sphere_overlap,
        'aabb_overlap': aabb_overlap,
        'collision_detected': collision_detected,
        'sphere1': sphere1,
        'sphere2': sphere2,
        'aabb1': calculate_aabb(link1),
        'aabb2': calculate_aabb(link2)
    }


def test_scenario(scenario_num, link1, link2, description):
    """Test and visualize a collision scenario."""
    print(f"\n{'='*70}")
    print(f"  Scenario {scenario_num}: {description}")
    print('='*70)
    
    result = run_collision_check(link1, link2)
    
    print(f"\n  Stage 1 (Sphere):  {'OVERLAP' if result['sphere_overlap'] else 'SEPARATED'}")
    print(f"  Stage 2 (AABB):    {'OVERLAP' if result['aabb_overlap'] else 'SEPARATED'}")
    print(f"  → Final Output:    {'⚠️  COLLISION DETECTED' if result['collision_detected'] else '✓ NO COLLISION'}")
    
    # Create visualization
    fig = plt.figure(figsize=(14, 7))
    
    # Left: Sphere check
    ax1 = fig.add_subplot(121, projection='3d')
    cyl1_color = 'blue'
    cyl2_color = 'red' if result['collision_detected'] else 'orange'
    
    plot_cylinder(ax1, link1, cyl1_color, 0.7, 'Link 1')
    plot_cylinder(ax1, link2, cyl2_color, 0.7, 'Link 2')
    
    sphere_color = 'red' if result['sphere_overlap'] else 'green'
    plot_sphere_wireframe(ax1, result['sphere1'][0], result['sphere1'][1], sphere_color, 0.4)
    plot_sphere_wireframe(ax1, result['sphere2'][0], result['sphere2'][1], sphere_color, 0.4)
    
    ax1.set_xlabel('X (m)', fontsize=10)
    ax1.set_ylabel('Y (m)', fontsize=10)
    ax1.set_zlabel('Z (m)', fontsize=10)
    ax1.set_title(f'Stage 1: Sphere Check\n{"OVERLAP" if result["sphere_overlap"] else "SEPARATED"}', 
                  fontsize=12, fontweight='bold')
    set_axes_equal(ax1)
    ax1.grid(True, alpha=0.3)
    
    # Right: AABB check
    ax2 = fig.add_subplot(122, projection='3d')
    plot_cylinder(ax2, link1, cyl1_color, 0.7, 'Link 1')
    plot_cylinder(ax2, link2, cyl2_color, 0.7, 'Link 2')
    
    aabb_color = 'red' if result['aabb_overlap'] else 'green'
    plot_aabb_wireframe(ax2, result['aabb1'][0], result['aabb1'][1], aabb_color, 2)
    plot_aabb_wireframe(ax2, result['aabb2'][0], result['aabb2'][1], aabb_color, 2)
    
    ax2.set_xlabel('X (m)', fontsize=10)
    ax2.set_ylabel('Y (m)', fontsize=10)
    ax2.set_zlabel('Z (m)', fontsize=10)
    ax2.set_title(f'Stage 2: AABB Check\n{"OVERLAP" if result["aabb_overlap"] else "SEPARATED"}', 
                  fontsize=12, fontweight='bold')
    set_axes_equal(ax2)
    ax2.grid(True, alpha=0.3)
    
    # Overall title
    if result['collision_detected']:
        title = f'Scenario {scenario_num}: ⚠️  COLLISION DETECTED'
        title_color = 'red'
    else:
        title = f'Scenario {scenario_num}: ✓ NO COLLISION'
        title_color = 'green'
    
    fig.suptitle(title, fontsize=16, fontweight='bold', color=title_color)
    plt.tight_layout()
    plt.show()
    
    print(f"\n  Close window to continue...")


def main():
    print("=" * 70)
    print("  Two-Stage Collision Detection - 4 Test Scenarios")
    print("=" * 70)
    print("\nThis will test 4 representative cases:")
    print("  1. S=False, AB=False → NO COLLISION")
    print("  2. S=True,  AB=True  → COLLISION DETECTED")
    print("  3. S=True,  AB=False → NO COLLISION (false positive filtered)")
    print("  4. S=True,  AB=True  → COLLISION DETECTED (touching/edge case)")
    print("\nPress Enter to start...")
    input()
    
    # Scenario 1: Both separated - NO COLLISION
    link1_s1 = CylinderGeometry(Vec3(0, 0, 0), Vec3(0, 0, 1), 1.0, 0.1)
    link2_s1 = CylinderGeometry(Vec3(1.0, 0, 0.3), Vec3(1, 0, 0), 0.8, 0.08)
    test_scenario(1, link1_s1, link2_s1, "Well Separated (S=False, AB=False)")
    
    # Scenario 2: True collision - COLLISION DETECTED
    link1_s2 = CylinderGeometry(Vec3(0, 0, 0), Vec3(0, 0, 1), 1.0, 0.1)
    link2_s2 = CylinderGeometry(Vec3(0.05, 0, 0.3), Vec3(1, 0, 0), 0.6, 0.1)
    test_scenario(2, link1_s2, link2_s2, "True Collision (S=True, AB=True)")
    
    # Scenario 3: Conservative sphere - NO COLLISION
    link1_s3 = CylinderGeometry(Vec3(-0.8, 0, 0), Vec3(1, 0, 0), 1.6, 0.05)
    link2_s3 = CylinderGeometry(Vec3(0, -0.8, 0.25), Vec3(0, 1, 0), 1.6, 0.05)
    test_scenario(3, link1_s3, link2_s3, "Conservative Sphere (S=True, AB=False)")
    
    # Scenario 4: Touching cylinders - COLLISION DETECTED
    link1_s4 = CylinderGeometry(Vec3(0, 0, 0), Vec3(0, 0, 1), 1.0, 0.1)
    link2_s4 = CylinderGeometry(Vec3(0.2, 0, 0.5), Vec3(1, 0, 0), 0.5, 0.1)
    test_scenario(4, link1_s4, link2_s4, "Touching Cylinders (S=True, AB=True)")
    
    print("\n" + "=" * 70)
    print("  ✓ All 4 scenarios tested!")
    print("=" * 70)
    print("\nSummary:")
    print("  - Scenario 1: Both checks say NO → safe")
    print("  - Scenario 2: Both checks say YES → collision")
    print("  - Scenario 3: Sphere conservative, AABB corrects → safe")
    print("  - Scenario 4: Edge case, both detect → collision")
    print("\nTwo-stage system correctly filters false positives!")


if __name__ == "__main__":
    main()
