"""
visual_collision_random_test.py - Random Multi-Body Collision Testing

Generates N random cylinders and visualizes collision detection.
Cylinders in collision are colored RED, safe ones are BLUE/GREEN.

Usage:
  uv run python src/exporterV2/core/optimizations/tests/visual_collision_random_test.py
  
  Then enter number of bodies when prompted (e.g., 5, 10, 20)
  Press 'n' to generate a new random configuration
  Press 'q' to quit
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


def generate_random_cylinder(bounds=2.0, min_height=0.3, max_height=1.5, 
                             min_radius=0.05, max_radius=0.15):
    """Generate a random cylinder within bounds."""
    # Random base position
    base_x = np.random.uniform(-bounds, bounds)
    base_y = np.random.uniform(-bounds, bounds)
    base_z = np.random.uniform(0, bounds)
    
    # Random axis direction (normalized)
    axis_x = np.random.uniform(-1, 1)
    axis_y = np.random.uniform(-1, 1)
    axis_z = np.random.uniform(0.2, 1)  # Bias towards upward
    axis_len = np.sqrt(axis_x**2 + axis_y**2 + axis_z**2)
    axis_x /= axis_len
    axis_y /= axis_len
    axis_z /= axis_len
    
    # Random dimensions
    height = np.random.uniform(min_height, max_height)
    radius = np.random.uniform(min_radius, max_radius)
    
    return CylinderGeometry(
        base=Vec3(base_x, base_y, base_z),
        axis=Vec3(axis_x, axis_y, axis_z),
        height=height,
        radius=radius
    )


def check_collision_two_stage(cyl1, cyl2, margin=0.01):
    """Check collision using two-stage algorithm."""
    # Stage 1: Sphere
    sphere1 = calculate_bounding_sphere(cyl1)
    sphere2 = calculate_bounding_sphere(cyl2)
    sphere_overlap = check_sphere_overlap(sphere1, sphere2, margin=margin)
    
    if not sphere_overlap:
        return False
    
    # Stage 2: AABB
    aabb1 = calculate_aabb(cyl1)
    aabb2 = calculate_aabb(cyl2)
    aabb_overlap = check_aabb_overlap(aabb1, aabb2)
    
    return aabb_overlap


def find_all_collisions(cylinders):
    """Find all pairwise collisions in a list of cylinders."""
    n = len(cylinders)
    collision_pairs = set()
    colliding_indices = set()
    
    for i in range(n):
        for j in range(i + 1, n):
            if check_collision_two_stage(cylinders[i], cylinders[j]):
                collision_pairs.add((i, j))
                colliding_indices.add(i)
                colliding_indices.add(j)
    
    return collision_pairs, colliding_indices


def plot_cylinder(ax, geom, color='blue', alpha=0.6, label=None):
    """Plot a cylinder in 3D."""
    n_theta = 20
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
    
    ax.plot_surface(x, y, z_out, color=color, alpha=alpha, edgecolor='black', linewidth=0.2)
    
    # End caps
    cap_theta = np.linspace(0, 2*np.pi, n_theta)
    cap_x = geom.base.x + geom.radius * (perp1.x * np.cos(cap_theta) + perp2.x * np.sin(cap_theta))
    cap_y = geom.base.y + geom.radius * (perp1.y * np.cos(cap_theta) + perp2.y * np.sin(cap_theta))
    cap_z = geom.base.z + geom.radius * (perp1.z * np.cos(cap_theta) + perp2.z * np.sin(cap_theta))
    verts_bottom = [list(zip(cap_x, cap_y, cap_z))]
    ax.add_collection3d(Poly3DCollection(verts_bottom, alpha=alpha, facecolor=color, edgecolor='black', linewidth=0.2))
    
    top_x = geom.base.x + axis_norm.x * geom.height
    top_y = geom.base.y + axis_norm.y * geom.height
    top_z = geom.base.z + axis_norm.z * geom.height
    cap_x_top = top_x + geom.radius * (perp1.x * np.cos(cap_theta) + perp2.x * np.sin(cap_theta))
    cap_y_top = top_y + geom.radius * (perp1.y * np.cos(cap_theta) + perp2.y * np.sin(cap_theta))
    cap_z_top = top_z + geom.radius * (perp1.z * np.cos(cap_theta) + perp2.z * np.sin(cap_theta))
    verts_top = [list(zip(cap_x_top, cap_y_top, cap_z_top))]
    ax.add_collection3d(Poly3DCollection(verts_top, alpha=alpha, facecolor=color, edgecolor='black', linewidth=0.2))
    
    # Axis line
    ax.plot([geom.base.x, top_x], [geom.base.y, top_y], [geom.base.z, top_z], 
            'k--', linewidth=1, alpha=0.5)
    
    if label:
        mid_x = (geom.base.x + top_x) / 2
        mid_y = (geom.base.y + top_y) / 2
        mid_z = (geom.base.z + top_z) / 2
        ax.text(mid_x, mid_y, mid_z, label, fontsize=8, fontweight='bold')


def plot_aabb_wireframe(ax, min_pt, max_pt, color='green', linewidth=1.5, alpha=0.6):
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
        ax.plot3D(*points.T, color=color, linewidth=linewidth, alpha=alpha)


def set_axes_equal(ax):
    """Set 3D axes to equal scale."""
    limits = np.array([ax.get_xlim3d(), ax.get_ylim3d(), ax.get_zlim3d()])
    center = np.mean(limits, axis=1)
    radius = 0.5 * np.max(np.abs(limits[:, 1] - limits[:, 0]))
    ax.set_xlim3d([center[0] - radius, center[0] + radius])
    ax.set_ylim3d([center[1] - radius, center[1] + radius])
    ax.set_zlim3d([center[2] - radius, center[2] + radius])


def plot_aabb_wireframe(ax, min_pt, max_pt, color='green', linewidth=1.5, alpha=0.6):
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
        ax.plot3D(*points.T, color=color, linewidth=linewidth, alpha=alpha)


def visualize_random_scene(n_bodies, seed=None):
    """Generate and visualize random collision scene."""
    if seed is not None:
        np.random.seed(seed)
    
    print(f"\n{'='*70}")
    print(f"  Generating {n_bodies} random cylinders...")
    print('='*70)
    
    # Generate cylinders
    cylinders = [generate_random_cylinder() for _ in range(n_bodies)]
    
    # Find collisions
    print("  Running collision detection...")
    collision_pairs, colliding_indices = find_all_collisions(cylinders)
    
    n_collisions = len(collision_pairs)
    n_safe = n_bodies - len(colliding_indices)
    
    print(f"\n  Results:")
    print(f"    Bodies:           {n_bodies}")
    print(f"    Safe (blue):      {n_safe}")
    print(f"    Colliding (red):  {len(colliding_indices)}")
    print(f"    Collision pairs:  {n_collisions}")
    
    if collision_pairs:
        print(f"\n  Collision pairs:")
        for i, j in sorted(collision_pairs)[:10]:  # Show first 10
            print(f"    - Body {i} ↔ Body {j}")
        if len(collision_pairs) > 10:
            print(f"    ... and {len(collision_pairs) - 10} more")
    
    # Visualize
    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot all cylinders with their AABBs
    for idx, cyl in enumerate(cylinders):
        if idx in colliding_indices:
            cyl_color = 'red'
            aabb_color = 'darkred'
            alpha_cyl = 0.7
            alpha_aabb = 0.5
            label = f"{idx}"
        else:
            cyl_color = 'blue'
            aabb_color = 'darkblue'
            alpha_cyl = 0.5
            alpha_aabb = 0.3
            label = f"{idx}"
        
        # Plot cylinder
        plot_cylinder(ax, cyl, color=cyl_color, alpha=alpha_cyl, label=label)
        
        # Plot AABB
        aabb = calculate_aabb(cyl)
        plot_aabb_wireframe(ax, aabb[0], aabb[1], color=aabb_color, linewidth=1.0, alpha=alpha_aabb)
    
    ax.set_xlabel('X (m)', fontsize=12)
    ax.set_ylabel('Y (m)', fontsize=12)
    ax.set_zlabel('Z (m)', fontsize=12)
    ax.set_title(f'{n_bodies} Random Cylinders with AABBs\n{n_safe} Safe (Blue) | {len(colliding_indices)} Colliding (Red)', 
                 fontsize=14, fontweight='bold')
    set_axes_equal(ax)
    ax.grid(True, alpha=0.3)
    
    # Add legend
    from matplotlib.patches import Patch
    legend_elements = [
        Patch(facecolor='blue', alpha=0.5, label='Safe (Blue cylinder + light blue AABB)'),
        Patch(facecolor='red', alpha=0.7, label='Colliding (Red cylinder + dark red AABB)')
    ]
    ax.legend(handles=legend_elements, loc='upper left', fontsize=9)
    
    # Add info text
    info_text = "Note: Bodies are RED if their AABBs overlap\n(cylinders may look separated but AABBs touch)"
    ax.text2D(0.02, 0.02, info_text, transform=ax.transAxes, fontsize=9, 
              verticalalignment='bottom', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    plt.tight_layout()
    plt.show()
    
    return n_collisions > 0


def main():
    print("=" * 70)
    print("  Random Multi-Body Collision Testing")
    print("=" * 70)
    print("\nThis tool generates N random cylinders and detects collisions.")
    print("Cylinders in collision are colored RED, safe ones are BLUE/GREEN.")
    print("\nControls:")
    print("  - Rotate view: Left-click + drag")
    print("  - Zoom: Right-click + drag")
    print("  - Pan: Middle-click + drag")
    
    while True:
        print("\n" + "=" * 70)
        n_bodies_input = input("\nEnter number of bodies (or 'q' to quit): ").strip()
        
        if n_bodies_input.lower() == 'q':
            print("\n✓ Exiting...")
            break
        
        try:
            n_bodies = int(n_bodies_input)
            if n_bodies < 2:
                print("  ⚠️  Need at least 2 bodies!")
                continue
            if n_bodies > 50:
                print("  ⚠️  Max 50 bodies to avoid performance issues!")
                continue
        except ValueError:
            print("  ⚠️  Invalid input! Enter a number or 'q'")
            continue
        
        # Generate and visualize
        visualize_random_scene(n_bodies)
        
        # Ask for another iteration
        print("\n" + "=" * 70)
        again = input("Generate another configuration? (y/n): ").strip().lower()
        if again != 'y':
            print("\n✓ Done!")
            break
    
    print("\n" + "=" * 70)
    print("  Random collision testing complete!")
    print("=" * 70)


if __name__ == "__main__":
    main()
