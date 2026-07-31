"""
tree_config.py

Configuration and physics helpers for the recursive tree articulation experiment.

Physics are derived from Euler-Bernoulli beam theory, identical to generate_cantilever_usda.py:
    I  = π r⁴ / 4              [m⁴]  second moment of area
    K  = E · I / L             [N·m/rad] bending stiffness per joint
    D  = 2ζ √(K · M)           [N·m·s/rad] damping
    M  = ρ · π · r² · h        [kg] link mass

Dimensions are defined pre-scale (in meters at GLOBAL_SCALE=1), then multiplied
by GLOBAL_SCALE when used in generation. Physics params are always computed
AFTER scaling so that K/D are consistent with the world-unit lengths.

Run standalone to verify physics values per level:
    python tree_config.py
"""

import math

# ==============================================================================
# GLOBAL SCALE
# ==============================================================================

GLOBAL_SCALE = 10.0   # All raw dimensions are multiplied by this

# ==============================================================================
# BIOLOGICAL PHYSICS CONSTANTS
# ==============================================================================

class BioConfig:
    YOUNG_MODULUS  = 1.5e8   # [Pa] 150 MPa — mature stem (cantilever benchmark default)
    DAMPING_RATIO  = 0.2     # ζ, dimensionless
    PLANT_DENSITY  = 1000.0  # [kg/m³]

# ==============================================================================
# TREE STRUCTURE CONFIGURATION
# ==============================================================================

# Start simple: trunk → 1 branch → 1 sub-branch
# Dimensions are PRE-scale (meters). Generator applies GLOBAL_SCALE.
#
# children_per_level[i]  : how many children the last link of level i spawns
# n_links_per_level[i]   : number of rigid links in a chain at level i
# radius_per_level[i]    : cylinder radius at level i  [m pre-scale]
# height_per_level[i]    : cylinder height (per link)  [m pre-scale]
# tilt_per_level[i]      : tilt angle of children of level i away from parent axis [deg]
# rot_per_level[i]       : base azimuthal rotation for first child of level i [deg]
#
# Level 0 = trunk (vertical, anchored to world)
# Level 1 = first-order branches
# Level 2 = second-order sub-branches
#
# Max child radius is 0.005 m (0.5 cm) pre-scale → 0.05 m (5 cm) in world units.

TREE_CONFIG = {
    "depth"              : 3,           # trunk + branch + sub-branch
    "children_per_level" : [1, 1],      # index i = children of level i  (len = depth-1)
    "n_links_per_level"  : [5, 4, 3],   # links per chain at each level
    "radius_per_level"   : [0.10,       # trunk   : 10 cm pre-scale → 1.0 m world
                             0.03,      # branch  :  3 cm pre-scale → 0.3 m world
                             0.005],    # subbranch: 0.5 cm pre-scale → 5 cm world (max)
    "height_per_level"   : [0.20,       # trunk link height : 20 cm → 2.0 m world
                             0.15,      # branch            : 15 cm → 1.5 m world
                             0.10],     # sub-branch        : 10 cm → 1.0 m world
    "tilt_per_level"     : [45.0,       # branches tilt 45° from trunk axis
                             40.0],     # sub-branches tilt 40° from branch axis
    "rot_per_level"      : [0.0,        # first branch azimuth
                             90.0],     # first sub-branch azimuth
    "gap"                : 0.001,       # gap between adjacent links [m pre-scale]
    "bend_limit_deg"     : 30.0,        # ±30° soft limit on rotX/rotY drives
}

# ==============================================================================
# PHYSICS HELPERS
# ==============================================================================

def compute_mass(radius: float, height: float) -> float:
    """Cylindrical segment mass [kg]. Inputs in world-unit meters."""
    volume = math.pi * (radius ** 2) * height
    return BioConfig.PLANT_DENSITY * volume


def compute_second_moment(radius: float) -> float:
    """Second moment of area for a solid cylinder [m⁴]."""
    return (math.pi * (radius ** 4)) / 4.0


def calculate_physics_params(radius: float, height: float, mass: float) -> tuple[float, float]:
    """
    Return (K, D) for a cylindrical beam segment using Euler-Bernoulli theory.

    K [N·m/rad]     = E · I / L   (bending stiffness, used as joint drive stiffness)
    D [N·m·s/rad]   = 2ζ √(K·M)  (damping coefficient)

    Args:
        radius: cylinder radius  [m, world units — i.e. after GLOBAL_SCALE]
        height: cylinder height  [m, world units]
        mass:   cylinder mass    [kg]
    """
    I = compute_second_moment(radius)
    K = (BioConfig.YOUNG_MODULUS * I) / height
    D = 2.0 * BioConfig.DAMPING_RATIO * math.sqrt(K * mass)
    return K, D


def scaled(value: float) -> float:
    """Apply GLOBAL_SCALE to a pre-scale dimension."""
    return value * GLOBAL_SCALE


# ==============================================================================
# SUMMARY PRINTER
# ==============================================================================

def print_tree_summary() -> None:
    """Print a table of physical parameters for each tree level."""
    depth   = TREE_CONFIG["depth"]
    radii   = TREE_CONFIG["radius_per_level"]
    heights = TREE_CONFIG["height_per_level"]
    names   = ["Trunk", "Branch", "Sub-branch", "Level-3", "Level-4"]

    print()
    print("=" * 80)
    print(f"  Recursive Tree Config  |  GLOBAL_SCALE = {GLOBAL_SCALE}  |  E = {BioConfig.YOUNG_MODULUS:.2e} Pa")
    print("=" * 80)
    header = f"  {'Level':<12} {'Radius(m)':>10} {'Height(m)':>10} {'Mass(kg)':>10} {'K(N·m/r)':>12} {'D(N·m·s)':>12} {'T(s)':>8}"
    print(header)
    print("-" * 80)

    for lvl in range(depth):
        r_world = scaled(radii[lvl])
        h_world = scaled(heights[lvl])
        m       = compute_mass(r_world, h_world)
        K, D    = calculate_physics_params(r_world, h_world, m)
        T       = 2.0 * math.pi * math.sqrt(m / K) if K > 0 else float("inf")
        name    = names[lvl] if lvl < len(names) else f"Level-{lvl}"
        n_links = TREE_CONFIG["n_links_per_level"][lvl]
        children = TREE_CONFIG["children_per_level"][lvl] if lvl < len(TREE_CONFIG["children_per_level"]) else 0
        print(f"  {name:<12} {r_world:>10.4f} {h_world:>10.4f} {m:>10.4f} {K:>12.2f} {D:>12.4f} {T:>8.4f}")

    print("-" * 80)
    total_links = sum(
        TREE_CONFIG["n_links_per_level"][lvl] * (
            1 if lvl == 0 else
            math.prod(TREE_CONFIG["children_per_level"][:lvl])
        )
        for lvl in range(depth)
    )
    print(f"  Total links (PhysX): {int(total_links)}  (max 64 allowed)")
    print()

    # Stiffness monotonicity check
    Ks = []
    for lvl in range(depth):
        r_w = scaled(radii[lvl])
        h_w = scaled(heights[lvl])
        m   = compute_mass(r_w, h_w)
        K, _ = calculate_physics_params(r_w, h_w, m)
        Ks.append(K)

    ok = all(Ks[i] > Ks[i+1] for i in range(len(Ks)-1))
    status = "✅" if ok else "⚠️  K not monotonically decreasing — check config!"
    print(f"  K monotonically decreasing (trunk > branch > sub): {status}")
    print()


if __name__ == "__main__":
    print_tree_summary()
