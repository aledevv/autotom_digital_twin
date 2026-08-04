"""
tree_config.py

Configuration and physics helpers for the recursive tree articulation experiment.

Physics (Euler-Bernoulli beam theory):
    I  = pi r^4 / 4              [m^4]
    K  = E * I / L               [N*m/rad]
    D  = 2*zeta * sqrt(K * M)    [N*m*s/rad]
    M  = rho * pi * r^2 * h      [kg]

All dimensions in BRANCHES are PRE-scale (meters at GLOBAL_SCALE=1).
The generator multiplies by GLOBAL_SCALE before building USD or computing physics.

--- BRANCHES list format ---

Each dict describes one chain (trunk or branch).  Fields:

  id          (str)        Unique identifier for this chain.
  parent      (str|None)   id of the parent chain, or None for the trunk.
  attach_link (int|None)   1-based index of the parent link to attach to.
                           None when parent is None.
                           1 = first (bottom) link, n_links = top link.
  n_links     (int)        Number of rigid segments in this chain.
  radius      (float)      Cylinder radius  [m, pre-scale].
  height      (float)      Cylinder height per link [m, pre-scale].
  tilt        (float)      Tilt angle away from parent local-Z axis [deg].
  rot         (float)      Azimuthal rotation around parent local-Z axis [deg].

Run standalone to verify physics:
    python tree_config.py
"""

import math

MAX_N_LINK = 100  # PhysX articulation limit (for 16gb GPU, max tested about 250)

# ==============================================================================
# GLOBAL SCALE & PHYSICS CONSTANTS
# ==============================================================================

GLOBAL_SCALE = 2.0     # All raw dimensions are multiplied by this

BEND_LIMIT_DEG = 30.0   # +/- deg soft limit on rotX/rotY joint drives
GAP            = 0.001  # Gap between adjacent links [m, pre-scale]


class BioConfig:
    YOUNG_MODULUS = 80.0e6   # [Pa] 20-50 MPa - mature tomato stem
    DAMPING_RATIO = 0.3      # 0.1-0.2 zeta, dimensionless
    PLANT_DENSITY = 1000.0   # [kg/m^3] plant tissue density


# ==============================================================================
# BRANCH LIST
# ==============================================================================

BRANCHES = [
    {
        "id"         : "trunk",
        "parent"     : None,
        "attach_link": None,
        "n_links"    : 5,
        "radius"     : 0.10,   # 10 cm -> 1.0 m world
        "height"     : 0.20,   # 20 cm -> 2.0 m world
        "tilt"       : 0.0,
        "rot"        : 0.0,
    },
    {
        "id"         : "branchA",
        "parent"     : "trunk",
        "attach_link": 3,      # attaches to trunk link 3 (1-based)
        "n_links"    : 4,
        "radius"     : 0.03,   # 3 cm -> 0.3 m world
        "height"     : 0.15,   # 15 cm -> 1.5 m world
        "tilt"       : 45.0,
        "rot"        : 0.0,
    },
    {
        "id"         : "subA1",
        "parent"     : "branchA",
        "attach_link": 2,      # attaches to branchA link 2 (1-based)
        "n_links"    : 3,
        "radius"     : 0.005,  # 0.5 cm -> 5 cm world  (max child radius)
        "height"     : 0.10,   # 10 cm -> 1.0 m world
        "tilt"       : 40.0,
        "rot"        : 90.0,
    },
]


# ==============================================================================
# PHYSICS HELPERS
# ==============================================================================

def scaled(value: float) -> float:
    """Apply GLOBAL_SCALE to a pre-scale dimension."""
    return value * GLOBAL_SCALE


def compute_mass(radius: float, height: float) -> float:
    """Cylindrical segment mass [kg]. Inputs in world-unit meters."""
    return BioConfig.PLANT_DENSITY * math.pi * (radius ** 2) * height


def compute_second_moment(radius: float) -> float:
    """Second moment of area for a solid cylinder [m^4]."""
    return (math.pi * (radius ** 4)) / 4.0

def compute_moment_of_inertia(radius: float, height: float, mass: float) -> float:
    """Moment of inertia of a solid cylinder about an axis through one end (parallel-axis theorem)."""
    J_center = mass * (3.0 * radius**2 + height**2) / 12.0
    J_pivot = J_center + mass * (height / 2.0)**2
    return J_pivot


def calculate_physics_params(radius: float, height: float, mass: float):
    """
    Compute spring constant K [N*m/rad] and damper D [N*m*s/rad] for a cylindrical link.
    Inputs are in world-unit meters.
    Returns tuple (K, D) spring constant and damping coefficient.
    """
    I = compute_second_moment(radius)
    K = (BioConfig.YOUNG_MODULUS * I) / height
    J = compute_moment_of_inertia(radius, height, mass)
    D = 2.0 * BioConfig.DAMPING_RATIO * math.sqrt(K * J)

    # Isaac Sim sets stiffness and damping w.r.t. deg (not rad), so convert
    rad_to_deg = math.pi / 180.0
    K_deg = K * rad_to_deg
    D_deg = D * rad_to_deg

    return K_deg, D_deg


# ==============================================================================
# VALIDATION
# ==============================================================================

def validate_branches(branches: list, skip_limit_check: bool = False) -> None:
    """
    Validate the BRANCHES list and raise ValueError with a clear message on any issue.

    Checks:
      - No duplicate ids
      - Exactly one root (parent=None)
      - Every parent id exists in the list
      - attach_link is within [1, parent.n_links] for non-root branches
      - Total link count <= 64 (PhysX articulation limit, unless skip_limit_check=True)
    
    Args:
        branches: List of branch definitions to validate
        skip_limit_check: If True, skip the 64-link PhysX limit check (for experimental tests)
    """
    ids = [b["id"] for b in branches]

    # Duplicate ids
    seen = set()
    for bid in ids:
        if bid in seen:
            raise ValueError(f"[tree_config] Duplicate branch id: '{bid}'")
        seen.add(bid)

    # Root count
    roots = [b for b in branches if b.get("parent") is None]
    if len(roots) == 0:
        raise ValueError("[tree_config] No root branch found (parent=None). Add one trunk.")
    if len(roots) > 1:
        root_ids = [r["id"] for r in roots]
        raise ValueError(f"[tree_config] Multiple root branches: {root_ids}. Only one is allowed.")

    # Build id -> n_links map for parent range check
    id_to_nlinks = {b["id"]: b["n_links"] for b in branches}

    for b in branches:
        if b.get("parent") is None:
            continue  # root - skip parent checks

        parent_id = b["parent"]
        if parent_id not in id_to_nlinks:
            raise ValueError(
                f"[tree_config] Branch '{b['id']}' references unknown parent '{parent_id}'. "
                f"Known ids: {ids}"
            )

        attach = b.get("attach_link")
        if attach is None:
            raise ValueError(
                f"[tree_config] Branch '{b['id']}' has a parent but no 'attach_link'. "
                f"Set attach_link to an integer in [1, {id_to_nlinks[parent_id]}]."
            )
        if not isinstance(attach, int):
            raise ValueError(
                f"[tree_config] Branch '{b['id']}' attach_link must be an int, "
                f"got {type(attach).__name__}."
            )
        parent_nlinks = id_to_nlinks[parent_id]
        if not (1 <= attach <= parent_nlinks):
            raise ValueError(
                f"[tree_config] Branch '{b['id']}' attach_link={attach} is out of range "
                f"[1, {parent_nlinks}] for parent '{parent_id}'."
            )

    # PhysX limit (can be disabled for experimental tests)
    total = sum(b["n_links"] for b in branches)
    if total > MAX_N_LINK and not skip_limit_check:
        raise ValueError(
            f"[tree_config] Total link count {total} exceeds PhysX articulation limit of 64. "
            f"Reduce n_links in some branches."
        )


# ==============================================================================
# SUMMARY PRINTER
# ==============================================================================

def print_tree_summary(branches=None) -> None:
    """Print physics parameters for every branch in the list."""
    if branches is None:
        branches = BRANCHES

    validate_branches(branches)

    print()
    print("=" * 88)
    print(f"  Tree Config  |  GLOBAL_SCALE={GLOBAL_SCALE}  |  E={BioConfig.YOUNG_MODULUS:.2e} Pa  |  zeta={BioConfig.DAMPING_RATIO}")
    print("=" * 88)
    hdr = (f"  {'id':<12} {'parent':<12} {'alink':>5} {'r(m)':>8} {'h(m)':>8} "
           f"{'mass(kg)':>9} {'K(N*m/r)':>12} {'D(N*m*s)':>12} {'T(s)':>7}")
    print(hdr)
    print("-" * 88)

    total_links = 0
    for b in branches:
        r_w  = scaled(b["radius"])
        h_w  = scaled(b["height"])
        m    = compute_mass(r_w, h_w)
        K, D = calculate_physics_params(r_w, h_w, m)
        T    = 2.0 * math.pi * math.sqrt(m / K) if K > 0 else float("inf")
        al   = str(b.get("attach_link") or "-")
        pa   = b.get("parent") or "-"
        print(f"  {b['id']:<12} {pa:<12} {al:>5} {r_w:>8.4f} {h_w:>8.4f} "
              f"{m:>9.3f} {K:>12.2f} {D:>12.4f} {T:>7.4f}")
        total_links += b["n_links"]

    print("-" * 88)
    ok = "OK" if total_links <= 64 else "EXCEEDS LIMIT"
    print(f"  Total links: {total_links}  (PhysX limit: 64)  [{ok}]")
    print()


if __name__ == "__main__":
    print_tree_summary()
