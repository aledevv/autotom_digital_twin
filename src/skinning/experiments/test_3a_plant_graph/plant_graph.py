"""
plant_graph.py — reusable data-driven plant graph for Test 3A+.

The graph layer is deliberately independent from USD stage authoring.

Each graph node describes one branch. The root owns explicit control points;
child branches are generated automatically from:
    - parent branch
    - normalized attachment position
    - azimuth around the parent tangent
    - tilt away from the parent tangent
    - branch length / curvature / radii

The resulting BranchData objects are built using branch_core_fixed.py.

Physics baseline intentionally matches the user-approved Test 2D-B2:
    E               = 1.5 MPa
    damping ratio   = 4.0
    bend limit      = +/-40 deg
    linear-density mass model
"""

from dataclasses import dataclass
import math

from pxr import Gf

import branch_core_fixed as core


# ============================================================================
# GRAPH SPECS
# ============================================================================

@dataclass(frozen=True)
class PlantPhysicsProfile:
    young_modulus_pa: float = 1.5e6
    damping_ratio: float = 4.0
    bend_limit_deg: float = 40.0


@dataclass(frozen=True)
class PlantBranchNode:
    name: str
    parent: str | None = None

    # Optional per-node override. When omitted, PlantGraph's default profile
    # is used so existing graph definitions keep their current behavior.
    physics_profile: PlantPhysicsProfile | None = None

    # Root-only geometry.
    control_points: tuple[tuple[float, float, float], ...] | None = None

    # Child attachment/orientation.
    attach_fraction: float = 0.5
    azimuth_deg: float = 0.0
    tilt_deg: float = 55.0

    # Child procedural geometry.
    length: float = 0.22
    curvature_side: float = 0.010
    curvature_vertical: float = -0.012

    physics_links: int = 3
    samples_per_control_segment: int = 22
    radial_segments: int = 16

    base_radius: float = 0.011
    tip_radius: float = 0.0065
    linear_density_kg_per_m: float = 0.19

    micro_variation_amplitude: float = 0.010
    micro_variation_cycles: float = 1.5


@dataclass
class ResolvedPlantNode:
    config: PlantBranchNode
    branch: core.BranchData
    physics_profile: PlantPhysicsProfile

    parent_name: str | None = None
    attachment_world: Gf.Vec3d | None = None
    attachment_arc: float | None = None
    parent_link_index: int | None = None
    parent_radius: float | None = None


# ============================================================================
# MATH
# ============================================================================

def _normalize(v):
    v = Gf.Vec3d(v)
    length = float(v.GetLength())

    if length < 1e-10:
        raise ValueError(
            "Cannot normalize zero-length vector."
        )

    return v / length


def _orthogonal_basis(tangent):
    """
    Build two stable perpendicular directions around tangent.

    This is only used to choose the CHILD branch direction around its parent.
    The actual branch surface still uses the validated Parallel Transport Frame
    implemented in branch_core_fixed.py.
    """
    tangent = _normalize(tangent)

    world_z = Gf.Vec3d(
        0.0,
        0.0,
        1.0,
    )

    if abs(float(tangent * world_z)) < 0.92:
        reference = world_z
    else:
        reference = Gf.Vec3d(
            1.0,
            0.0,
            0.0,
        )

    u = _normalize(
        reference
        - tangent
        * float(
            tangent * reference
        )
    )

    v = _normalize(
        Gf.Cross(tangent, u)
    )

    return u, v


def _branch_direction(
    parent_tangent,
    azimuth_deg,
    tilt_deg,
):
    """
    tilt_deg = angle away from parent tangent.

    0 deg  -> child continues along the parent
    90 deg -> child emerges perpendicular to the parent
    """
    tangent = _normalize(
        parent_tangent
    )

    u, v = _orthogonal_basis(
        tangent
    )

    azimuth = math.radians(
        float(azimuth_deg)
    )

    radial = _normalize(
        u * math.cos(azimuth)
        + v * math.sin(azimuth)
    )

    tilt = math.radians(
        float(tilt_deg)
    )

    direction = _normalize(
        tangent * math.cos(tilt)
        + radial * math.sin(tilt)
    )

    # A second local direction used only to give the control points a small,
    # organic 3D curvature.
    side = Gf.Cross(
        direction,
        tangent
    )

    if float(side.GetLength()) < 1e-8:
        side = v

    side = _normalize(
        side
    )

    return direction, side


def _child_control_points(
    parent_branch,
    config,
):
    total_parent_length = float(
        parent_branch.centerline[
            "total_length"
        ]
    )

    attachment_arc = (
        total_parent_length
        * float(
            config.attach_fraction
        )
    )

    origin = core.point_at_arc(
        parent_branch.centerline,
        attachment_arc,
    )

    parent_tangent = core.tangent_at_arc(
        parent_branch.centerline,
        attachment_arc,
    )

    direction, side = _branch_direction(
        parent_tangent,
        config.azimuth_deg,
        config.tilt_deg,
    )

    length = float(
        config.length
    )

    gravity_dir = Gf.Vec3d(
        0.0,
        0.0,
        -1.0,
    )

    # Four smooth control points. Geometry is intentionally simple:
    # the graph abstraction, not branch-shape sophistication, is under test.
    p0 = Gf.Vec3d(origin)

    p1 = (
        p0
        + direction * (0.32 * length)
        + side * (
            0.55
            * config.curvature_side
        )
        + gravity_dir * (
            0.15
            * abs(
                config.curvature_vertical
            )
        )
    )

    p2 = (
        p0
        + direction * (0.68 * length)
        - side * (
            0.35
            * config.curvature_side
        )
        + gravity_dir * (
            0.55
            * abs(
                config.curvature_vertical
            )
        )
    )

    p3 = (
        p0
        + direction * length
        + gravity_dir * abs(
            config.curvature_vertical
        )
    )

    return (
        tuple(p0),
        tuple(p1),
        tuple(p2),
        tuple(p3),
    )


# ============================================================================
# PHYSICS PROFILE
# ============================================================================

def beam_drive_params(
    profile,
    radius,
    link_length,
    linear_density,
):
    """
    Same effective beam-drive formulation used by the approved 2D-B2 baseline.
    """
    radius = float(radius)
    link_length = float(link_length)
    linear_density = float(
        linear_density
    )

    mass = (
        linear_density
        * link_length
    )

    second_moment = (
        math.pi
        * radius**4
        / 4.0
    )

    k_rad = (
        profile.young_modulus_pa
        * second_moment
        / link_length
    )

    j_center = (
        mass
        * (
            3.0 * radius**2
            + link_length**2
        )
        / 12.0
    )

    j_pivot = (
        j_center
        + mass
        * (
            link_length / 2.0
        )**2
    )

    d_rad = (
        2.0
        * profile.damping_ratio
        * math.sqrt(
            k_rad
            * j_pivot
        )
    )

    rad_to_deg = (
        math.pi / 180.0
    )

    return (
        k_rad * rad_to_deg,
        d_rad * rad_to_deg,
    )


def _make_branch_spec(
    config,
    control_points,
    profile,
):
    # Use a representative radius slightly biased toward the stronger base.
    radius = (
        0.65
        * config.base_radius
        + 0.35
        * config.tip_radius
    )

    if config.parent is None:
        # Root length is measured from its generated centerline after the first
        # lightweight spec is built below. A chord estimate is sufficient for K/D.
        estimate = 0.0

        cps = [
            Gf.Vec3d(p)
            for p in control_points
        ]

        for i in range(
            len(cps) - 1
        ):
            estimate += float(
                (
                    cps[i + 1]
                    - cps[i]
                ).GetLength()
            )

        total_length = estimate
    else:
        total_length = float(
            config.length
        )

    link_length = (
        total_length
        / config.physics_links
    )

    stiffness, damping = beam_drive_params(
        profile,
        radius,
        link_length,
        config.linear_density_kg_per_m,
    )

    return core.BranchSpec(
        control_points=tuple(
            tuple(p)
            for p in control_points
        ),
        physics_links=config.physics_links,
        samples_per_control_segment=(
            config.samples_per_control_segment
        ),
        radial_segments=(
            config.radial_segments
        ),
        radius=core.RadiusProfile(
            base_radius=config.base_radius,
            tip_radius=config.tip_radius,
            taper_start=0.04,
            taper_end=0.96,
            swell_fractions=(),
            swell_amplitude=0.0,
            micro_variation_amplitude=(
                config.micro_variation_amplitude
            ),
            micro_variation_cycles=(
                config.micro_variation_cycles
            ),
        ),
        linear_density_kg_per_m=(
            config.linear_density_kg_per_m
        ),
        collider_radius_scale=0.90,
        colliders_per_link=2,
        collider_length_scale=0.92,
        joint_stiffness=stiffness,
        joint_damping=damping,
        bend_limit_deg=(
            profile.bend_limit_deg
        ),
        skin_blend_fraction=0.32,
        show_physics_colliders=False,
    )


# ============================================================================
# PLANT GRAPH
# ============================================================================

class PlantGraph:
    def __init__(
        self,
        nodes,
        physics_profile=None,
    ):
        self.nodes = tuple(nodes)

        self.physics_profile = (
            physics_profile
            or PlantPhysicsProfile()
        )

        self._validate()

    def _validate(self):
        if not self.nodes:
            raise ValueError(
                "PlantGraph requires at least one branch."
            )

        names = [
            node.name
            for node in self.nodes
        ]

        if len(set(names)) != len(names):
            raise ValueError(
                "PlantGraph branch names must be unique."
            )

        roots = [
            node
            for node in self.nodes
            if node.parent is None
        ]

        if len(roots) != 1:
            raise ValueError(
                "PlantGraph requires exactly one root branch."
            )

        known = set(names)

        for node in self.nodes:
            if node.parent is not None:
                if node.parent not in known:
                    raise ValueError(
                        f"Branch '{node.name}' references "
                        f"unknown parent '{node.parent}'."
                    )

                if not (
                    0.0
                    < node.attach_fraction
                    < 1.0
                ):
                    raise ValueError(
                        f"Branch '{node.name}' attach_fraction "
                        "must be inside (0, 1)."
                    )

            if node.physics_links < 1:
                raise ValueError(
                    f"Branch '{node.name}' needs at least one physics link."
                )

            if node.base_radius <= 0.0:
                raise ValueError(
                    f"Branch '{node.name}' base_radius must be positive."
                )

            if node.tip_radius <= 0.0:
                raise ValueError(
                    f"Branch '{node.name}' tip_radius must be positive."
                )

    def topological_order(self):
        """
        Small deterministic topological sort.
        """
        remaining = {
            node.name: node
            for node in self.nodes
        }

        ordered = []
        resolved = set()

        while remaining:
            progress = False

            for name in list(
                remaining.keys()
            ):
                node = remaining[name]

                if (
                    node.parent is None
                    or node.parent in resolved
                ):
                    ordered.append(node)
                    resolved.add(name)
                    del remaining[name]
                    progress = True

            if not progress:
                cycle = ", ".join(
                    sorted(
                        remaining.keys()
                    )
                )

                raise ValueError(
                    "PlantGraph contains a cycle or unresolved "
                    f"dependency: {cycle}"
                )

        return ordered

    def resolve(self):
        """
        Convert graph DATA into BranchData objects.

        No USD prims are authored here.
        """
        resolved = {}
        ordered = []

        for config in self.topological_order():
            if config.parent is None:
                if not config.control_points:
                    raise ValueError(
                        "Root branch requires explicit control_points."
                    )

                control_points = (
                    config.control_points
                )

                attachment_world = None
                attachment_arc = None
                parent_link_index = None
                parent_radius = None

            else:
                parent_node = resolved[
                    config.parent
                ]

                parent_branch = (
                    parent_node.branch
                )

                control_points = (
                    _child_control_points(
                        parent_branch,
                        config,
                    )
                )

                attachment_arc = (
                    float(
                        parent_branch.centerline[
                            "total_length"
                        ]
                    )
                    * config.attach_fraction
                )

                attachment_world = (
                    core.point_at_arc(
                        parent_branch.centerline,
                        attachment_arc,
                    )
                )

                parent_link_index = (
                    core.physics_link_for_arc(
                        parent_branch.physics,
                        attachment_arc,
                    )
                )

                parent_radius = (
                    core.radius_for_arc(
                        parent_branch.spec,
                        parent_branch.centerline,
                        attachment_arc,
                    )
                )

            physics_profile = (
                config.physics_profile
                or self.physics_profile
            )

            spec = _make_branch_spec(
                config,
                control_points,
                physics_profile,
            )

            branch = core.make_branch_data(
                config.name,
                spec,
            )

            node = ResolvedPlantNode(
                config=config,
                branch=branch,
                physics_profile=physics_profile,
                parent_name=config.parent,
                attachment_world=attachment_world,
                attachment_arc=attachment_arc,
                parent_link_index=parent_link_index,
                parent_radius=parent_radius,
            )

            resolved[
                config.name
            ] = node

            ordered.append(
                node
            )

        return ordered

    def root_name(self):
        for node in self.nodes:
            if node.parent is None:
                return node.name

        raise RuntimeError(
            "Validated graph has no root."
        )
