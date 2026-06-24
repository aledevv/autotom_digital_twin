import math
import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf

from .models import (
    PlantSnapshot, OrganNode, InternodeNode, RootNode, LeafNode, FruitsNode
)

from .constants import (
    INTERNODE_TRUSS_LENGTH_M,
    INTERNODE_TRUSS_DIAMETER_M,
    ANGLE_AMONG_SUBSEQUENT_FRUITS_DEG,
    FRUIT_PAIRING, ROOT_SPHERE_RADIUS,
    PETIOLE_LENGTH_M,
    TRUSS_LENGTH, TRUSS_RADIUS, PHYLLOTAXIS,
    JOINT_STIFFNESS_BASE, JOINT_STIFFNESS_TIP,
    JOINT_DAMPING, JOINT_MAX_ANGLE_DEG, STEM_DENSITY_KG_M3,
)

# Optional override for lateral leaflet insertion angle.
# If set to a float (e.g., 50.0), this angle will be used for all lateral leaflets.
# If set to None, the exact angle from the CSV ('leaf_inclination_segments') will be used.
OVERRIDE_LEAF_INCLINATION: float | None = 50.0


# ─────────────────────────────────────────────────────────────────────────────
# Matrix helpers
# ─────────────────────────────────────────────────────────────────────────────

def _translate(tx: float, ty: float, tz: float) -> np.ndarray:
    m = np.eye(4, dtype=float)
    m[0, 3] = tx
    m[1, 3] = ty
    m[2, 3] = tz
    return m

def _mat_to_gf(m: np.ndarray) -> Gf.Matrix4d:
    # Gf.Matrix4d is column-major — transpose before flatten
    return Gf.Matrix4d(*m.T.flatten().tolist())

def _set_transform(prim, mat: np.ndarray):
    xformable = UsdGeom.Xformable(prim)
    xformable.ClearXformOpOrder()
    xformable.AddTransformOp().Set(_mat_to_gf(mat))


# ─────────────────────────────────────────────────────────────────────────────
# Geometry helpers
# ─────────────────────────────────────────────────────────────────────────────

def _make_cylinder(stage, path: str, height: float, radius: float, base_z: float):
    """
    Cylinder centred at (0, 0, base_z + height/2), axis=Z.
    base_z: world Z of the bottom face.
    """
    centre_z = base_z + height / 2.0
    mat = _translate(0.0, 0.0, centre_z)

    cyl = UsdGeom.Cylinder.Define(stage, path)
    cyl.GetHeightAttr().Set(height)
    cyl.GetRadiusAttr().Set(radius)
    cyl.GetAxisAttr().Set(UsdGeom.Tokens.z)
    _set_transform(cyl, mat)

    print(f"  [Cylinder] {path}")
    print(f"    base_z  = {base_z:.6f} m")
    print(f"    tip_z   = {base_z + height:.6f} m")
    print(f"    height  = {height:.6f} m")
    print(f"    radius  = {radius:.6f} m")
    print(f"    centre  = (0, 0, {centre_z:.6f})")

    return cyl

def _make_sphere(stage, path: str, radius: float, cx: float, cy: float, cz: float):
    """Sphere centred at (cx, cy, cz)."""
    mat = _translate(cx, cy, cz)
    sph = UsdGeom.Sphere.Define(stage, path)
    sph.GetRadiusAttr().Set(radius)
    _set_transform(sph, mat)

    print(f"  [Sphere]   {path}")
    print(f"    centre  = ({cx:.6f}, {cy:.6f}, {cz:.6f})")
    print(f"    radius  = {radius:.6f} m")

    return sph

def _make_leaf(stage, leaf_group: str, node, tip_z: float, materials: dict):
    """
    Leaf = Petiole cylinder + Rachis cylinder + Compound blade quads.
    tip_z: world Z where the leaf attaches (top of parent internode).
    """
    import math

    # If the CSV provides an explicit orientation (non-zero), use it directly.
    # Otherwise fall back to cumulative phyllotaxis (rank * 137.5°) to replicate
    # GroIMP's turtle-based RH rotation that is not captured in the export.
    if abs(node.ccw_orientation) > 1e-3:
        azimuth_deg = node.ccw_orientation
    else:
        azimuth_deg = (node.key.rank * PHYLLOTAXIS) % 360.0

    az  = math.radians(azimuth_deg)            # azimuth around stem
    el  = math.radians(node.angle_petiole)     # elevation from horizontal (90°=horiz)

    tilt = math.radians(90.0 - node.angle_petiole)  # tilt from horizontal
    dx = math.cos(az) * math.cos(tilt)
    dy = math.sin(az) * math.cos(tilt)
    dz = math.sin(tilt)

    Lp = max(node.length_petiole, 1e-4)
    Rp = max(node.diameter_petiole / 2.0, 1e-4)

    # ── Petiole ──────────────────────────────────────────────────────────────
    pcx = dx * Lp / 2.0
    pcy = dy * Lp / 2.0
    pcz = tip_z + dz * Lp / 2.0

    petiole_mat = _align_z_to(dx, dy, dz, pcx, pcy, pcz)

    pet = UsdGeom.Cylinder.Define(stage, f"{leaf_group}/Petiole")
    pet.GetHeightAttr().Set(Lp)
    pet.GetRadiusAttr().Set(Rp)
    pet.GetAxisAttr().Set(UsdGeom.Tokens.z)
    _set_transform(pet, petiole_mat)

    print(f"  [Petiole] az={azimuth_deg:.1f}° el={node.angle_petiole}° "
          f"L={Lp:.4f}m centre=({pcx:.4f},{pcy:.4f},{pcz:.4f})")

    rtx = dx * Lp
    rty = dy * Lp
    rtz = tip_z + dz * Lp

    # ── Rachis ───────────────────────────────────────────────────────────────
    Lr = max(node.rachis_length, 1e-4)
    rachis_mat = _align_z_to(dx, dy, dz, rtx + dx * Lr/2, rty + dy * Lr/2, rtz + dz * Lr/2)

    rac = UsdGeom.Cylinder.Define(stage, f"{leaf_group}/Rachis")
    rac.GetHeightAttr().Set(Lr)
    rac.GetRadiusAttr().Set(max(Rp * 0.6, 5e-5))
    rac.GetAxisAttr().Set(UsdGeom.Tokens.z)
    _set_transform(rac, rachis_mat)
    
    _bind_material(pet, materials["leaf"])
    _bind_material(rac, materials["leaf"])

    # ── Blades (Compound Leaf Logic) ─────────────────────────────────────────
    n = max(node.blades_nr, 1)
    pairs = n - 1

    # Extract parsed arrays from CSV
    area_array = node.leaf_area_m2blades
    seg_len_array = node.leaf_segments_length
    incl_array = node.leaf_inclination_segments

    # Terminal leaflet is the last element in GroIMP's area array (index bladesNr-1).
    # If array is missing, fallback to even distribution.
    terminal_area = area_array[-1] if len(area_array) >= n else (node.area_blades_total / n if node.area_blades_total > 0 else 4e-4)
    terminal_length = math.sqrt(terminal_area / 0.6)
    terminal_width  = terminal_length * 0.6

    # Perpendicular vector to petiole dir, in horizontal plane
    perp_x = -math.sin(az)
    perp_y =  math.cos(az)
    perp_z =  0.0

    mesh_idx = 0

    # 1. Lateral leaflets
    current_dist = 0.0
    for j in range(pairs):
        # We need the segment length to position this pair
        bx = rtx + dx * current_dist
        by = rty + dy * current_dist
        bz = rtz + dz * current_dist

        # Calculate area/length for this pair
        pair_area = area_array[j] if j < len(area_array) else (node.area_blades_total / n)
        lat_area = pair_area / 2.0  # GroIMP does area_m2blades[q]/2 for each leaflet
        lat_length = math.sqrt(lat_area / 0.6)
        lat_width = lat_length * 0.6

        # Determine insertion angle
        if OVERRIDE_LEAF_INCLINATION is not None:
            insertion_angle = OVERRIDE_LEAF_INCLINATION
        else:
            insertion_angle = incl_array[j] if j < len(incl_array) else 90.0

        for side in [1.0, -1.0]:
            blade_mat = _blade_transform(
                bx, by, bz,
                dx, dy, dz,
                perp_x * side, perp_y * side, 0.0,
                insertion_angle
            )
            
            # Draw petiolule cylinder
            y_axis = blade_mat[:3, 1]
            petiolule_len = 0.01  # 1 cm
            
            pcx_l = bx + y_axis[0] * petiolule_len / 2.0
            pcy_l = by + y_axis[1] * petiolule_len / 2.0
            pcz_l = bz + y_axis[2] * petiolule_len / 2.0
            
            pet_mat = _align_z_to(y_axis[0], y_axis[1], y_axis[2], pcx_l, pcy_l, pcz_l)
            
            petl = UsdGeom.Cylinder.Define(stage, f"{leaf_group}/Petiolule_{mesh_idx}")
            petl.GetHeightAttr().Set(petiolule_len)
            petl.GetRadiusAttr().Set(Rp * 0.4)
            petl.GetAxisAttr().Set(UsdGeom.Tokens.z)
            _set_transform(petl, pet_mat)
            _bind_material(petl, materials["leaf"])

            # Shift the blade base to start AFTER the petiolule
            blade_mat[0, 3] += y_axis[0] * petiolule_len
            blade_mat[1, 3] += y_axis[1] * petiolule_len
            blade_mat[2, 3] += y_axis[2] * petiolule_len

            mesh = UsdGeom.Mesh.Define(stage, f"{leaf_group}/Blade_{mesh_idx}")
            mesh_idx += 1
            hw, L = lat_width / 2.0, lat_length
            mesh.GetPointsAttr().Set([
                Gf.Vec3f(-hw, 0, 0), Gf.Vec3f(hw, 0, 0),
                Gf.Vec3f(hw,  L, 0), Gf.Vec3f(-hw,  L, 0),
            ])
            mesh.GetFaceVertexCountsAttr().Set([3, 3])
            mesh.GetFaceVertexIndicesAttr().Set([0, 1, 2, 0, 2, 3])
            mesh.GetSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
            _set_transform(mesh, blade_mat)
            _bind_material(mesh, materials["leaf"])
            
        # Advance distance for the next pair (or terminal leaflet)
        seg_len = seg_len_array[j] if j < len(seg_len_array) else (Lr / max(pairs, 1))
        current_dist += seg_len

    # 2. Terminal leaflet
    # Positioned exactly after the last segment length
    bx = rtx + dx * current_dist
    by = rty + dy * current_dist
    bz = rtz + dz * current_dist

    blade_mat = _blade_transform(
        bx, by, bz,
        dx, dy, dz,
        perp_x, perp_y, 0.0,
        0.0  # 0 degree insertion = straight ahead
    )

    mesh = UsdGeom.Mesh.Define(stage, f"{leaf_group}/Blade_{mesh_idx}")
    hw, L = terminal_width / 2.0, terminal_length
    mesh.GetPointsAttr().Set([
        Gf.Vec3f(-hw, 0, 0), Gf.Vec3f(hw, 0, 0),
        Gf.Vec3f(hw,  L, 0), Gf.Vec3f(-hw,  L, 0),
    ])
    mesh.GetFaceVertexCountsAttr().Set([3, 3])
    mesh.GetFaceVertexIndicesAttr().Set([0, 1, 2, 0, 2, 3])
    mesh.GetSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
    _set_transform(mesh, blade_mat)
    _bind_material(mesh, materials["leaf"])

    print(f"  [Compound Leaf] {n} segments -> {mesh_idx + 1} leaflets created")


def _align_z_to(dx: float, dy: float, dz: float,
                cx: float, cy: float, cz: float) -> np.ndarray:
    """
    Build a 4x4 matrix that places the origin at (cx,cy,cz)
    and rotates Z-axis to point along (dx,dy,dz).
    Uses Rodrigues rotation from (0,0,1) to (dx,dy,dz).
    """
    z = np.array([dx, dy, dz], dtype=float)
    norm = np.linalg.norm(z)
    if norm < 1e-9:
        z = np.array([0.0, 0.0, 1.0])
    else:
        z /= norm

    up = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(z, up)) > 0.999:
        up = np.array([1.0, 0.0, 0.0])

    x = np.cross(up, z); x /= np.linalg.norm(x)
    y = np.cross(z, x)

    m = np.eye(4, dtype=float)
    m[:3, 0] = x
    m[:3, 1] = y
    m[:3, 2] = z
    m[0, 3]  = cx
    m[1, 3]  = cy
    m[2, 3]  = cz
    return m

def _blade_transform(bx, by, bz, ax, ay, az_,
                     px, py, pz,
                     insertion_deg: float) -> np.ndarray:
    """
    Transform for a leaf blade quad.
    (bx,by,bz)     = attachment point on rachis (local origin)
    (ax,ay,az_)    = rachis direction
    (px,py,pz)     = lateral direction (left or right of rachis)
    insertion_deg  = angle of blade from rachis axis
    """
    import math
    ins = math.radians(insertion_deg)

    a = np.array([ax, ay, az_], dtype=float)
    p = np.array([px, py, pz],  dtype=float)
    if np.linalg.norm(p) < 1e-9:
        p = np.array([1.0, 0.0, 0.0])
    else:
        p /= np.linalg.norm(p)

    # Local Y axis is the direction of growth
    y_axis = a * math.cos(ins) + p * math.sin(ins)
    y_axis /= np.linalg.norm(y_axis)

    # Local X axis is perpendicular to growth direction and World Up
    up = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(y_axis, up)) > 0.999:
        up = np.array([1.0, 0.0, 0.0])
        
    x_axis = np.cross(y_axis, up)
    x_axis /= np.linalg.norm(x_axis)

    # Local Z axis is the normal to the blade
    z_axis = np.cross(x_axis, y_axis)

    m = np.eye(4, dtype=float)
    m[:3, 0] = x_axis
    m[:3, 1] = y_axis
    m[:3, 2] = z_axis
    m[0, 3]  = bx
    m[1, 3]  = by
    m[2, 3]  = bz
    return m
        
# -------------------------        
# MATERIALS creation helpers
# -------------------------
def _make_material(stage, path: str, color: tuple, roughness: float = 0.6, metallic: float = 0.0):
    """
    Crea un materiale UsdPreviewSurface con colore RGB (0-1).
    """
    from pxr import UsdShade
    mat = UsdShade.Material.Define(stage, path)

    shader = UsdShade.Shader.Define(stage, f"{path}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shader.CreateInput("roughness",    Sdf.ValueTypeNames.Float).Set(roughness)
    shader.CreateInput("metallic",     Sdf.ValueTypeNames.Float).Set(metallic)

    mat.CreateSurfaceOutput().ConnectToSource(
        shader.ConnectableAPI(), "surface"
    )
    return mat


def _bind_material(prim, mat):
    from pxr import UsdShade
    UsdShade.MaterialBindingAPI(prim).Bind(mat)


# ----------------------------
# PHYSICS: Rigitd body helpers
# ----------------------------
def _apply_rigid_body(stage, prim_path: str, mass_kg: float):
    """It applies RigidBodyAPI + MassAPI + CollisionAPI to an existing prim."""
    prim = stage.GetPrimAtPath(prim_path)
    UsdPhysics.RigidBodyAPI.Apply(prim)
    mass_api = UsdPhysics.MassAPI.Apply(prim)
    mass_api.GetMassAttr().Set(mass_kg)
    UsdPhysics.CollisionAPI.Apply(prim)

def _make_stem_joint(stage, joint_path: str,
                     body0_path: str, body1_path: str,
                     pivot_z: float, stiffness: float):
    """
    SphericalJoint between body0 (lower internode) and body1 (upper).
    The pivot is at body0's tip = body1's base, in world space Z = pivot_z.
    """
    joint = UsdPhysics.SphericalJoint.Define(stage, joint_path)

    joint.GetBody0Rel().SetTargets([Sdf.Path(body0_path)])
    joint.GetBody1Rel().SetTargets([Sdf.Path(body1_path)])

    # Pivot in body0's local space: tip = (0, 0, +height/2) already centered
    joint.GetLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, pivot_z))
    joint.GetLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))

    # symmetrical angular limits
    cone_api = UsdPhysics.SphericalJoint(joint)
    lim = math.radians(JOINT_MAX_ANGLE_DEG)
    joint.GetConeAngle0LimitAttr().Set(math.degrees(lim))
    joint.GetConeAngle1LimitAttr().Set(math.degrees(lim))

    # Drive (spring + damper) on both axes
    for axis in ("rotX", "rotY"):
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), axis)
        drive.GetTypeAttr().Set("force")
        drive.GetStiffnessAttr().Set(stiffness)
        drive.GetDampingAttr().Set(JOINT_DAMPING)
        drive.GetTargetPositionAttr().Set(0.0)  # it "wants" to stay straight


# ─────────────────────────────────────────────────────────────────────────────
# Main exporter
# ─────────────────────────────────────────────────────────────────────────────

def export_plant_usd(snapshot: PlantSnapshot, output_path: str) -> None:

    # ── Stage setup ──────────────────────────────────────────────────────────
    stage = Usd.Stage.CreateNew(output_path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)  # set axis z for height
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)  # set meters as length unit

    plant_path = f"/World/Plant_{snapshot.plant_id}"
    UsdGeom.Xform.Define(stage, plant_path)

    stem_path  = f"{plant_path}/Stem"
    roots_path = f"{plant_path}/Roots"
    UsdGeom.Xform.Define(stage, stem_path)
    UsdGeom.Xform.Define(stage, roots_path)
    
    # ── Materials ────────────────────────────────────────────────────────────────
    mats_path = f"{plant_path}/Materials"
    UsdGeom.Xform.Define(stage, mats_path)

    mat_stem         = _make_material(stage, f"{mats_path}/Stem",         (0.45, 0.30, 0.10))  # brown
    mat_root         = _make_material(stage, f"{mats_path}/Root",         (0.55, 0.35, 0.15))  # dark brown
    mat_leaf         = _make_material(stage, f"{mats_path}/Leaf",         (0.15, 0.55, 0.10))  # green
    mat_pedicel      = _make_material(stage, f"{mats_path}/Pedicel",      (0.20, 0.50, 0.10))  # dark green
    mat_fruit_ripe   = _make_material(stage, f"{mats_path}/FruitRipe",    (0.90, 0.17, 0.10))  # ripe tomato red (from GroIMP spectra)
    mat_fruit_unripe = _make_material(stage, f"{mats_path}/FruitUnripe",  (0.45, 0.58, 0.25))  # unripe green-yellowish (from GroIMP spectra)
    
    materials = {
        "stem":         mat_stem,
        "root":         mat_root,
        "leaf":         mat_leaf,
        "pedicel":      mat_pedicel,
        "fruit_ripe":   mat_fruit_ripe,
        "fruit_unripe": mat_fruit_unripe,
    }

    # ── Separate organs ──────────────────────────────────────────────────────
    root_node = next((n for n in snapshot.organs if isinstance(n, RootNode)), None)
    
    def get_base_z(n) -> float:
        if n is None or not isinstance(n, InternodeNode): return 0.0
        if hasattr(n, 'world_base_z'): return n.world_base_z
        z = get_base_z(n.parent) + n.parent.length if (n.parent and isinstance(n.parent, InternodeNode)) else 0.0
        n.world_base_z = z
        return z

    for n in snapshot.organs:
        if isinstance(n, InternodeNode):
            get_base_z(n)

    internodes = sorted(
        [n for n in snapshot.organs if isinstance(n, InternodeNode)],
        key=lambda n: (n.key.order, n.key.rank)
    )

    print(f"\n{'='*50}")
    print(f"  Plant {snapshot.plant_id}  |  Day {snapshot.day}")
    print(f"  Internodes found: {len(internodes)}")
    print(f"{'='*50}\n")

    # ── Root sphere ───────────────────────────────────────────────────────────
    # Placed just below z=0 so it sits at ground level
    if root_node:
        print("[ROOT]")
        sph = _make_sphere(
            stage,
            path=f"{roots_path}/Root",
            radius=ROOT_SPHERE_RADIUS,
            cx=0.0, cy=0.0, cz=-ROOT_SPHERE_RADIUS,
        )
        
        _bind_material(sph, materials["root"])

    # ── Internode hierarchical rendering ──────────────────────────────────────
    max_z = 0.0
    for node in internodes:
        L = node.length
        R = node.width_m / 2.0
        rank = node.key.rank
        order = node.key.order
        base_z = getattr(node, 'world_base_z', 0.0)
        max_z = max(max_z, base_z + L)

        print(f"\n[INTERNODE order={order} rank={rank}]")
        cyl = _make_cylinder(
            stage,
            path=f"{stem_path}/Internode_o{order}_r{rank}",
            height=L,
            radius=R,
            base_z=base_z,
        )
        _bind_material(cyl, materials["stem"])
        

    # ── Physics: joint chain ─────────────────────────────────────────────────
    joints_path = f"{plant_path}/Joints"
    UsdGeom.Xform.Define(stage, joints_path)

    # Still rank 0 to ground
    if internodes:
        anchor_path = f"{stem_path}/Internode_o{internodes[0].key.order}_r{internodes[0].key.rank}"
        fixed = UsdPhysics.FixedJoint.Define(stage, f"{joints_path}/GroundAnchor")
        fixed.GetBody1Rel().SetTargets([Sdf.Path(anchor_path)])

    n_ranks = len(internodes)

    for i, node in enumerate(internodes):
        L = node.length
        R = node.width_m / 2.0
        path = f"{stem_path}/Internode_o{node.key.order}_r{node.key.rank}"

        # Mass: cylinder volume * density
        mass = math.pi * R**2 * L * STEM_DENSITY_KG_M3
        _apply_rigid_body(stage, path, mass)

        # Stiffness linearly decreases from bottom to the top
        t = i / max(n_ranks - 1, 1)
        stiffness = JOINT_STIFFNESS_BASE + t * (JOINT_STIFFNESS_TIP - JOINT_STIFFNESS_BASE)

        # Joint with parent
        if node.parent and isinstance(node.parent, InternodeNode):
            prev_path = f"{stem_path}/Internode_o{node.parent.key.order}_r{node.parent.key.rank}"
            _make_stem_joint(
                stage,
                joint_path=f"{joints_path}/Joint_o{node.key.order}_r{node.key.rank}",
                body0_path=prev_path,
                body1_path=path,
                pivot_z=node.parent.length / 2.0,
                stiffness=stiffness,
            )
        
        

    # ── Leaves ───────────────────────────────────────────────────────────────
    leaves = [n for n in snapshot.organs if isinstance(n, LeafNode)]

    leaves_path = f"{plant_path}/Leaves"
    UsdGeom.Xform.Define(stage, leaves_path)

    for node in leaves:
        if node.parent and isinstance(node.parent, InternodeNode):
            tip_z = getattr(node.parent, 'world_base_z', 0.0) + node.parent.length
        else:
            tip_z = 0.0

        leaf_id = f"o{node.key.order}_r{node.key.rank}_i{node.key.organ_index}"
        leaf_group = f"{leaves_path}/Leaf_{leaf_id}"
        UsdGeom.Xform.Define(stage, leaf_group)

        print(f"\n[LEAF order={node.key.order} rank={node.key.rank} idx={node.key.organ_index}] "
                f"attaches at z={tip_z:.4f}m")
        _make_leaf(stage, leaf_group, node, tip_z, materials)
        
        
    # ── Fruits ───────────────────────────────────────────────────────────────────
    # Replicate GroIMP truss structure:
    #   - A main rachis made of INTERNODETRUSSLENGTH segments, tilting by
    #     internodeTrussAngle (9°) between each fruit.
    #   - Lateral pedicels (PETIOLELENGTH) branching off alternately (RU ±90°)
    #     from each rachis node, with a fruit sphere at the tip.
    #   - The last fruit sits at the terminal end of the rachis.

    trusses_path = f"{plant_path}/Trusses"
    UsdGeom.Xform.Define(stage, trusses_path)

    fruits_nodes = sorted(
        [n for n in snapshot.organs if isinstance(n, FruitsNode)],
        key=lambda n: n.key.rank
    )

    RACHIS_SEG   = INTERNODE_TRUSS_LENGTH_M   # 0.012 m — rachis segment length
    PEDICEL_LEN  = PETIOLE_LENGTH_M           # 0.003 m — lateral pedicel length
    PEDICEL_R    = TRUSS_RADIUS               # 0.00075 m
    INITIAL_TILT = 45.0                        # GroIMP: RL(45) at start

    for node in fruits_nodes:
        if node.parent and isinstance(node.parent, InternodeNode):
            attach_z = getattr(node.parent, 'world_base_z', 0.0) + node.parent.length
        else:
            attach_z = 0.0

        truss_az = math.radians((node.key.rank * PHYLLOTAXIS) % 360)
        bend_per_fruit = node.truss_angle   # 9° per segment

        radii = [r for r in node.fruit_radii if r > 1e-5][:node.fruit_nr]
        n_fruits = len(radii)
        if n_fruits == 0:
            continue

        truss_group = f"{trusses_path}/Truss_r{node.key.rank}_i{node.key.organ_index}"
        UsdGeom.Xform.Define(stage, truss_group)

        print(f"\n[TRUSS rank={node.key.rank}] {n_fruits} fruits, attach_z={attach_z:.4f}m")

        # Current tip of the rachis — starts at attachment point on stem
        # Direction: initially tilted INITIAL_TILT° from Z towards the azimuth
        tilt_from_z = math.radians(INITIAL_TILT)
        # Rachis direction vector (in world coords)
        rach_dx = math.sin(tilt_from_z) * math.cos(truss_az)
        rach_dy = math.sin(tilt_from_z) * math.sin(truss_az)
        rach_dz = math.cos(tilt_from_z)

        cur_x, cur_y, cur_z = 0.0, 0.0, attach_z

        # Lateral direction for pedicels: perpendicular to rachis in the
        # horizontal plane. We alternate sign for RU(±90).
        lat_dx = -math.sin(truss_az)
        lat_dy =  math.cos(truss_az)
        lat_dz = 0.0

        seg_idx = 0

        for fi in range(n_fruits):
            r = radii[fi]
            is_last = (fi == n_fruits - 1)

            if fi == 0:
                # First segment: rachis from attachment point
                seg_len = RACHIS_SEG
            elif is_last:
                # Terminal fruit: no rachis segment, just a pedicel from tip
                seg_len = 0.0
            else:
                # Intermediate: rachis continues with a bend
                # Apply cumulative bend (RL in GroIMP = tilt further from Z)
                tilt_from_z += math.radians(bend_per_fruit)
                rach_dx = math.sin(tilt_from_z) * math.cos(truss_az)
                rach_dy = math.sin(tilt_from_z) * math.sin(truss_az)
                rach_dz = math.cos(tilt_from_z)
                seg_len = RACHIS_SEG

            # Draw rachis segment (if any)
            if seg_len > 0:
                seg_cx = cur_x + rach_dx * seg_len / 2.0
                seg_cy = cur_y + rach_dy * seg_len / 2.0
                seg_cz = cur_z + rach_dz * seg_len / 2.0
                seg_mat = _align_z_to(rach_dx, rach_dy, rach_dz, seg_cx, seg_cy, seg_cz)

                seg_prim = UsdGeom.Cylinder.Define(
                    stage, f"{truss_group}/Rachis_{seg_idx}")
                seg_prim.GetHeightAttr().Set(seg_len)
                seg_prim.GetRadiusAttr().Set(PEDICEL_R)
                seg_prim.GetAxisAttr().Set(UsdGeom.Tokens.z)
                _set_transform(seg_prim, seg_mat)
                _bind_material(seg_prim, materials["pedicel"])
                seg_idx += 1

                # Advance tip
                cur_x += rach_dx * seg_len
                cur_y += rach_dy * seg_len
                cur_z += rach_dz * seg_len

            # Pedicel branch to the fruit
            if is_last and n_fruits > 1:
                # Terminal fruit: pedicel continues in rachis direction
                ped_dx, ped_dy, ped_dz = rach_dx, rach_dy, rach_dz
            else:
                # Lateral pedicel: alternate sides (RU ±90°)
                sign = -1.0 if (fi % 2 == 0) else 1.0
                ped_dx = sign * lat_dx
                ped_dy = sign * lat_dy
                ped_dz = sign * lat_dz

            ped_cx = cur_x + ped_dx * PEDICEL_LEN / 2.0
            ped_cy = cur_y + ped_dy * PEDICEL_LEN / 2.0
            ped_cz = cur_z + ped_dz * PEDICEL_LEN / 2.0
            ped_mat = _align_z_to(ped_dx, ped_dy, ped_dz, ped_cx, ped_cy, ped_cz)

            ped_prim = UsdGeom.Cylinder.Define(
                stage, f"{truss_group}/Pedicel_{fi}")
            ped_prim.GetHeightAttr().Set(PEDICEL_LEN)
            ped_prim.GetRadiusAttr().Set(PEDICEL_R * 0.8)
            ped_prim.GetAxisAttr().Set(UsdGeom.Tokens.z)
            _set_transform(ped_prim, ped_mat)
            _bind_material(ped_prim, materials["pedicel"])

            # Fruit sphere at the tip of the pedicel
            fx = cur_x + ped_dx * (PEDICEL_LEN + r)
            fy = cur_y + ped_dy * (PEDICEL_LEN + r)
            fz = cur_z + ped_dz * (PEDICEL_LEN + r)

            sph = _make_sphere(stage, f"{truss_group}/Fruit_{fi}", r, fx, fy, fz)

            # Ripe vs unripe coloring based on fruit age (GroIMP logic)
            age = node.fruit_age_dd[fi] if fi < len(node.fruit_age_dd) else 0.0
            is_ripe = age >= node.ripening_dd
            fruit_mat_key = "fruit_ripe" if is_ripe else "fruit_unripe"
            _bind_material(sph, materials[fruit_mat_key])

            state = "ripe" if is_ripe else "unripe"
            print(f"  [Fruit {fi}] r={r:.4f}m {state} (age={age:.0f}/{node.ripening_dd:.0f}dd) at ({fx:.4f},{fy:.4f},{fz:.4f})")

    print(f"\n  Max stem height: {max_z:.6f} m\n")

    # ── Save ─────────────────────────────────────────────────────────────────
    stage.GetRootLayer().Save()
    print(f"[USD] Saved → {output_path}")