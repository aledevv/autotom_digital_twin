from pathlib import Path
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

    absolute_azimuth = getattr(node, 'world_azimuth', 0.0) + node.ccw_orientation
    az  = math.radians(absolute_azimuth)   # absolute azimuth around stem
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

    print(f"  [Petiole] az={node.ccw_orientation}° el={node.angle_petiole}° "
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
    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(path))
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

    mat_stem    = _make_material(stage, f"{mats_path}/Stem",       (0.45, 0.30, 0.10))  # brown
    mat_root    = _make_material(stage, f"{mats_path}/Root",       (0.55, 0.35, 0.15))  # dark brown
    mat_leaf    = _make_material(stage, f"{mats_path}/Leaf",       (0.15, 0.55, 0.10))  # green
    mat_pedicel = _make_material(stage, f"{mats_path}/Pedicel",    (0.20, 0.50, 0.10))  # dark green
    mat_fruit   = _make_material(stage, f"{mats_path}/Fruit",      (0.90, 0.15, 0.05))  # tomato red
    
    materials = {
        "stem":    mat_stem,
        "root":    mat_root,
        "leaf":    mat_leaf,
        "pedicel": mat_pedicel,
        "fruit":   mat_fruit,
    }

    # ── Separate organs ──────────────────────────────────────────────────────
    root_node = next((n for n in snapshot.organs if isinstance(n, RootNode)), None)
    
    def get_base_z(n) -> float:
        if n is None or not isinstance(n, InternodeNode): return 0.0
        if hasattr(n, 'world_base_z'): return n.world_base_z
        z = get_base_z(n.parent) + n.parent.length if (n.parent and isinstance(n.parent, InternodeNode)) else 0.0
        n.world_base_z = z
        return z

    def get_azimuth(n) -> float:
        if n is None or not isinstance(n, InternodeNode): return 0.0
        if hasattr(n, 'world_azimuth'): return n.world_azimuth
        if n.key.order == 0:
            az = (n.key.rank * PHYLLOTAXIS) % 360.0
        else:
            p_az = get_azimuth(n.parent)
            sign = 1 if (n.key.rank % 2 == 0) else -1
            az = (p_az + sign * 90.0) % 360.0
        n.world_azimuth = az
        return az

    for n in snapshot.organs:
        if isinstance(n, InternodeNode):
            get_base_z(n)
            get_azimuth(n)

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
            node.world_azimuth = getattr(node.parent, 'world_azimuth', 0.0)
        else:
            tip_z = 0.0

        leaf_id = f"o{node.key.order}_r{node.key.rank}_i{node.key.organ_index}"
        leaf_group = f"{leaves_path}/Leaf_{leaf_id}"
        UsdGeom.Xform.Define(stage, leaf_group)

        print(f"\n[LEAF order={node.key.order} rank={node.key.rank} idx={node.key.organ_index}] "
                f"attaches at z={tip_z:.4f}m")
        _make_leaf(stage, leaf_group, node, tip_z, materials)
        
        
    # ── Fruits ───────────────────────────────────────────────────────────────────

    trusses_path = f"{plant_path}/Trusses"
    UsdGeom.Xform.Define(stage, trusses_path)

    fruits_nodes = sorted(
        [n for n in snapshot.organs if isinstance(n, FruitsNode)],
        key=lambda n: n.key.rank
    )

    for node in fruits_nodes:
        if node.parent and isinstance(node.parent, InternodeNode):
            tip_z = getattr(node.parent, 'world_base_z', 0.0) + node.parent.length
            truss_az = math.radians(getattr(node.parent, 'world_azimuth', 0.0))
        else:
            tip_z = 0.0
            truss_az = math.radians((node.key.rank * PHYLLOTAXIS) % 360)
        tilt = math.radians(90 - node.truss_angle)   # from stem (Z), small angle

        #Pedicel orientation: vertical part (+Z), rotated by tild towards azimut
        pdx = math.sin(tilt) * math.cos(truss_az)
        pdy = math.sin(tilt) * math.sin(truss_az)
        pdz = math.cos(tilt)

        radii = [r for r in node.fruit_radii if r > 1e-5][:node.fruit_nr]

        # Pedicel length = sum of fruit diameter + GAP offset
        GAP = 0.001
        pedicel_length = sum(r * 2 for r in radii) + GAP * (len(radii) + 1)
        pedicel_length = max(pedicel_length, TRUSS_LENGTH)  # minimo fisso

        truss_group = f"{trusses_path}/Truss_r{node.key.rank}_i{node.key.organ_index}"
        UsdGeom.Xform.Define(stage, truss_group)

        # Pedicel: center at tip_z + dir * TRUSS_LENGTH/2
        pcx = pdx * pedicel_length / 2.0
        pcy = pdy * pedicel_length / 2.0
        pcz = tip_z + pdz * pedicel_length / 2.0
        pedicel_mat = _align_z_to(pdx, pdy, pdz, pcx, pcy, pcz)

        ped = UsdGeom.Cylinder.Define(stage, f"{truss_group}/Pedicel")
        ped.GetHeightAttr().Set(pedicel_length)
        ped.GetRadiusAttr().Set(TRUSS_RADIUS)
        ped.GetAxisAttr().Set(UsdGeom.Tokens.z)
        _set_transform(ped, pedicel_mat)
        
        _bind_material(ped, materials["pedicel"])

        offset = GAP
        for fi, r in enumerate(radii):
            offset += r   # center first sphere in r from pedicel's tip
            fx = pdx * offset
            fy = pdy * offset
            fz = tip_z + pdz * offset

            sph = _make_sphere(stage, f"{truss_group}/Fruit_{fi}", r, fx, fy, fz)
            _bind_material(sph, materials["fruit"])

            print(f"  [Fruit {fi}] r={r:.4f}m centre=({fx:.4f},{fy:.4f},{fz:.4f})")
            offset += r + GAP   # gap between fruits

    print(f"\n  Max stem height: {max_z:.6f} m\n")

    # ── Save ─────────────────────────────────────────────────────────────────
    stage.GetRootLayer().Save()
    print(f"[USD] Saved → {output_path}")