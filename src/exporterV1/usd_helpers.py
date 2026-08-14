import math
import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf

from .models import LeafNode
from .constants import (
    PHYLLOTAXIS, JOINT_MAX_ANGLE_DEG, JOINT_DAMPING
)

# Optional override for lateral leaflet insertion angle (JUST FOR NICER LOOK, NOT FOR PHYSICS OR KINEMATICS)
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


# TRYING TO MAKE A MORE REALISTIC LEAF MESH
def _set_leaf_mesh_geometry(mesh, hw: float, L: float):
    """
    Sets the points and faces for a 16-point smooth leaf blade mesh.
    Creates a rounded ovate shape typical for tomato leaflets.
    """
    import math
    points = [Gf.Vec3f(0.0, 0.0, 0.0)]  # 0: base (attachment)
    
    n_side = 8
    # Right side (CCW: base to tip)
    for i in range(1, n_side):
        t = i / n_side
        y = L * t
        # shape profile: sin(t * pi) gives a symmetric oval. 
        # multiplying by (1.2 - 0.4*t) makes it slightly wider at the bottom (ovate)
        x = hw * math.sin(math.pi * t) * (1.2 - 0.4 * t)
        points.append(Gf.Vec3f(x, y, 0.0))
        
    points.append(Gf.Vec3f(0.0, L, 0.0))  # Tip
    
    # Left side (CCW: tip back to base)
    for i in range(n_side - 1, 0, -1):
        t = i / n_side
        y = L * t
        x = hw * math.sin(math.pi * t) * (1.2 - 0.4 * t)
        points.append(Gf.Vec3f(-x, y, 0.0))

    mesh.GetPointsAttr().Set(points)
    
    # Triangulate as a fan from the base (point 0)
    num_triangles = len(points) - 2
    mesh.GetFaceVertexCountsAttr().Set([3] * num_triangles)
    
    indices = []
    for i in range(1, len(points) - 1):
        indices.extend([0, i, i + 1])
        
    mesh.GetFaceVertexIndicesAttr().Set(indices)
    mesh.GetSubdivisionSchemeAttr().Set("none")


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
            _set_leaf_mesh_geometry(mesh, hw, L)
            _set_transform(mesh, blade_mat)
            _bind_material(mesh, materials["leaf"])
            
        # Advance distance for the next pair (or terminal leaflet)
        seg_len = seg_len_array[j] if j < len(seg_len_array) else (Lr / max(pairs, 1))
        current_dist += seg_len

    # 2. Terminal leaflet
    # Positioned exactly at the tip of the rachis cylinder
    bx = rtx + dx * Lr
    by = rty + dy * Lr
    bz = rtz + dz * Lr

    blade_mat = _blade_transform(
        bx, by, bz,
        dx, dy, dz,
        perp_x, perp_y, 0.0,
        0.0  # 0 degree insertion = straight ahead
    )

    # Draw terminal petiolule
    y_axis = blade_mat[:3, 1]
    petiolule_len = 0.01  # 1 cm
    
    pcx_t = bx + y_axis[0] * petiolule_len / 2.0
    pcy_t = by + y_axis[1] * petiolule_len / 2.0
    pcz_t = bz + y_axis[2] * petiolule_len / 2.0
    
    pet_mat = _align_z_to(y_axis[0], y_axis[1], y_axis[2], pcx_t, pcy_t, pcz_t)
    
    petl_t = UsdGeom.Cylinder.Define(stage, f"{leaf_group}/Petiolule_term")
    petl_t.GetHeightAttr().Set(petiolule_len)
    petl_t.GetRadiusAttr().Set(Rp * 0.4)
    petl_t.GetAxisAttr().Set(UsdGeom.Tokens.z)
    _set_transform(petl_t, pet_mat)
    _bind_material(petl_t, materials["leaf"])

    # Shift the blade base to start AFTER the petiolule
    blade_mat[0, 3] += y_axis[0] * petiolule_len
    blade_mat[1, 3] += y_axis[1] * petiolule_len
    blade_mat[2, 3] += y_axis[2] * petiolule_len

    mesh = UsdGeom.Mesh.Define(stage, f"{leaf_group}/Blade_{mesh_idx}")
    hw, L = terminal_width / 2.0, terminal_length
    _set_leaf_mesh_geometry(mesh, hw, L)
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
# PHYSICS: Rigid body helpers
# ----------------------------
def _apply_rigid_body(stage, prim_path: str, mass_kg: float, kinematic: bool = False):
    """It applies RigidBodyAPI + MassAPI + CollisionAPI to an existing prim."""
    prim = stage.GetPrimAtPath(prim_path)
    rigid_api = UsdPhysics.RigidBodyAPI.Apply(prim)
    if kinematic:
        rigid_api.CreateKinematicEnabledAttr().Set(True)
    mass_api = UsdPhysics.MassAPI.Apply(prim)
    mass_api.GetMassAttr().Set(mass_kg)
    UsdPhysics.CollisionAPI.Apply(prim)

def _make_stem_joint(stage, joint_path: str,
                     body0_path: str, body1_path: str,
                     pivot0_z: float, pivot1_z: float,
                     stiffness: float):
    """
    SphericalJoint between body0 (lower internode) and body1 (upper).
    pivot0_z: local Z in body0 frame (positive = towards tip).
    pivot1_z: local Z in body1 frame (negative = towards base).
    """
    joint = UsdPhysics.SphericalJoint.Define(stage, joint_path)

    joint.GetBody0Rel().SetTargets([Sdf.Path(body0_path)])
    joint.GetBody1Rel().SetTargets([Sdf.Path(body1_path)])

    # Pivot at body0's tip and body1's base (both in their own local frames)
    joint.GetLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, pivot0_z))
    joint.GetLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, pivot1_z))

    # Symmetrical angular limits
    joint.GetConeAngle0LimitAttr().Set(JOINT_MAX_ANGLE_DEG)
    joint.GetConeAngle1LimitAttr().Set(JOINT_MAX_ANGLE_DEG)

    # Drive (spring + damper) on both axes
    for axis in ("rotX", "rotY"):
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), axis)
        drive.GetTypeAttr().Set("force")
        drive.GetStiffnessAttr().Set(stiffness)
        drive.GetDampingAttr().Set(JOINT_DAMPING)
        drive.GetTargetPositionAttr().Set(0.0)  # it "wants" to stay straight

    # ── Collision filter: prevent body0 ↔ body1 from colliding ──
    prim0 = stage.GetPrimAtPath(body0_path)
    prim1 = stage.GetPrimAtPath(body1_path)
    filt0 = UsdPhysics.FilteredPairsAPI.Apply(prim0) if prim0 else None
    if filt0:
        rel = filt0.GetFilteredPairsRel()
        targets = list(rel.GetTargets())
        targets.append(Sdf.Path(body1_path))
        rel.SetTargets(targets)


# ----------------------------
# PHYSICS: Leaf helpers
# ----------------------------
def _apply_rigid_body_to_leaf(stage, leaf_path: str, mass_kg: float):
    """Applies RigidBodyAPI to the entire leaf group and CollisionAPI to child geometries."""
    leaf_prim = stage.GetPrimAtPath(leaf_path)
    UsdPhysics.RigidBodyAPI.Apply(leaf_prim)
    mass_api = UsdPhysics.MassAPI.Apply(leaf_prim)
    mass_api.GetMassAttr().Set(mass_kg)
    
    # Apply CollisionAPI to descendant geometries
    for child in leaf_prim.GetChildren():
        if child.IsA(UsdGeom.Cylinder):
            UsdPhysics.CollisionAPI.Apply(child)
        elif child.IsA(UsdGeom.Mesh):
            UsdPhysics.CollisionAPI.Apply(child)
            UsdPhysics.MeshCollisionAPI.Apply(child).GetApproximationAttr().Set("convexHull")

def _make_leaf_joint(stage, joint_path: str,
                     body0_path: str, body1_path: str,
                     pivot0_z: float, pivot1_world: Gf.Vec3f,
                     stiffness: float, damping: float):
    """
    SphericalJoint connecting a leaf group (body1) to an internode (body0).
    pivot1_world is the local position for body1 because body1 (leaf group) has identity transform.
    """
    joint = UsdPhysics.SphericalJoint.Define(stage, joint_path)

    joint.GetBody0Rel().SetTargets([Sdf.Path(body0_path)])
    joint.GetBody1Rel().SetTargets([Sdf.Path(body1_path)])

    joint.GetLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, pivot0_z))
    joint.GetLocalPos1Attr().Set(pivot1_world)

    cone_api = UsdPhysics.SphericalJoint(joint)
    lim = math.radians(45.0)  # Leaves can oscillate up to 45 degrees
    joint.GetConeAngle0LimitAttr().Set(math.degrees(lim))
    joint.GetConeAngle1LimitAttr().Set(math.degrees(lim))

    # Spring/damper to return to 0
    for axis in ("rotX", "rotY", "rotZ"):
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), axis)
        drive.GetTypeAttr().Set("force")
        drive.GetStiffnessAttr().Set(stiffness)
        drive.GetDampingAttr().Set(damping)
        drive.GetTargetPositionAttr().Set(0.0)

