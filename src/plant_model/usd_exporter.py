import math
import numpy as np
from pxr import Usd, UsdGeom, Gf, Sdf

from .models import (
    PlantSnapshot, OrganNode, InternodeNode, RootNode, LeafNode, FruitsNode
)

from .constants import (
    INTERNODE_TRUSS_LENGTH_M,
    INTERNODE_TRUSS_DIAMETER_M,
    ANGLE_AMONG_SUBSEQUENT_FRUITS_DEG,
    FRUIT_PAIRING, ROOT_SPHERE_RADIUS,
    TRUSS_LENGTH, TRUSS_RADIUS, PHYLLOTAXIS
)

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
    Leaf = Petiole cylinder + Rachis cylinder + N blade quads.
    All geometry is in the XZ plane rotated by ccw_orientation around Z.
    tip_z: world Z where the leaf attaches (top of parent internode).
    """
    import math

    az  = math.radians(node.ccw_orientation)   # azimuth around stem
    el  = math.radians(node.angle_petiole)     # elevation from horizontal (90°=horiz)

    # Direction unit vector of petiole in world space
    # angle_petiole=90° → horizontal → dir = (cos(az), sin(az), 0)
    # angle_petiole=0°  → vertical   → dir = (0, 0, 1)
    tilt = math.radians(90.0 - node.angle_petiole)  # tilt from horizontal
    dx = math.cos(az) * math.cos(tilt)
    dy = math.sin(az) * math.cos(tilt)
    dz = math.sin(tilt)

    Lp = max(node.length_petiole, 1e-4)
    Rp = max(node.diameter_petiole / 2.0, 1e-4)

    # ── Petiole ──────────────────────────────────────────────────────────────
    # Centre of petiole = attachment + dir * Lp/2
    pcx = dx * Lp / 2.0
    pcy = dy * Lp / 2.0
    pcz = tip_z + dz * Lp / 2.0

    # Rotation matrix to align cylinder Z-axis to (dx, dy, dz)
    petiole_mat = _align_z_to(dx, dy, dz, pcx, pcy, pcz)

    pet = UsdGeom.Cylinder.Define(stage, f"{leaf_group}/Petiole")
    pet.GetHeightAttr().Set(Lp)
    pet.GetRadiusAttr().Set(Rp)
    pet.GetAxisAttr().Set(UsdGeom.Tokens.z)
    _set_transform(pet, petiole_mat)

    print(f"  [Petiole] az={node.ccw_orientation}° el={node.angle_petiole}° "
          f"L={Lp:.4f}m centre=({pcx:.4f},{pcy:.4f},{pcz:.4f})")

    # Tip of petiole = start of rachis
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

    # ── Blades ───────────────────────────────────────────────────────────────
    n = max(node.blades_nr, 1)
    blade_area   = node.area_blades_total / n if node.area_blades_total > 0 else 4e-4
    blade_length = math.sqrt(blade_area / 0.6)
    blade_width  = blade_length * 0.6

    # Perpendicular vector to petiole dir, in horizontal plane → blade normal
    perp_x = -math.sin(az)
    perp_y =  math.cos(az)
    perp_z =  0.0

    for i in range(n):
        if n == 1:
            # Single terminal blade at rachis tip
            bx = rtx + dx * Lr
            by = rty + dy * Lr
            bz = rtz + dz * Lr
            insertion_angle = 30.0   # slight droop
        else:
            # Distribute pairs along rachis; last one is terminal
            t = i / (n - 1)         # 0.0 → 1.0 along rachis
            bx = rtx + dx * Lr * t
            by = rty + dy * Lr * t
            bz = rtz + dz * Lr * t
            insertion_angle = 45.0 if i < n - 1 else 20.0

        # Graphical orientation of Terminal Leaf
        is_terminal = (i == n - 1)   # ← True per l'ultima blade

        if is_terminal:
            # Big terminal leaf: coplanar to rachidid, pivot on edge
            rachis_dir = np.array([dx, dy, dz])
            world_up   = np.array([0.0, 0.0, 1.0])

            # world_up component orthogonal to rachidis → set blad "upwards"
            blade_z = world_up - np.dot(world_up, rachis_dir) * rachis_dir
            if np.linalg.norm(blade_z) < 1e-9:
                blade_z = np.array([1.0, 0.0, 0.0])
            blade_z /= np.linalg.norm(blade_z)

            up = np.array([1.0, 0.0, 0.0]) if abs(blade_z[2]) > 0.999 else np.array([0.0, 0.0, 1.0])
            bx_ax = np.cross(up, blade_z); bx_ax /= np.linalg.norm(bx_ax)
            by_ax = np.cross(blade_z, bx_ax)

            blade_mat = np.eye(4, dtype=float)
            blade_mat[:3, 0] = bx_ax
            blade_mat[:3, 1] = by_ax
            blade_mat[:3, 2] = blade_z
            blade_mat[0, 3]  = bx
            blade_mat[1, 3]  = by
            blade_mat[2, 3]  = bz

        else:
            # Lateral Blade — use _blade_transform with pivot on border
            side = 1.0 if i % 2 == 0 else -1.0
            insertion_angle = 45.0
            blade_mat = _blade_transform(
                bx, by, bz,
                dx, dy, dz,
                perp_x * side, perp_y * side, 0.0,
                insertion_angle,
                blade_width, blade_length
            )
        

        mesh = UsdGeom.Mesh.Define(stage, f"{leaf_group}/Blade_{i}")
        hw, hl = blade_width / 2.0, blade_length / 2.0
        mesh.GetPointsAttr().Set([
            Gf.Vec3f(-hw, -hl, 0), Gf.Vec3f(hw, -hl, 0),
            Gf.Vec3f(hw,  hl, 0), Gf.Vec3f(-hw,  hl, 0),
        ])
        mesh.GetFaceVertexCountsAttr().Set([3, 3])
        mesh.GetFaceVertexIndicesAttr().Set([0, 1, 2, 0, 2, 3])
        mesh.GetSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
        _set_transform(mesh, blade_mat)
        
        _bind_material(pet, materials["leaf"])   # petiole
        _bind_material(rac, materials["leaf"])   # rachis
        _bind_material(mesh, materials["leaf"])  # blade

        print(f"  [Blade {i}] pos=({bx:.4f},{by:.4f},{bz:.4f}) "
              f"size={blade_width:.4f}x{blade_length:.4f}m")


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
                     insertion_deg: float,
                     width: float, height: float) -> np.ndarray:
    """
    Transform for a leaf blade quad.
    (bx,by,bz)     = attachment point on rachis
    (ax,ay,az_)    = rachis direction
    (px,py,pz)     = lateral direction (left or right of rachis)
    insertion_deg  = angle of blade from rachis axis
    """
    import math
    ins = math.radians(insertion_deg)

    # Blade normal = rachis_dir rotated toward lateral by insertion_deg
    a = np.array([ax, ay, az_], dtype=float)
    p = np.array([px, py, pz],  dtype=float)
    if np.linalg.norm(p) < 1e-9:
        p = np.array([1.0, 0.0, 0.0])
    else:
        p /= np.linalg.norm(p)

    blade_z = -a * math.cos(ins) + p * math.sin(ins) # REMOVE - if you want to mirror the leaf along the rachidis
    blade_z /= np.linalg.norm(blade_z)

    up = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(blade_z, up)) > 0.999:
        up = np.array([1.0, 0.0, 0.0])
    bx_ax = np.cross(up, blade_z); bx_ax /= np.linalg.norm(bx_ax)
    by_ax = np.cross(blade_z, bx_ax)


    # Graphical fix: translation make leaves to be tangent to the rachidis (otherwise it will intercept in the midle of the leaf)
    cx = bx + blade_z[0] * height / 2.0
    cy = by + blade_z[1] * height / 2.0
    cz = bz + blade_z[2] * height / 2.0
    

    m = np.eye(4, dtype=float)
    m[:3, 0] = bx_ax
    m[:3, 1] = by_ax
    m[:3, 2] = blade_z
    m[0, 3]  = cx
    m[1, 3]  = cy
    m[2, 3]  = cz
    return m
        
        
# Material creation helpers
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
    internodes = sorted(
        [n for n in snapshot.organs if isinstance(n, InternodeNode)],
        key=lambda n: n.key.rank   # rank 0 → 1 → 2 ... bottom to top
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

    # ── Internode chain — each starts where the previous ended ────────────────
    current_z = 0.0   # base of first internode = ground level z=0

    for node in internodes:
        L = node.length
        R = node.width_m / 2.0
        rank = node.key.rank

        print(f"\n[INTERNODE rank={rank}]")
        cyl = _make_cylinder(
            stage,
            path=f"{stem_path}/Internode_r{rank}",
            height=L,
            radius=R,
            base_z=current_z,
        )
        current_z += L   # tip of this internode = base of next
        
        _bind_material(cyl, materials["stem"])

    # ── Leaves ───────────────────────────────────────────────────────────────
    leaves = [n for n in snapshot.organs if isinstance(n, LeafNode)
                and n.key.order == 0]

    # Build rank→tip_z map from the internode chain
    rank_tip_z = {}
    z = 0.0
    for node in internodes:          # already sorted by rank
        z += node.length
        rank_tip_z[node.key.rank] = z

    leaves_path = f"{plant_path}/Leaves"
    UsdGeom.Xform.Define(stage, leaves_path)

    for node in leaves:
        tip_z = rank_tip_z.get(node.key.rank, 0.0)
        leaf_id = f"r{node.key.rank}_i{node.key.organ_index}"
        leaf_group = f"{leaves_path}/Leaf_{leaf_id}"
        UsdGeom.Xform.Define(stage, leaf_group)

        print(f"\n[LEAF rank={node.key.rank} idx={node.key.organ_index}] "
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
        tip_z    = rank_tip_z.get(node.key.rank, 0.0)
        truss_az = math.radians((node.key.rank * PHYLLOTAXIS) % 360)
        tilt     = math.radians(node.truss_angle)   # from stem (Z), small angle

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

    print(f"\n  Total stem height: {current_z:.6f} m\n")

    # ── Save ─────────────────────────────────────────────────────────────────
    stage.GetRootLayer().Save()
    print(f"[USD] Saved → {output_path}")