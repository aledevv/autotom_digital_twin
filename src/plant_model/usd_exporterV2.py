"""
usd_exporterV2.py

Articulated-stem exporter for the tomato plant digital twin.
Phase 1: main stem only (internodes), no leaves/fruits/roots.

Key design decisions vs V1:
- Link pattern from generate_articulation_usda.py:
    /Stem/Link_NN  (Xform + RigidBodyAPI + MassAPI)
    /Stem/Link_NN/Cylinder  (UsdGeom.Cylinder + CollisionAPI)
  This avoids the nested-cylinder issues of V1.
- D6 joints (UsdPhysics.Joint) with translational locks + rotXY spring drives.
  Tested and stable in IsaacSim (generate_articulation_usda test).
- Internodes longer than STEM_SEGMENT_MAX_LENGTH_M are automatically split
  into sub-segments so each PhysX link stays short and well-conditioned.
- PhysX scene/articulation config is intentionally NOT included here;
  it is applied by the loader (load_stem_v2.py) inside SimulationApp,
  following the same separation of concerns as load_articulation_subbranch.py.
"""

import math
import os
from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf

from .models import PlantSnapshot, InternodeNode, RootNode
from .constants import (
    STEM_DENSITY_KG_M3,
    STEM_SEGMENT_MAX_LENGTH_M,
    STEM_V2_STIFFNESS_BASE,
    STEM_V2_STIFFNESS_TIP,
    STEM_V2_DAMPING,
    STEM_V2_BEND_LIMIT_DEG,
    STEM_V2_GAP,
)
from .usd_helpers import _make_material, _bind_material

# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _compute_world_base_z(node: InternodeNode) -> float:
    if hasattr(node, 'world_base_z'):
        return node.world_base_z
    if node.parent is None or not isinstance(node.parent, InternodeNode):
        node.world_base_z = 0.0
    else:
        node.world_base_z = _compute_world_base_z(node.parent) + node.parent.length
    return node.world_base_z


def _subdivide_internode(node: InternodeNode, max_seg_len: float) -> list[dict]:
    """
    Splits one InternodeNode into N uniform sub-segments if its length
    exceeds max_seg_len.  Returns a list of dicts:
        {'length': float, 'radius': float, 'base_z': float}
    """
    total_len = node.length
    radius = node.width_m / 2.0
    base_z = _compute_world_base_z(node)

    n_segs = max(1, math.ceil(total_len / max_seg_len))
    seg_len = total_len / n_segs

    return [
        {
            'length': seg_len,
            'radius': radius,
            'base_z': base_z + i * seg_len,
        }
        for i in range(n_segs)
    ]


def _link_mass(length: float, radius: float) -> float:
    return math.pi * radius ** 2 * length * STEM_DENSITY_KG_M3


def _create_stem_link(stage, parent_path: str, link_idx: int,
                      seg: dict, mat_stem) -> str:
    """
    Creates one articulation link following the generate_articulation_usda pattern:
      <parent_path>/Link_<NNN>          Xform + RigidBodyAPI + MassAPI
      <parent_path>/Link_<NNN>/Cylinder  Cylinder + CollisionAPI
    The Xform origin sits at the BASE of the segment (z = seg['base_z']).
    The Cylinder is offset by +length/2 so its base aligns with the Xform origin.
    """
    link_path = f"{parent_path}/Link_{link_idx:04d}"
    length = seg['length']
    radius = seg['radius']
    base_z = seg['base_z']

    # Xform at base of segment
    xform = UsdGeom.Xform.Define(stage, link_path)
    xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, base_z))

    UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())
    mass_api = UsdPhysics.MassAPI.Apply(xform.GetPrim())
    mass_api.CreateMassAttr().Set(_link_mass(length, radius))

    # Cylinder child — local offset so base is at Xform origin
    cyl_path = f"{link_path}/Cylinder"
    cyl = UsdGeom.Cylinder.Define(stage, cyl_path)
    cyl.GetHeightAttr().Set(length)
    cyl.GetRadiusAttr().Set(radius)
    cyl.GetAxisAttr().Set(UsdGeom.Tokens.z)
    cyl.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, length / 2.0))
    UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())
    _bind_material(cyl, mat_stem)

    print(f"  [Link {link_idx:04d}] base_z={base_z:.4f}m  L={length:.4f}m  R={radius:.4f}m")
    return link_path


def _anchor_to_world(stage, link_path: str) -> None:
    """Fixed joint anchoring the first link to the world."""
    joint = UsdPhysics.FixedJoint.Define(stage, f"{link_path}/GroundAnchor")
    joint.CreateBody1Rel().SetTargets([Sdf.Path(link_path)])


def _create_d6_joint(stage, joint_path: str,
                     body0_path: str, body1_path: str,
                     seg0_length: float, stiffness: float) -> None:
    """
    D6 joint (same as generate_articulation_usda):
    - Translations: locked
    - rotX, rotY: spring drive with bend limit
    - rotZ: locked (no twist — tomato stem doesn't twist at joints)
    Pivot0 sits at the tip of body0 (local Z = +seg0_length + GAP).
    Pivot1 sits at the base of body1 (local Z = 0 because Xform origin IS the base).
    """
    joint = UsdPhysics.Joint.Define(stage, joint_path)
    joint.CreateBody0Rel().SetTargets([Sdf.Path(body0_path)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(body1_path)])

    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, seg0_length + STEM_V2_GAP))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

    # Lock translations
    for axis in ("transX", "transY", "transZ"):
        lim = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), axis)
        lim.CreateLowAttr().Set(1.0)
        lim.CreateHighAttr().Set(-1.0)

    # Swing drives (rotX, rotY) with angular limit
    for axis in ("rotX", "rotY"):
        lim = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), axis)
        lim.CreateLowAttr().Set(-STEM_V2_BEND_LIMIT_DEG)
        lim.CreateHighAttr().Set(STEM_V2_BEND_LIMIT_DEG)
        drv = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), axis)
        drv.CreateTypeAttr().Set("force")
        drv.CreateStiffnessAttr().Set(stiffness)
        drv.CreateDampingAttr().Set(STEM_V2_DAMPING)
        drv.CreateTargetPositionAttr().Set(0.0)

    # Lock twist (rotZ)
    lim_z = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), "rotZ")
    lim_z.CreateLowAttr().Set(1.0)
    lim_z.CreateHighAttr().Set(-1.0)


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def export_stem_articulated_usd(snapshot: PlantSnapshot, output_path: str) -> None:
    """
    Builds a USD stage with ONLY the articulated main stem.
    Leaves, fruits, and roots are intentionally skipped (Phase 1).

    The stage is pure OpenUSD (no PhysxSchema calls).
    PhysX scene + articulation settings must be applied by the loader
    (load_stem_v2.py) inside SimulationApp, exactly as in the subbranch test.
    """

    # ── Stage setup ──────────────────────────────────────────────────────────
    stage = Usd.Stage.CreateNew(output_path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    world_path = "/World"
    world_prim = UsdGeom.Xform.Define(stage, world_path)
    stage.SetDefaultPrim(world_prim.GetPrim())

    stem_path = f"{world_path}/Stem"
    stem_prim = UsdGeom.Xform.Define(stage, stem_path)
    UsdPhysics.ArticulationRootAPI.Apply(stem_prim.GetPrim())

    # ── Material ─────────────────────────────────────────────────────────────
    mats_path = f"{world_path}/Materials"
    UsdGeom.Xform.Define(stage, mats_path)
    mat_stem = _make_material(stage, f"{mats_path}/Stem", (0.45, 0.30, 0.10))

    # ── Collect & sort internodes ─────────────────────────────────────────────
    internodes = sorted(
        [n for n in snapshot.organs if isinstance(n, InternodeNode)],
        key=lambda n: (n.key.order, n.key.rank),
    )

    if not internodes:
        print("[WARN] No internodes found in snapshot. Saving empty stage.")
        stage.GetRootLayer().Save()
        return

    # Cache world_base_z on all nodes before subdividing
    for n in internodes:
        _compute_world_base_z(n)

    print(f"\n{'='*55}")
    print(f"  Plant {snapshot.plant_id} | Day {snapshot.day}")
    print(f"  Internodes: {len(internodes)}")
    print(f"{'='*55}\n")

    # ── Build flat list of physical segments ──────────────────────────────────
    # Each InternodeNode may expand into multiple sub-segments.
    all_segments = []
    for node in internodes:
        all_segments.extend(_subdivide_internode(node, STEM_SEGMENT_MAX_LENGTH_M))

    n_links = len(all_segments)
    print(f"  Total articulation links (after subdivision): {n_links}\n")

    # ── Create links and joints ───────────────────────────────────────────────
    link_paths = []
    for i, seg in enumerate(all_segments):
        t = i / max(n_links - 1, 1)
        # Stiffness interpolation: base (stiff) → tip (flexible)
        stiffness = STEM_V2_STIFFNESS_BASE + t * (STEM_V2_STIFFNESS_TIP - STEM_V2_STIFFNESS_BASE)

        link_path = _create_stem_link(stage, stem_path, i, seg, mat_stem)
        link_paths.append((link_path, seg['length'], stiffness))

        if i == 0:
            _anchor_to_world(stage, link_path)
        else:
            prev_path, prev_len, _ = link_paths[i - 1]
            joint_path = f"{stem_path}/Joint_{i-1:04d}_{i:04d}"
            _create_d6_joint(
                stage=stage,
                joint_path=joint_path,
                body0_path=prev_path,
                body1_path=link_path,
                seg0_length=prev_len,
                stiffness=stiffness,
            )

    total_height = all_segments[-1]['base_z'] + all_segments[-1]['length']
    print(f"\n  Max stem height: {total_height:.4f} m")
    print(f"  Links created:  {n_links}")

    # ── Save ──────────────────────────────────────────────────────────────────
    stage.GetRootLayer().Save()
    print(f"\n[USD] Saved → {output_path}")