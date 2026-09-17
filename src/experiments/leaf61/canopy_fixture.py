"""Frozen full-plant visual context for the incremental 0/1/5 leaf experiment."""

import hashlib

import numpy as np
from plant_model import verify_layout, verify_layout_hulls, yaw_matrix


def freeze(stage, root):
    """Remove all physics from a copied subtree, preserving authored visual poses."""
    from pxr import Sdf, Usd

    removed = 0
    for prim in list(Usd.PrimRange(stage.GetPrimAtPath(root))):
        if prim.GetTypeName().startswith("Physics"):
            stage.RemovePrim(prim.GetPath())
            removed += 1
            continue
        schemas = prim.GetAppliedSchemas()
        prim.SetMetadata(
            "apiSchemas",
            Sdf.TokenListOp.CreateExplicit(
                [s for s in schemas if not s.startswith(("Physics", "Physx"))]
            ),
        )
        for prop in list(prim.GetProperties()):
            if prop.GetName().startswith(("physics:", "physx")):
                prim.RemoveProperty(prop.GetName())
    return removed


def build_canopy(world, a, c, points, faces):
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, Vt
    from scene import build

    stage = world.stage
    source = a.run_dir / "input_canopy.usda"
    original = Usd.Stage.Open(str(source))
    if (
        UsdGeom.GetStageMetersPerUnit(original) != 1
        or UsdGeom.GetStageUpAxis(original) != "Z"
    ):
        raise ValueError("Canopy source must use metres and Z up")
    UsdGeom.Xform.Define(stage, "/World")
    Sdf.CopySpec(original.Flatten(), "/World", stage.GetRootLayer(), "/World/Canopy")
    removed = freeze(stage, "/World/Canopy")
    cache = UsdGeom.XformCache()
    leaves = []
    for prim in Usd.PrimRange(stage.GetPrimAtPath("/World/Canopy")):
        if prim.GetName() != "LeafBlade":
            continue
        base = prim.GetAttribute("autotom:leafShapeBase").Get()
        tip = prim.GetAttribute("autotom:leafShapeTip").Get()
        if base is None or tip is None:
            raise ValueError(f"Missing semantic leaf endpoints: {prim.GetPath()}")
        transform = cache.GetLocalToWorldTransform(prim)
        base = np.array(transform.Transform(Gf.Vec3d(base)))
        tip = np.array(transform.Transform(Gf.Vec3d(tip)))
        direction = tip - base
        if np.linalg.norm(direction[:2]) < 1e-6:
            continue
        leaves.append(
            {
                "path": str(prim.GetPath()),
                "base": base,
                "yaw": float(np.arctan2(direction[1], direction[0])),
            }
        )
    if len(leaves) < 5:
        raise ValueError("Need at least five leaf blades")
    # Choose a visible external leaf, then its nearest neighbours. The same five
    # replacements exist in every case, including the fully static reference.
    anchor = max(leaves, key=lambda r: r["base"][0] + r["base"][1] + 0.3 * r["base"][2])
    primary = getattr(a, "canopy_selection", "nearby") == "primary"
    layout_check = verify_layout_hulls if primary else verify_layout
    candidates = sorted(
        leaves,
        key=lambda r: (
            bool(primary and "/LatLeaf_" in r["path"]),
            float(np.linalg.norm(r["base"] - anchor["base"])),
            r["path"],
        ),
    )
    scaling = getattr(a, "drop_test", "single") == "scale"
    selected, placements = [], []
    for record in candidates:
        minimum_height = (
            0.3 * (c.length - c.fixed_length) + c.thickness / 2 + 0.001
            if primary
            else 0.08
        )
        if scaling and record["base"][2] < minimum_height:
            continue
        offset = record["base"] - np.array([0, 0, c.height])
        placement = (offset, record["yaw"])
        try:
            layout_check(points, placements + [placement], c.thickness)
        except ValueError:
            continue
        selected.append(record)
        placements.append(placement)
        if not scaling and len(selected) == 5:
            break
    shifts = [np.zeros(3)]
    if scaling and getattr(a, "plant_copies", 1) > 1:
        originals, original_placements = list(selected), list(placements)
        for replica in range(1, a.plant_copies):
            shift = np.array([float(replica % 3), float(replica // 3), 0.0])
            wrapper = f"/World/Replicas/R{replica:03d}"
            UsdGeom.Xform.Define(stage, wrapper).AddTranslateOp().Set(Gf.Vec3d(*shift))
            Sdf.CopySpec(
                stage.GetRootLayer(),
                "/World/Canopy",
                stage.GetRootLayer(),
                wrapper + "/Canopy",
            )
            selected.extend(
                {
                    **r,
                    "path": r["path"].replace("/World/Canopy", wrapper + "/Canopy", 1),
                    "base": r["base"] + shift,
                }
                for r in originals
            )
            placements.extend((o + shift, y) for o, y in original_placements)
            shifts.append(shift)
        layout_check(points, placements, c.thickness)
    if scaling and a.leaves > len(selected):
        raise ValueError(
            f"Only {len(selected)} separated native placements available; requested {a.leaves}"
        )
    if not scaling and len(selected) != 5:
        raise ValueError("Cannot find five separated lamina placements")
    paths, animations, geometry = [], [], []
    for i, (record, (offset, yaw)) in enumerate(zip(selected, placements)):
        UsdGeom.Imageable(stage.GetPrimAtPath(record["path"])).MakeInvisible()
        prefix = f"/World/Leaves/L{i:03d}"
        root = UsdGeom.Xform.Define(stage, prefix)
        _, _, anim, geo, info = build(
            world,
            c,
            "skinning",
            points,
            faces,
            "real",
            prefix,
            i == 0,
            create_view=False,
        )
        root.AddTranslateOp().Set(Gf.Vec3d(*offset))
        orientation = Gf.Quatf(
            float(np.cos(yaw / 2)), Gf.Vec3f(0, 0, float(np.sin(yaw / 2)))
        )
        root.AddOrientOp().Set(orientation)
        fixed = UsdPhysics.FixedJoint.Get(stage, prefix + "/FixedPetiole")
        fixed.CreateLocalPos0Attr().Set(
            Gf.Vec3f(*(geo[0][0] @ yaw_matrix(yaw).T + offset))
        )
        fixed.CreateLocalRot0Attr().Set(orientation)
        anim.CreateTranslationsAttr().Set(Vt.Vec3fArray(geo[0].tolist()))
        anim.CreateRotationsAttr().Set(Vt.QuatfArray([Gf.Quatf(1.0)] * 4))
        if i == 0:
            Sdf.CopySpec(
                stage.GetRootLayer(),
                prefix + "/Probe",
                stage.GetRootLayer(),
                "/World/Probe",
            )
            stage.RemovePrim(prefix + "/Probe")
            stage.GetPrimAtPath("/World/Probe").GetAttribute("xformOp:translate").Set(
                Gf.Vec3d(
                    *(np.array([0.06, 0, c.height + 0.05]) @ yaw_matrix(yaw).T + offset)
                )
            )
        if i >= a.leaves:
            freeze(stage, prefix)
        else:
            paths += [prefix + "/Petiole"] + [prefix + f"/Link{j}" for j in range(3)]
            animations.append(anim)
            geometry.append(geo)
    # Background geometry is visual-only for this first isolated cost comparison.
    remaining_physics = [
        str(p.GetPath())
        for p in Usd.PrimRange(stage.GetPrimAtPath("/World/Canopy"))
        if any(s.startswith(("Physics", "Physx")) for s in p.GetAppliedSchemas())
    ]
    if remaining_physics:
        raise RuntimeError("Unfrozen physics in source plant")
    bbox = (
        UsdGeom.BBoxCache(0, ["default", "render"])
        .ComputeWorldBound(stage.GetPrimAtPath("/World/Canopy"))
        .ComputeAlignedRange()
    )
    low, high = np.array(bbox.GetMin()), np.array(bbox.GetMax())
    low = np.min([low + shift for shift in shifts], axis=0)
    high = np.max([high + shift for shift in shifts], axis=0)
    envelope = np.concatenate(
        [points @ yaw_matrix(yaw).T + offset for offset, yaw in placements]
    )
    low, high = np.minimum(low, envelope.min(0)), np.maximum(high, envelope.max(0))
    extent = high - low
    target = (low + high) / 2
    # Conservative sphere fit to the camera's vertical field of view.
    radius = float(np.linalg.norm(extent) / 2)
    eye = (
        target
        + np.array([1.0, 1.0, 0.65]) / np.linalg.norm([1, 1, 0.65]) * radius * 4.3
    )
    info["canopy"] = {
        "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "source_leaf_count": len(leaves),
        "selection": "primary" if primary else "nearby",
        "initial_clearance_check": "convex hull SAT, 1 mm" if primary else "AABB, 1 mm",
        "primary_branch_selected": sum("/LatLeaf_" not in r["path"] for r in selected),
        "unselected_source_paths": [
            r["path"] for r in leaves if r["path"] not in {v["path"] for v in selected}
        ],
        "source_joints_and_scenes_removed": removed,
        "background_collision_enabled": False,
        "replacement_count": len(selected),
        "plant_copies": len(shifts),
        "copy_offsets_m": [shift.tolist() for shift in shifts],
        "replacement_policy": "Same 90 mm seed-42 candidate at native attachment points; horizontal, native projected yaw; all other visual geometry frozen",
        "selected": [
            {"path": r["path"], "base_m": r["base"].tolist(), "yaw": r["yaw"]}
            for r in selected
        ],
        "camera_eye": eye.tolist(),
        "camera_target": target.tolist(),
        "bounds": [low.tolist(), high.tolist()],
        "all_placements": [{"offset": o.tolist(), "yaw": y} for o, y in placements],
    }
    return placements[: a.leaves], paths, animations, geometry, info
