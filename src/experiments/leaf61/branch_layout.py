"""Replicate the accepted branch fixture without changing its local physics."""

import numpy as np


def replicate(stage, count, paths, animations, geometries, offsets):
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdSkel

    shifts = np.array([[0.30 * (i % 4), 0.38 * (i // 4), 0.0] for i in range(count)])
    prefixes = ["/World"] + [f"/World/Replicas/B{i:03d}" for i in range(1, count)]
    original_paths, original_geo, original_offsets = (
        list(paths),
        list(geometries),
        offsets.copy(),
    )
    all_offsets = [offsets]
    for i, prefix in enumerate(prefixes[1:], 1):
        root = UsdGeom.Xform.Define(stage, prefix)
        root.AddTranslateOp().Set(Gf.Vec3d(*shifts[i]))
        for name in [
            "Leaves",
            "Branch",
            "Anchor",
            "FixedRoot",
            "BranchRoot",
            "Support",
        ]:
            Sdf.CopySpec(
                stage.GetRootLayer(),
                "/World/" + name,
                stage.GetRootLayer(),
                prefix + "/" + name,
            )
        for prim in Usd.PrimRange(root.GetPrim()):
            for rel in prim.GetRelationships():
                targets = rel.GetTargets()
                rel.SetTargets(
                    [
                        Sdf.Path(prefix + str(x)[len("/World") :])
                        if str(x).startswith("/World/")
                        and not str(x).startswith(prefix + "/")
                        else x
                        for x in targets
                    ]
                )
        stage.GetPrimAtPath(prefix + "/FixedRoot").GetAttribute(
            "physics:localPos0"
        ).Set(Gf.Vec3f(*(shifts[i] + [0, 0, 0.2])))
        paths += [prefix + x[len("/World") :] for x in original_paths]
        animations += [
            UsdSkel.Animation.Get(stage, prefix + f"/Leaves/L{j:03d}/Leaf/Animation")
            for j in range(3)
        ]
        geometries += original_geo
        all_offsets.append(original_offsets + shifts[i])
    return prefixes, shifts, np.concatenate(all_offsets)


def isolate(stage, prefixes):
    """Each replica interacts only within its own group, including its floor."""
    from pxr import UsdPhysics

    groups = []
    for i, prefix in enumerate(prefixes):
        group = UsdPhysics.CollisionGroup.Define(
            stage, f"/World/BranchCollisionGroups/G{i:03d}"
        )
        group.GetCollidersCollectionAPI().CreateIncludesRel().SetTargets(
            [
                prefix + "/" + name
                for name in ["Leaves", "Branch", "Tomato", "Floor"]
                if stage.GetPrimAtPath(prefix + "/" + name).IsValid()
            ]
        )
        groups.append(group)
    for i, group in enumerate(groups):
        group.CreateFilteredGroupsRel().SetTargets(
            [g.GetPath() for j, g in enumerate(groups) if j != i]
        )


def attachment_error(branch_poses, petiole_positions, shifts, local_offsets):
    """Full-rate attachment check for all branches in one NumPy batch."""
    from model import rotation

    rotations = rotation(branch_poses[:, [6, 3, 4, 5]])
    root_error = np.linalg.norm(
        branch_poses[:, :3] - 0.14 * rotations[:, :, 1] - shifts - [0, 0, 0.20], axis=1
    )
    expected = branch_poses[:, None, :3] + np.einsum(
        "bij,bkj->bki", rotations, local_offsets.reshape(-1, 3, 3)
    )
    leaf_error = np.linalg.norm(expected - petiole_positions.reshape(-1, 3, 3), axis=2)
    return float(max(root_error.max(), leaf_error.max()))
