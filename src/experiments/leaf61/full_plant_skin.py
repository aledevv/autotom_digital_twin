"""Batch heterogeneous native blades without changing their skin weights or physics."""

import numpy as np


def multiply_xyzw(a, b):
    """Broadcast Hamilton product, scalar last (PhysX and Vt numpy order)."""
    return np.concatenate(
        (
            a[..., 3:] * b[..., :3]
            + b[..., 3:] * a[..., :3]
            + np.cross(a[..., :3], b[..., :3]),
            a[..., 3:] * b[..., 3:]
            - np.sum(a[..., :3] * b[..., :3], axis=-1, keepdims=True),
        ),
        axis=-1,
    )


class NativeSkinBatch:
    def __init__(self, stage, rows, size=16):
        from pxr import Gf, UsdGeom, UsdSkel, Vt

        self.rows = rows
        self.ids = np.array([r["ids"] for r in rows])
        self.hosts = np.array([r["host_id"] for r in rows])
        self.offsets = np.array([r["host_offset"] for r in rows])
        self.frames = []
        self.blocks = []
        self.displayed = None
        self.maximum_visual_error = 0.0
        for r in rows:
            q = (
                Gf.Matrix3d(*r["host_frame"].T.ravel().tolist())
                .ExtractRotation()
                .GetQuat()
            )
            self.frames.append([*q.GetImaginary(), q.GetReal()])
        self.frames = np.array(self.frames)
        bones = self.ids.shape[1] + 1
        for lo in range(0, len(rows), size):
            hi = min(lo + size, len(rows))
            path = f"/World/NativeSkinBatches/B{lo:03d}"
            root = UsdSkel.Root.Define(stage, path)
            skel = UsdSkel.Skeleton.Define(stage, path + "/Skeleton")
            anim = UsdSkel.Animation.Define(stage, path + "/Animation")
            mesh = UsdGeom.Mesh.Define(stage, path + "/Mesh")
            vertices, faces, indices, weights, binds, names = [], [], [], [], [], []
            offset = 0
            for i in range(lo, hi):
                r = rows[i]
                vertices.append(r["points"])
                faces.append(r["faces"] + offset)
                offset += len(r["points"])
                indices.append(r["indices"] + (i - lo) * bones)
                weights.append(r["weights"])
                for j, center in enumerate(r["centers"]):
                    binds.append(Gf.Matrix4d().SetTranslate(Gf.Vec3d(*center)))
                    names.append(f"leaf{i}_bone{j}")
                UsdGeom.Imageable(
                    stage.GetPrimAtPath(r["prefix"] + "/Leaf")
                ).MakeInvisible()
            mesh.CreatePointsAttr().Set(
                Vt.Vec3fArray.FromNumpy(np.concatenate(vertices).astype(np.float32))
            )
            triangles = np.concatenate(faces).astype(np.int32)
            mesh.CreateFaceVertexCountsAttr().Set(
                Vt.IntArray.FromNumpy(np.full(len(triangles), 3, np.int32))
            )
            mesh.CreateFaceVertexIndicesAttr().Set(
                Vt.IntArray.FromNumpy(triangles.ravel())
            )
            mesh.CreateSubdivisionSchemeAttr().Set("none")
            mesh.CreateDoubleSidedAttr().Set(True)
            mesh.CreateDisplayColorAttr().Set([Gf.Vec3f(0.12, 0.42, 0.05)])
            skel.CreateJointsAttr().Set(names)
            skel.CreateBindTransformsAttr().Set(Vt.Matrix4dArray(binds))
            skel.CreateRestTransformsAttr().Set(Vt.Matrix4dArray(binds))
            anim.CreateJointsAttr().Set(names)
            anim.CreateScalesAttr().Set(Vt.Vec3hArray([Gf.Vec3h(1)] * len(names)))
            UsdSkel.BindingAPI.Apply(
                skel.GetPrim()
            ).CreateAnimationSourceRel().SetTargets([anim.GetPath()])
            binding = UsdSkel.BindingAPI.Apply(mesh.GetPrim())
            binding.CreateSkeletonRel().SetTargets([skel.GetPath()])
            binding.CreateGeomBindTransformAttr().Set(Gf.Matrix4d(1))
            binding.CreateJointIndicesPrimvar(False, 2).Set(
                Vt.IntArray.FromNumpy(np.concatenate(indices).ravel().astype(np.int32))
            )
            binding.CreateJointWeightsPrimvar(False, 2).Set(
                Vt.FloatArray.FromNumpy(
                    np.concatenate(weights).ravel().astype(np.float32)
                )
            )
            self.blocks.append(
                (
                    lo,
                    hi,
                    root,
                    skel,
                    mesh,
                    anim.CreateTranslationsAttr(),
                    anim.CreateRotationsAttr(),
                )
            )

    def poses(self, poses, rotations):
        host = poses[self.hosts]
        bp = np.concatenate(
            (
                (
                    host[:, :3]
                    + np.einsum("lij,lj->li", rotations[self.hosts], self.offsets)
                )[:, None],
                poses[self.ids, :3],
            ),
            axis=1,
        )
        bq = np.concatenate(
            (multiply_xyzw(host[:, 3:], self.frames)[:, None], poses[self.ids, 3:]),
            axis=1,
        )
        return np.concatenate((bp, bq), axis=-1).astype(np.float32)

    def update(self, poses, rotations):
        from pxr import Vt

        world = self.poses(poses, rotations)
        for lo, hi, _, _, _, trans, rots in self.blocks:
            if self.displayed is not None and np.array_equal(
                world[lo:hi], self.displayed[lo:hi]
            ):
                continue
            trans.Set(
                Vt.Vec3fArray.FromNumpy(
                    np.ascontiguousarray(world[lo:hi, :, :3].reshape(-1, 3))
                )
            )
            rots.Set(
                Vt.QuatfArray.FromNumpy(
                    np.ascontiguousarray(world[lo:hi, :, 3:].reshape(-1, 4))
                )
            )
        self.displayed = world

    def validate(self, poses, rotations):
        from pxr import Usd, UsdSkel

        from model import skin

        self.update(poses, rotations)
        maximum = 0.0
        for lo, hi, root, skel, mesh, _, _ in self.blocks:
            cache = UsdSkel.Cache()
            cache.Populate(root, Usd.PrimDefaultPredicate)
            query = cache.GetSkelQuery(skel)
            output = mesh.GetPointsAttr().Get()
            if not cache.GetSkinningQuery(mesh.GetPrim()).ComputeSkinnedPoints(
                query.ComputeSkinningTransforms(Usd.TimeCode.Default()),
                output,
                Usd.TimeCode.Default(),
            ):
                raise RuntimeError("Native batch USD skin evaluation failed")
            expected = []
            for i in range(lo, hi):
                r = self.rows[i]
                p = self.displayed[i]
                expected.append(
                    skin(
                        r["points"],
                        r["centers"],
                        r["indices"],
                        r["weights"],
                        p[:, :3],
                        p[:, [6, 3, 4, 5]],
                    )
                )
            maximum = max(
                maximum,
                float(
                    np.linalg.norm(
                        np.asarray(output) - np.concatenate(expected), axis=-1
                    ).max()
                ),
            )
        if maximum > 1e-6:
            raise RuntimeError(f"Native skin batch differs by {maximum} m")
        self.maximum_visual_error = max(self.maximum_visual_error, maximum)
        return maximum
