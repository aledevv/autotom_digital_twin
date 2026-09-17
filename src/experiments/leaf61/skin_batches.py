"""One USD mesh/skeleton per small cluster; all physical leaves stay separate."""

import numpy as np
from batch_runtime import LocalPoseBatch
from plant_model import yaw_matrix

from model import skin


class SkinBatches:
    def __init__(self, stage, points, faces, geometry, offsets, yaws, size, quat_order):
        from pxr import Gf, UsdGeom, UsdSkel, Vt

        self.stage, self.points, self.faces, self.geometry = (
            stage,
            points,
            faces,
            geometry,
        )
        self.offsets, self.yaws, self.quat_order = offsets, yaws, quat_order
        self.matrices = np.asarray([yaw_matrix(y) for y in yaws])
        self.co, self.si = np.cos(yaws / 2)[:, None], np.sin(yaws / 2)[:, None]
        self.converter = LocalPoseBatch(offsets, yaws)
        self.displayed = np.zeros((len(offsets), 4, 7), dtype=np.float32)
        self.rest_error = np.zeros(len(offsets))
        self.blocks = []
        for lo in range(0, len(offsets), size):
            hi = min(lo + size, len(offsets))
            path = f"/World/SkinBatches/B{lo:04d}"
            root = UsdSkel.Root.Define(stage, path)
            skeleton = UsdSkel.Skeleton.Define(stage, path + "/Skeleton")
            animation = UsdSkel.Animation.Define(stage, path + "/Animation")
            mesh = UsdGeom.Mesh.Define(stage, path + "/Mesh")
            vertices, triangles, indices, weights, binds, names = [], [], [], [], [], []
            for i in range(lo, hi):
                centers, idx, w, _ = geometry[i]
                world = points @ self.matrices[i].T + offsets[i]
                self.rest_error[i] = np.linalg.norm(
                    world.astype(np.float32) - world, axis=1
                ).max()
                vertices.append(world)
                triangles.append(faces + (i - lo) * len(points))
                indices.append(idx + (i - lo) * 4)
                weights.append(w)
                for j, center in enumerate(centers):
                    bind = Gf.Matrix4d().SetRotate(
                        Gf.Quatd(
                            float(np.cos(yaws[i] / 2)),
                            Gf.Vec3d(0, 0, float(np.sin(yaws[i] / 2))),
                        )
                    )
                    bind.SetTranslateOnly(
                        Gf.Vec3d(*(center @ self.matrices[i].T + offsets[i]))
                    )
                    binds.append(bind)
                    names.append(f"leaf{i}_bone{j}")
                UsdGeom.Imageable(
                    stage.GetPrimAtPath(f"/World/Leaves/L{i:03d}/Leaf")
                ).MakeInvisible()
            mesh.CreatePointsAttr().Set(
                Vt.Vec3fArray.FromNumpy(np.concatenate(vertices).astype(np.float32))
            )
            mesh.CreateFaceVertexCountsAttr().Set(
                Vt.IntArray.FromNumpy(np.full((hi - lo) * len(faces), 3, np.int32))
            )
            mesh.CreateFaceVertexIndicesAttr().Set(
                Vt.IntArray.FromNumpy(
                    np.concatenate(triangles).ravel().astype(np.int32)
                )
            )
            mesh.CreateSubdivisionSchemeAttr().Set("none")
            mesh.CreateDoubleSidedAttr().Set(True)
            mesh.CreateDisplayColorAttr().Set([Gf.Vec3f(0.12, 0.42, 0.05)])
            skeleton.CreateJointsAttr().Set(names)
            skeleton.CreateBindTransformsAttr().Set(Vt.Matrix4dArray(binds))
            skeleton.CreateRestTransformsAttr().Set(Vt.Matrix4dArray(binds))
            animation.CreateJointsAttr().Set(names)
            animation.CreateScalesAttr().Set(Vt.Vec3hArray([Gf.Vec3h(1)] * len(names)))
            UsdSkel.BindingAPI.Apply(
                skeleton.GetPrim()
            ).CreateAnimationSourceRel().SetTargets([animation.GetPath()])
            binding = UsdSkel.BindingAPI.Apply(mesh.GetPrim())
            binding.CreateSkeletonRel().SetTargets([skeleton.GetPath()])
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
                    skeleton,
                    mesh,
                    animation.CreateTranslationsAttr(),
                    animation.CreateRotationsAttr(),
                )
            )

    def update(self, pose, changed):
        from pxr import Vt

        world = np.empty_like(pose)
        world[:, :, :3] = (
            pose[:, :, :3] @ self.matrices.transpose(0, 2, 1) + self.offsets[:, None, :]
        )
        w, x, y, z = np.moveaxis(pose[:, :, 3:], -1, 0)
        world[:, :, 3] = self.co * w - self.si * z
        world[:, :, 4] = self.co * x - self.si * y
        world[:, :, 5] = self.co * y + self.si * x
        world[:, :, 6] = self.co * z + self.si * w
        world = world.astype(np.float32)
        for lo, hi, _, _, _, trans, rots in self.blocks:
            if not changed[lo:hi].any():
                continue
            trans.Set(
                Vt.Vec3fArray.FromNumpy(
                    np.ascontiguousarray(world[lo:hi, :, :3].reshape(-1, 3))
                )
            )
            rots.Set(
                Vt.QuatfArray.FromNumpy(
                    np.ascontiguousarray(
                        world[lo:hi, :, 3:].reshape(-1, 4)[:, self.quat_order]
                    )
                )
            )
            self.displayed[lo:hi] = world[lo:hi]
        return self.converter(
            self.displayed[:, :, :3].reshape(-1, 3),
            self.displayed[:, :, 3:].reshape(-1, 4),
        )

    def validate(self, pose):
        from pxr import Usd, UsdSkel

        self.update(pose, np.ones(len(pose), dtype=bool))
        maximum = 0.0
        for lo, hi, root, skeleton, mesh, _, _ in self.blocks:
            cache = UsdSkel.Cache()
            cache.Populate(root, Usd.PrimDefaultPredicate)
            query = cache.GetSkelQuery(skeleton)
            skinner = cache.GetSkinningQuery(mesh.GetPrim())
            output = mesh.GetPointsAttr().Get()
            if not skinner.ComputeSkinnedPoints(
                query.ComputeSkinningTransforms(Usd.TimeCode.Default()),
                output,
                Usd.TimeCode.Default(),
            ):
                raise RuntimeError("USD skinning evaluation failed")
            expected = []
            for i in range(lo, hi):
                centers, indices, weights, _ = self.geometry[i]
                local = skin(
                    self.points,
                    centers,
                    indices,
                    weights,
                    pose[i, :, :3],
                    pose[i, :, 3:],
                )
                expected.append(local @ self.matrices[i].T + self.offsets[i])
            maximum = max(
                maximum,
                float(
                    np.linalg.norm(
                        np.asarray(output) - np.concatenate(expected), axis=1
                    ).max()
                ),
            )
        if maximum > 1e-6:
            raise RuntimeError(
                f"Batched USD skin differs from reference by {maximum} m"
            )
        return maximum
