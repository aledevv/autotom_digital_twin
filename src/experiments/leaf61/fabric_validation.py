"""Check renderer-side native rigid transforms against the physics tensors."""

import numpy as np


def validate_native_fabric(paths, poses):
    import omni.usd
    import usdrt

    stage = usdrt.Usd.Stage.Attach(omni.usd.get_context().get_stage_id())
    position_error = 0.0
    orientation_error = 0.0
    for path, pose in zip(paths, poses):
        xf = usdrt.Rt.Xformable(stage.GetPrimAtPath(path))
        matrix_attr = xf.GetFabricHierarchyWorldMatrixAttr()
        if not matrix_attr:
            raise RuntimeError("Missing Fabric rigid transform: " + path)
        matrix = np.array(matrix_attr.Get())
        from pxr import Gf

        pos = matrix[3, :3]
        q = Gf.Matrix4d(*matrix.ravel().tolist()).ExtractRotation().GetQuat()
        quat = np.array([*q.GetImaginary(), q.GetReal()])
        position_error = max(
            position_error, float(np.linalg.norm(np.array(pos) - pose[:3]))
        )
        orientation_error = max(
            orientation_error,
            float(
                min(np.linalg.norm(quat - pose[3:]), np.linalg.norm(quat + pose[3:]))
            ),
        )
    if position_error > 1e-6 or orientation_error > 1e-5:
        raise RuntimeError(
            f"Fabric render transform mismatch: {position_error} m / {orientation_error} quaternion"
        )
    return {
        "native_bodies": len(paths),
        "position_error_m": position_error,
        "quaternion_error": orientation_error,
    }
