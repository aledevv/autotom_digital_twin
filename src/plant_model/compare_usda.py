#!/usr/bin/env python3
"""
compare_usda.py

Semantic (pxr.Usd-based) comparison of two .usda stages — used to verify that
v1's export_plant_usd output is structurally/numerically unchanged after the
refactor. A plain text diff is NOT sufficient (attribute ordering / stage
metadata like creationTime can legitimately differ), so this opens both
files with Usd.Stage and compares:

  - the set of prim paths (added/removed prims)
  - prim type (e.g. Xform / Cylinder / Sphere / Mesh)
  - all authored attributes' values, with float/vector/matrix tolerance
  - mesh points / faceVertexCounts / faceVertexIndices topology
  - relationship targets (e.g. material bindings, joint bodies)

Usage:
    python3 compare_usda.py <baseline.usda> <candidate.usda> [--tol 1e-6]
"""
import sys
import argparse
import math

from pxr import Usd, UsdGeom, Gf, Vt


def _is_floatish(v):
    return isinstance(v, (float, int)) and not isinstance(v, bool)


def _to_float_list(v):
    """Flatten Gf vector/matrix/quat types (and plain floats) into a list of floats."""
    if v is None:
        return None
    if _is_floatish(v):
        return [float(v)]
    if isinstance(v, (Gf.Vec2f, Gf.Vec2d, Gf.Vec3f, Gf.Vec3d, Gf.Vec4f, Gf.Vec4d)):
        return [float(x) for x in v]
    if isinstance(v, (Gf.Matrix4d, Gf.Matrix4f)):
        return [float(v[i][j]) for i in range(4) for j in range(4)]
    if isinstance(v, (Gf.Quatf, Gf.Quatd)):
        imag = v.GetImaginary()
        return [float(v.GetReal()), float(imag[0]), float(imag[1]), float(imag[2])]
    if isinstance(v, (list, tuple, Vt.Vec3fArray, Vt.Vec3dArray, Vt.FloatArray,
                      Vt.DoubleArray, Vt.IntArray, Vt.Vec2fArray)):
        out = []
        for item in v:
            sub = _to_float_list(item)
            if sub is None:
                return None
            out.extend(sub)
        return out
    return None


def values_equal(a, b, tol):
    """Compare two attribute values with float tolerance where possible."""
    if a is None and b is None:
        return True
    if (a is None) != (b is None):
        return False

    fa = _to_float_list(a)
    fb = _to_float_list(b)
    if fa is not None and fb is not None:
        if len(fa) != len(fb):
            return False
        return all(math.isclose(x, y, rel_tol=0, abs_tol=tol) for x, y in zip(fa, fb))

    # Fall back to exact equality (strings, bools, tokens, etc.)
    return a == b


def load_stage(path):
    stage = Usd.Stage.Open(path)
    if stage is None:
        raise RuntimeError(f"Failed to open stage: {path}")
    return stage


def compare_stages(baseline_path, candidate_path, tol=1e-6):
    base = load_stage(baseline_path)
    cand = load_stage(candidate_path)

    diffs = []

    base_prims = {p.GetPath(): p for p in base.Traverse()}
    cand_prims = {p.GetPath(): p for p in cand.Traverse()}

    base_paths = set(base_prims.keys())
    cand_paths = set(cand_prims.keys())

    missing_in_candidate = sorted(base_paths - cand_paths, key=str)
    added_in_candidate = sorted(cand_paths - base_paths, key=str)

    for p in missing_in_candidate:
        diffs.append(f"[PRIM MISSING] {p} present in baseline but not in candidate")
    for p in added_in_candidate:
        diffs.append(f"[PRIM ADDED] {p} present in candidate but not in baseline")

    common_paths = sorted(base_paths & cand_paths, key=str)

    for path in common_paths:
        bp = base_prims[path]
        cp = cand_prims[path]

        if bp.GetTypeName() != cp.GetTypeName():
            diffs.append(
                f"[TYPE MISMATCH] {path}: baseline={bp.GetTypeName()} candidate={cp.GetTypeName()}"
            )
            continue

        # Compare authored attributes (union of both, since defaults may differ)
        battrs = {a.GetName(): a for a in bp.GetAttributes() if a.HasAuthoredValue()}
        cattrs = {a.GetName(): a for a in cp.GetAttributes() if a.HasAuthoredValue()}

        for name in sorted(set(battrs) | set(cattrs)):
            if name not in battrs:
                diffs.append(f"[ATTR ADDED] {path}.{name} authored only in candidate")
                continue
            if name not in cattrs:
                diffs.append(f"[ATTR MISSING] {path}.{name} authored only in baseline")
                continue

            bval = battrs[name].Get()
            cval = cattrs[name].Get()
            if not values_equal(bval, cval, tol):
                diffs.append(
                    f"[ATTR VALUE] {path}.{name}: baseline={bval!r} candidate={cval!r}"
                )

        # Compare relationships (material bindings, joint body targets, etc.)
        brels = {r.GetName(): r for r in bp.GetRelationships() if r.GetTargets()}
        crels = {r.GetName(): r for r in cp.GetRelationships() if r.GetTargets()}
        for name in sorted(set(brels) | set(crels)):
            btargets = sorted(str(t) for t in brels[name].GetTargets()) if name in brels else None
            ctargets = sorted(str(t) for t in crels[name].GetTargets()) if name in crels else None
            if btargets != ctargets:
                diffs.append(
                    f"[REL TARGETS] {path}.{name}: baseline={btargets} candidate={ctargets}"
                )

        # Mesh-specific topology check (points / faceVertexCounts / faceVertexIndices)
        if bp.IsA(UsdGeom.Mesh):
            bmesh = UsdGeom.Mesh(bp)
            cmesh = UsdGeom.Mesh(cp)
            bpts = bmesh.GetPointsAttr().Get()
            cpts = cmesh.GetPointsAttr().Get()
            if not values_equal(bpts, cpts, tol):
                diffs.append(f"[MESH POINTS] {path}: points differ beyond tolerance")

            bfc = bmesh.GetFaceVertexCountsAttr().Get()
            cfc = cmesh.GetFaceVertexCountsAttr().Get()
            if list(bfc) != list(cfc):
                diffs.append(f"[MESH TOPOLOGY] {path}: faceVertexCounts differ")

            bfi = bmesh.GetFaceVertexIndicesAttr().Get()
            cfi = cmesh.GetFaceVertexIndicesAttr().Get()
            if list(bfi) != list(cfi):
                diffs.append(f"[MESH TOPOLOGY] {path}: faceVertexIndices differ")

    return diffs


def main():
    parser = argparse.ArgumentParser(description="Semantic comparison of two USD stages.")
    parser.add_argument("baseline")
    parser.add_argument("candidate")
    parser.add_argument("--tol", type=float, default=1e-6)
    args = parser.parse_args()

    diffs = compare_stages(args.baseline, args.candidate, tol=args.tol)

    if not diffs:
        print(f"[OK] No semantic differences found between:\n  baseline : {args.baseline}\n  candidate: {args.candidate}")
        sys.exit(0)
    else:
        print(f"[DIFF] {len(diffs)} semantic difference(s) found between:\n  baseline : {args.baseline}\n  candidate: {args.candidate}\n")
        for d in diffs:
            print(" -", d)
        sys.exit(1)


if __name__ == "__main__":
    main()
