"""Add convex organic surfaces only across the manually identified support pair."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--include-lateral-leaf', action='store_true',
                        help='Include LatLeaf_r3_o0_g421593 identified in the manual contact trace')
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError('Output must be fresh')
    stage = Usd.Stage.Open(str(args.source / 'scene.usda'))
    reference = json.loads((args.source / 'headless-effective.json').read_text())
    properties = {b['path']: b for b in reference['bodies']}
    names = ('Branch_s2_o1_g421414_Link_04_Internode_g421675',
             'Leaf_r5_o0_g421371_rachis_Link_01_LeafRachis_g421371_s01',
             'Leaf_r5_o0_g421371_rachis_Link_02_LeafRachis_g421371_s02')
    selected = {str(p.GetPath()): p for p in stage.Traverse() if p.GetName() in names}
    assert len(selected) == 3
    if args.include_lateral_leaf:
        lateral = {str(p.GetPath()): p for p in stage.Traverse()
                   if p.HasAPI(UsdPhysics.RigidBodyAPI)
                   and '/LatLeaf_r3_o0_g421593_' in str(p.GetPath())}
        assert lateral, 'Missing observed lateral leaf'
        selected.update(lateral)
    # Main-stem leaf versus lateral branch + its leaf. No new collisions
    # between a petiolule and any part of its own selected support assembly.
    groups = {path: ('main_leaf' if '/Leaf_r5_o0_g421371_' in path else 'lateral_assembly')
              for path in selected}
    old_shapes = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)]

    def owner(prim):
        while prim and not prim.IsPseudoRoot():
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                return str(prim.GetPath())
            prim = prim.GetParent()
        return None

    # Optional future extension (not enabled): cover OrganicVisual meshes under
    # LeafVisual_g421563_petiolule_left_02 and analogous petiolules. Resolve each
    # mesh's owning rigid body and support assembly before extending selection:
    # exclude contacts within its own assembly, allow eligible third branches,
    # and preserve loaded mass/inertia rather than recomputing from new shapes.
    # User observed about 33 GUI FPS in this pilot (2026-09-16); this is feedback,
    # not an isolated measurement of collider cost. Benchmark matched scenes and
    # check initial overlaps/stability before any wider rollout.
    new_shapes = [p for p in stage.Traverse()
                  if p.IsA(UsdGeom.Mesh) and p.GetName().startswith('OrganicVisual_')
                  and owner(p) in selected]
    # Freeze the previously loaded mass properties: extra shapes must not
    # silently change auto-computed inertia or center of mass.
    for path, prim in selected.items():
        b = properties[path]
        inertia = np.asarray(b['inertia']).reshape(3, 3)
        assert np.allclose(inertia, np.diag(np.diag(inertia)), atol=1e-15)
        mass = UsdPhysics.MassAPI(prim)
        mass.CreateMassAttr(b['mass_kg'])
        mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*np.diag(inertia)))
        mass.CreateCenterOfMassAttr(Gf.Vec3f(*np.asarray(b['com']).reshape(3)))
        q = np.asarray(b['com_orientation']).reshape(4)
        mass.CreatePrincipalAxesAttr(Gf.Quatf(float(q[0]), Gf.Vec3f(*q[1:])))
    for prim in new_shapes:
        UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr(True)
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr('convexHull')

    def across(a, b):
        return (a in selected and b in selected
                and groups[a] != groups[b])

    for prim in new_shapes:
        exclusions = [p.GetPath() for p in old_shapes + new_shapes
                      if p != prim and not across(owner(prim), owner(p))]
        UsdPhysics.FilteredPairsAPI.Apply(prim).CreateFilteredPairsRel().SetTargets(exclusions)
    # Verify effective matrix, including existing filters on the original shapes.
    from prepare import matrix
    from exporterV2.fruit_experiments import bodies_and_joints
    shapes = [dict(path=str(p.GetPath()), body_path=owner(p)) for p in old_shapes + new_shapes]
    bodies, joints = bodies_and_joints(stage)
    effective = matrix(stage, shapes, bodies, joints)
    original = Usd.Stage.Open(str(args.source / 'scene.usda'))
    original_bodies, original_joints = bodies_and_joints(original)
    count = len(old_shapes)
    assert np.array_equal(effective[:count, :count],
                          matrix(original, shapes[:count], original_bodies, original_joints))
    for i in range(count, len(shapes)):
        for j in range(len(shapes)):
            assert bool(effective[i, j]) == across(shapes[i]['body_path'], shapes[j]['body_path'])
    args.output.mkdir(parents=True)
    stage.GetRootLayer().Export(str(args.output / 'scene.usda'))
    config = json.loads((args.source / 'config.json').read_text())
    config.update(run_dir=str(args.output.resolve()), gui=False, duration=20.,
                  collision_variant='C-organic-leaf-pair' if args.include_lateral_leaf else 'C-organic-pair', force_target=None)
    config['expected_body_properties'] = {p: {k: b[k] for k in
        ('mass_kg', 'inertia', 'com', 'com_orientation')} for p, b in properties.items()}
    config['scene_sha256'] = hashlib.sha256((args.output / 'scene.usda').read_bytes()).hexdigest()
    config['implementation_sha256']['src/experiments/branch_collisions/prepare_organic_pair.py'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    config.pop('branch_drag', None)
    (args.output / 'config.json').write_text(json.dumps(config, indent=2)+'\n')
    audit = dict(source=str(args.source), original_colliders=len(old_shapes),
                 new_mesh_colliders=[str(p.GetPath()) for p in new_shapes],
                 approximation='convexHull', selected_bodies=list(selected),
                 body_groups=groups, effective_pair_matrix_verified=True,
                 policy='New shapes contact only the opposite selected support; all other new pairs filtered',
                 preserved_runtime_mass_properties=True,
                 limitations='Convex hull fills concavities; blades remain visual; not a global rollout')
    (args.output / 'organic-audit.json').write_text(json.dumps(audit, indent=2)+'\n')
    print(json.dumps(audit))


if __name__ == '__main__':
    main()
