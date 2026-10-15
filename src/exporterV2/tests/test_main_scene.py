"""The main scene rebuilds from the exporter plus the stored five trusses."""
import json

import numpy as np
import pytest
from pxr import Usd, UsdGeom, UsdPhysics

from exporterV2.main_scene import ASSETS, build, prepare_gui_run


@pytest.fixture(scope='module')
def built(tmp_path_factory):
    output = tmp_path_factory.mktemp('main_scene')
    scene = build(output)
    return output, Usd.Stage.Open(str(scene))


def test_only_the_five_main_stem_trusses_with_calyces(built):
    _, stage = built
    fruits = [p for p in stage.Traverse() if p.GetAttribute('autotom:role').Get() == 'fruit']
    assert len(fruits) == 40
    for fruit in fruits:
        calyx = UsdGeom.Mesh(stage.GetPrimAtPath(str(fruit.GetPath()) + '/CalyxVisual'))
        assert calyx and not calyx.GetPrim().HasAPI(UsdPhysics.CollisionAPI)
    roots = [p for p in stage.Traverse() if str(p.GetPath()).endswith('_rachis_Link_01')
             and p.GetAttribute('autotom:standardTruss').Get() == 'main-original-ranks-experiment']
    assert len(roots) == 5
    for root in roots:
        parent = UsdPhysics.Joint(stage.GetPrimAtPath(str(root.GetPath()) + '/AttachJoint')).GetBody0Rel().GetTargets()[0]
        assert '/Vegetative/trunk/' in str(parent)


def test_grafted_trusses_keep_the_stored_masses(built):
    _, stage = built
    records = json.loads((ASSETS / 'five_trusses_mass.json').read_text())['bodies']
    grafted = [p for p in stage.Traverse()
               if p.GetAttribute('autotom:standardTruss').Get() == 'main-original-ranks-experiment']
    assert len(grafted) == len(records) == 100
    stored = sorted(r['mass_kg'] for r in records)
    authored = sorted(UsdPhysics.MassAPI(p).GetMassAttr().Get() for p in grafted)
    assert np.allclose(stored, authored)


def test_welded_trunk_is_one_mesh(built):
    _, stage = built
    trunk = [p for p in stage.Traverse()
             if p.IsA(UsdGeom.Mesh) and '/Vegetative/trunk/' in str(p.GetPath())
             and p.GetName().startswith('OrganicVisual')]
    assert len(trunk) == 1


def test_gui_run_gets_a_fresh_folder(built):
    output, _ = built
    first, second = prepare_gui_run(output), prepare_gui_run(output)
    assert first != second
    config = json.loads((first / 'config.json').read_text())
    assert config['run_dir'] == str(first) and config['gui_real_time'] is True
