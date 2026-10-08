from __future__ import annotations

import importlib.util
from pathlib import Path

_PATH = Path(__file__).resolve().parents[2] / "experiments" / "complexity_study" / "fidelity_bench.py"
_SPEC = importlib.util.spec_from_file_location("fidelity_bench", _PATH)
fidelity_bench = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(fidelity_bench)

V = "/World/Stem/Vegetative"


def test_keys_survive_link_merging():
    plain = f"{V}/trunk/trunk_Link_02_Internode_g2/OrganicVisual_02"
    merged = f"{V}/trunk/trunk_Link_01_Internode_g1/Merged_trunk_Link_02_Internode_g2/OrganicVisual_02"
    nested = f"{V}/b/b_Link_01_X/Merged_b_Link_02_Y/Merged_b_Link_03_Internode_g3/OrganicVisual_03"
    assert fidelity_bench.mesh_key(plain) == fidelity_bench.mesh_key(merged)
    assert fidelity_bench.mesh_key(nested) == "b_Link_03_Internode_g3/OrganicVisual_03"


def test_leaflet_keys_match_visual_and_physical_petiolules():
    visual = f"{V}/Leaf_petiole/L_Link_01_Petiole_g7/RigidLeafVisuals/LeafVisual_g7_petiolule_left_01/LeafBlade"
    physical = (f"{V}/Leaf_petiole/L_Link_01_Petiole_g7/Merged_LeafVisual_g7_petiolule_left_01_physical_Link_01_"
                "LeafVisual_g7_petiolule_left_01/LeafBlade")
    terminal = f"{V}/x/RigidLeafVisuals/LeafVisual_g7_rachis_terminal_02/LeafBlade"
    assert fidelity_bench.mesh_key(visual) == fidelity_bench.mesh_key(physical) == "LeafVisual_g7_petiolule_left_01/LeafBlade"
    assert fidelity_bench.mesh_key(terminal) == "LeafVisual_g7_rachis_terminal_02/LeafBlade"
    assert fidelity_bench.role_of("LeafVisual_g7_petiolule_left_01/LeafBlade") == "leaf_blade"
    assert fidelity_bench.role_of("Truss_Link_02_TrussRachis_g9/OrganicVisual_02") == "truss_rachis"
