"""Fresh v2.3 scaffold export for experimental original-truss graft tests.

The intermediate rank6 bodies must be replaced by graft_original_trusses.py
before simulation. This script is not a new production preset.
"""
import argparse
import hashlib
import json
from pathlib import Path
from plant_state import load_plant_state
from exporterV2.plant_state_legacy_backend import export_incremental_checkpoint


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--fixture',choices=('direct','full'),required=True)
    p.add_argument('--input',type=Path,default=Path('data/plant_states/plant_state_day_160.json'))
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    state=load_plant_state(a.input)
    plan,path,manifest=export_incremental_checkpoint(state,a.output/'source.usda',debug_profile='full',physics_hz=60,
        allow_experimental_fruit_physics=True,experimental_truss_preset='main-rank6-standard',experimental_fixture=a.fixture)
    summary=dict(input=str(a.input.resolve()),input_sha256=hashlib.sha256(a.input.read_bytes()).hexdigest(),
                 fixture=a.fixture,source=str(path),manifest=str(manifest),
                 physical_links=plan.physical_link_count,predicted_d6_joints=plan.predicted_d6_joints,
                 note='Intermediate scaffold only; replace standardized trusses before simulation.')
    (a.output/'build-summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary))


if __name__=='__main__':main()
