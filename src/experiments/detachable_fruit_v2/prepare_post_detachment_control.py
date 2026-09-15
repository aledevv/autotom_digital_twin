"""Compare post-drag recovery against a fresh scene with the same fruit load removed."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
from pxr import Usd, UsdPhysics
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'src'))
from exporterV2.fruit_experiments import audit


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',type=Path,required=True)
    p.add_argument('--gui',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    report=json.loads((a.gui/'gui-report.json').read_text())
    removed=sorted({e['fruit'] for e in report['events'] if e['kind']=='joint_break'})
    assert removed
    stage=Usd.Stage.Open(Usd.Stage.Open(str(a.source/'scene.usda')).Flatten())
    for path in removed:
        assert stage.GetPrimAtPath(path)
        stage.RemovePrim(path)
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.FilteredPairsAPI):
            rel=UsdPhysics.FilteredPairsAPI(prim).GetFilteredPairsRel()
            rel.SetTargets([p for p in rel.GetTargets() if stage.GetPrimAtPath(p)])
    result=audit(stage);assert not result['errors'],result['errors']
    stage.GetRootLayer().Export(str(a.output/'scene.usda'))
    c=json.loads((a.source/'config.json').read_text())
    c.update(run_dir=str(a.output.resolve()),gui=False,duration=60.,force_target=None)
    c.pop('native_recording',None)
    c['expected_body_properties']={k:v for k,v in c['expected_body_properties'].items() if k not in removed}
    c['scene_sha256']=hashlib.sha256((a.output/'scene.usda').read_bytes()).hexdigest()
    c['post_detachment_control']=dict(removed=removed,source=str(a.source),gui=str(a.gui),
        gui_report_sha256=hashlib.sha256((a.gui/'gui-report.json').read_bytes()).hexdigest(),
        purpose='Same remaining supported load, no drag history; free fallen fruit omitted, all pedicels retained.')
    c['implementation_sha256'][str(Path(__file__).relative_to(ROOT))]=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    for name,data in [('config',c),('audit',result)]:
        (a.output/(name+'.json')).write_text(json.dumps(data,indent=2)+'\n')

if __name__=='__main__':main()
