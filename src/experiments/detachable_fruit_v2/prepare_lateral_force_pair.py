"""Prepare matched PGS COM-force or native-drag tests on lateral supports."""
import argparse
import json
from pathlib import Path
from prepare_lateral_diagnosis import prepare

TARGET='/World/TerminalBodies/Truss_r5_o0_g421757_rachis_pedicel_lat_2_L_tomato'


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('source',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--interaction',choices=['com','native'],default='com')
    a=p.parse_args();source=a.source.resolve();output=a.output.resolve()
    original=json.loads((source/'config.json').read_text())
    assert original['solver']=='PGS' and original['hz']==60 and original['break_force']==6.
    for name,variant in [('mobile','baseline'),('fixed','fixed')]:
        directory=output/name
        prepare(source,directory,variant,60.)
        c=json.loads((directory/'config.json').read_text())
        c.update(interaction=a.interaction,force_target=TARGET,force_start=30.,force_direction=[0.,0.,1.],retain_fruit_grip=False)
        if a.interaction=='native':
            c.update(mouse_grab_mode='joint',mouse_force_coefficient=10.,drag_distance=.2)
        (directory/'config.json').write_text(json.dumps(c,indent=2)+'\n')
        v=json.loads((directory/'variant.json').read_text())
        v.update(config=c,force_protocol=dict(target=TARGET,mode='center_of_mass',direction_world=[0.,0.,1.],
            settle_seconds=30.,ramp_seconds=5.,peak_newtons=12.,stop_on_break=True,observe_until_seconds=60.))
        if a.interaction=='native':
            v.pop('force_protocol')
            v['native_protocol']=dict(target=TARGET,mode='joint',coefficient=10.,
                direction_world=[0.,0.,1.],settle_seconds=30.,drag_seconds=5.,
                displacement_m=.2,release_on_break=True,observe_until_seconds=60.,
                applied_force_newtons=None,view='horizontal ray from 0.5 m away; verify matched selection directions')
        (directory/'variant.json').write_text(json.dumps(v,indent=2)+'\n')


if __name__=='__main__':main()
