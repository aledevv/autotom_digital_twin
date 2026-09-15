"""Launch a prepared collision experiment; contact instrumentation stays outside core."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import shutil
import tempfile
import time

ROOT = Path(__file__).resolve().parents[3]


def install_monitor():
    import numpy as np
    sys.path.insert(0, str(ROOT / 'src'))
    from exporterV2 import fruit_diagnostics
    original = fruit_diagnostics.run

    def run(stage, world, app, args, config):
        from pxr import PhysicsSchemaTools
        from isaacsim.core.prims import RigidPrim
        from omni.physx import get_physx_simulation_interface
        from exporterV2.fruit_experiments import bodies_and_joints
        from exporterV2.gui_watchdog import GuiWatchdog
        output = Path(config['run_dir'])
        if not args.gui_watchdog:
            args.gui_watchdog = GuiWatchdog(output)
        paths = sorted(bodies_and_joints(stage)[0])
        view = RigidPrim(paths, name='collision_offsets', reset_xform_properties=False)
        view.initialize()
        offsets = np.asarray(view._physics_view.get_contact_offsets())
        rest = np.asarray(view._physics_view.get_rest_offsets())
        (output / 'loaded-offsets.json').write_text(json.dumps({p:dict(contact=offsets[i].tolist(), rest=rest[i].tolist())
            for i,p in enumerate(view.prim_paths)}, indent=2)+'\n')
        pending, counts, decoded = [], [], {}
        def path(identifier):
            if identifier not in decoded:
                decoded[identifier] = str(PhysicsSchemaTools.intToSdfPath(identifier))
            return decoded[identifier]
        callback_seconds = 0.
        def contact(headers, data):
            nonlocal callback_seconds
            start = time.perf_counter()
            for h in headers:
                points = data[h.contact_data_offset:h.contact_data_offset+h.num_contact_data]
                pending.append(dict(type=int(h.type), body_a=path(h.actor0), body_b=path(h.actor1),
                    collider_a=path(h.collider0), collider_b=path(h.collider1),
                    points=[dict(position=list(x.position), separation=float(x.separation), impulse=list(x.impulse)) for x in points]))
            callback_seconds += time.perf_counter()-start
        subscription = (get_physx_simulation_interface().subscribe_contact_report_events(contact)
                        if config.get('contact_recording', True) else None)
        saved = world.step
        start_sim = float(world.current_time)
        log = (output/'contacts.jsonl').open('w', buffering=65536)
        flush_time = time.monotonic()
        drag = None
        if config.get('branch_drag') and args.headless:
            from native_drag import NativeBranchDrag
            drag=NativeBranchDrag(view,config['branch_drag'],output/'native-branch-rays.json')
        def step(*a, **kw):
            nonlocal flush_time
            if drag:drag.step(float(world.current_time)-start_sim)
            result = saved(*a, **kw)
            t = float(world.current_time)-start_sim
            counts.append([t, len(pending), sum(len(x['points']) for x in pending)])
            if pending:
                log.write(json.dumps(dict(time_s=t, contacts=pending))+'\n')
                pending.clear()
            if time.monotonic()-flush_time >= 1:
                log.flush();flush_time=time.monotonic()
            return result
        world.step = step
        try:
            return original(stage, world, app, args, config)
        finally:
            world.step = saved
            subscription = None
            if drag:drag.close(float(world.current_time)-start_sim)
            log.close()
            np.savez_compressed(output/'contact-counts.npz', samples=np.asarray(counts))
            (output/'contact-monitor.json').write_text(json.dumps(dict(callback_seconds=callback_seconds,
                steps=len(counts), recording=config.get('contact_recording',True)))+'\n')
    fruit_diagnostics.run = run


def launch():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('case',type=Path)
    p.add_argument('--gui',action='store_true')
    p.add_argument('--benchmark',action='store_true',help='Fresh unpaced 30-second GUI benchmark, then close')
    p.add_argument('--no-contact-recording',action='store_true')
    a=p.parse_args();directory=a.case.resolve()
    config=json.loads((directory/'config.json').read_text())
    if a.gui or a.benchmark:
        source=directory
        directory=Path(tempfile.mkdtemp(prefix='gui-'+config['collision_variant']+'-',dir=source.parent))
        shutil.copyfile(source/'scene.usda',directory/'scene.usda')
        config.update(run_dir=str(directory),gui=True,gui_real_time=not a.benchmark,
                      duration=30. if a.benchmark else 60.,force_target=None,
                      contact_recording=not a.no_contact_recording)
        config.pop('branch_drag',None)
        (directory/'config.json').write_text(json.dumps(config,indent=2)+'\n')
    elif (directory/'report.json').exists() or (directory/'process.json').exists():
        raise ValueError('Use a fresh case; existing evidence will not be overwritten')
    command=[str(Path.home()/'isaacsim/python.sh'),str(Path(__file__).resolve()),'--inside',
        '--usd',str(directory/'scene.usda'),'--physics-preset','flexible','--physics-hz',str(config['hz']),
        '--duration',str(config['duration']),'--fruit-experiment',str(directory/'config.json')]
    command += ([] if a.benchmark else ['--gui-until-close']) if (a.gui or a.benchmark) else ['--headless']
    (directory/'launch.json').write_text(json.dumps(dict(command=command,cwd=str(ROOT)),indent=2)+'\n')
    started=time.monotonic()
    print('RUN_DIR='+str(directory),flush=True)
    with (directory/'isaac.log').open('w') as log:
        process=subprocess.Popen(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
        code=process.wait()
    (directory/'process.json').write_text(json.dumps(dict(returncode=code,wall_s=time.monotonic()-started))+'\n')
    print('Isaac exit=',code,flush=True)


if __name__=='__main__':
    if '--inside' in sys.argv:
        sys.argv.remove('--inside');install_monitor()
        from exporterV2.isaac_app import main
        code=main();sys.stdout.flush();sys.stderr.flush();os._exit(code)
    else:
        launch()
