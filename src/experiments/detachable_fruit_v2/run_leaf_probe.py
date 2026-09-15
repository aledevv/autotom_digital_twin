"""Headless-only force probe wrapper; the exporter and GUI are unchanged."""
import json
import os
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from exporterV2 import fruit_diagnostics

original_run = fruit_diagnostics.run


def run_probe(stage, world, app, args, config):
    probe = config.get('leaf_compliance_probe')
    if not probe:
        return original_run(stage, world, app, args, config)
    if not args.headless:
        raise ValueError('The leaf force probe is headless-only')
    from isaacsim.core.prims import RigidPrim
    body = RigidPrim([probe['body']], name='leaf_compliance_probe', reset_xform_properties=False)
    body.initialize()
    initial_time = float(world.current_time)
    saved_step = world.step
    samples = []

    def step(*a, **kw):
        t = float(world.current_time) - initial_time
        fraction = (float(np.clip((t - probe['start_s']) / probe['ramp_s'], 0, 1))
                    if t < probe['end_s'] else 0.)
        force = np.asarray(probe['force_n'], dtype=np.float32) * fraction
        if fraction:
            body.apply_forces_and_torques_at_pos(forces=force[None, :], is_global=True)
        result = saved_step(*a, **kw)
        position, orientation = body.get_world_poses()
        samples.append([float(world.current_time) - initial_time, *force, *np.asarray(position)[0],
                        *np.asarray(orientation)[0]])
        return result

    world.step = step
    try:
        return original_run(stage, world, app, args, config)
    finally:
        world.step = saved_step
        output = Path(config['run_dir'])
        np.savez_compressed(output / 'leaf-probe.npz', samples=np.asarray(samples),
                            columns=['time', 'fx', 'fy', 'fz', 'x', 'y', 'z', 'qw', 'qx', 'qy', 'qz'])
        (output / 'leaf-probe.json').write_text(json.dumps(probe, indent=2) + '\n')


if __name__ == '__main__':
    fruit_diagnostics.run = run_probe
    from exporterV2.isaac_app import main
    code = main()
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)
