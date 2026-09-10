"""Build a stem/truss fixture using an isolated, unmodified exporter snapshot.

Run adapter discovery with project uv; run --build with Isaac Python for main.
Branches are removed before USD construction, including all leaf visual masses.
"""
import argparse
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import re
import sys


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--code', type=Path, required=True)
    p.add_argument('--method', choices=('main', 'v23'), required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--rank', type=int)
    p.add_argument('--build', action='store_true')
    args = p.parse_args()
    args.code = args.code.resolve()
    args.output.mkdir(parents=True, exist_ok=True)
    sys.path.insert(0, str(args.code / 'src'))
    app = None
    if args.build and args.method == 'main':
        from isaacsim import SimulationApp
        app = SimulationApp({'headless': True})
    try:
        if args.method == 'main':
            from exporterV2.adapters.groimp_csv import parse_csv_to_branches
            from exporterV2.core.tree_config import limit_branch_resolution
            branches, fruits, _ = parse_csv_to_branches(160, include_terminal_bodies=True, save_json=False)
            branches, _ = limit_branch_resolution(branches)
            input_path = args.code / 'data/simulation_output/dynamic_output/graphs/graph_day_160.csv'
        else:
            from plant_state import load_plant_state
            from exporterV2.plant_state_branches import build_truss_branches, apply_checkpoint_physics_policy
            input_path = args.code / 'data/plant_states/plant_state_day_160.json'
            state = load_plant_state(input_path)
            adapter = apply_checkpoint_physics_policy(build_truss_branches(state, include_fruits=True, physical_fruits=True))
            branches, fruits = list(adapter.branches), list(adapter.terminal_bodies)
        trunk = next(b for b in branches if b['id'] == 'trunk')
        candidates = []
        for b in branches:
            match = re.match(r'Truss_r(\d+)_o\d+.*_rachis$', b['id'])
            if not match or b['parent'] != trunk['id']:
                continue
            pedicels = {x['id'] for x in branches if x.get('parent') == b['id']}
            tomatoes = [x for x in fruits if x['parent_branch_id'] in pedicels]
            candidates.append({'rank': int(match[1]), 'branch': b['id'], 'fruit_count': len(tomatoes),
                               'pedicels': sorted(pedicels), 'fruits': [x['id'] for x in tomatoes],
                               'attach_link': b['attach_link']})
        candidates.sort(key=lambda x: (x['rank'], x['branch']))
        manifest = {'method': args.method, 'input_sha256': hashlib.sha256(input_path.read_bytes()).hexdigest(),
                    'candidates': candidates, 'trunk': trunk, 'source_branch_count': len(branches)}
        (args.output / 'discovery.json').write_text(json.dumps(manifest, indent=2)+'\n')
        if not args.build:
            print(json.dumps(candidates, indent=2))
            return
        selected = next(x for x in candidates if x['rank'] == args.rank)
        keep = {trunk['id'], selected['branch'], *selected['pedicels']}
        branches = [b for b in branches if b['id'] in keep]
        fruits = [f for f in fruits if f['id'] in selected['fruits']]
        from exporterV2.core.usd import build_stage
        from exporterV2.core.physics import apply_physx_scene_settings, apply_physx_articulation_settings
        stage, stem = build_stage(str((args.output/'source.usda').resolve()), branches=branches,
                                  terminal_bodies=fruits, branch_backend='skinned', skinning_visual_mode='segmented')
        if args.method == 'v23':
            from exporterV2.plant_state_legacy_backend import (_author_stage_metadata,
                _author_historical_truss_visuals, _auto_filter_initial_overlaps)
            adapter = replace(adapter, branches=tuple(branches), terminal_bodies=tuple(fruits),
                              rigid_leaf_visuals=(), approved_collision_filters=())
            _author_stage_metadata(stage, state, adapter, 'flexible', allow_experimental_fruit_physics=True)
            _author_historical_truss_visuals(stage, adapter)
            manifest['overlap_filters'] = _auto_filter_initial_overlaps(stage)
        apply_physx_scene_settings(stage)
        apply_physx_articulation_settings(stage, stem)
        stage.GetRootLayer().Save()
        manifest.update(selected=selected, branches=branches, terminal_bodies=fruits,
                        removed_before_build=True, aggregated_leaf_mass_kg=0)
        (args.output/'build.json').write_text(json.dumps(manifest, indent=2)+'\n')
        print('[BUILT]', args.output/'source.usda')
    finally:
        if app:
            app.close()


if __name__ == '__main__':
    main()
