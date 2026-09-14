"""Fresh full-truss cases along an explicit transition from the main reference."""
import argparse
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[3]
CASES = {
    'baseline': (),
    'density': ('density',),
    'geometry': ('geometry',),
    'stiffness': ('stiffness',),
    'arming': ('arming',),
    'density-geometry': ('density','geometry'),
    'density-geometry-stiffness': ('density','geometry','stiffness'),
    'latest': ('density','geometry','stiffness','arming'),
    'geometry-stiffness': ('geometry','stiffness'),
    'latest-heavy-supports': ('geometry','stiffness','arming'),
}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-root',type=Path,required=True)
    parser.add_argument('--duration',type=float,default=20.)
    parser.add_argument('cases',nargs='+',choices=tuple(CASES))
    args=parser.parse_args()
    for name in args.cases:
        factors=CASES[name]
        directory=(args.run_root/name).resolve()
        command=[sys.executable,str(Path(__file__).with_name('prepare_support_matrix.py')),
                 '--source-case',str(ROOT/'artifacts/detachable_fruit_v2/2026-09-10/native-comparison/main/tgs-6n-preflight'),
                 '--run-dir',str(directory),'--density','2000' if 'density' in factors else '20000',
                 '--duration',str(args.duration), '--fruit-break-force','6']
        if 'geometry' in factors:
            command.append('--coherent-fruit')
        if 'stiffness' in factors:
            command += ['--rachis-stiffness-scale','2']
        if 'arming' in factors:
            command.append('--arm-after-settle')
        subprocess.run(command,cwd=ROOT,check=True)
        (directory/'transition.json').write_text(json.dumps(dict(case=name,factors=factors,prepare_command=command),indent=2)+'\n')


if __name__=='__main__':
    main()
