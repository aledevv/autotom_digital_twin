"""Reproduce the first recorded slow GUI drag, including post-break native holding.

Uses local evidence from gui-wu5dikte; outputs eight fresh diagnostic cases.
The original source scenes must exist; existing cases are never overwritten.
"""
import json,hashlib,shutil
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parents[3];base=root/'artifacts/detachable_fruit_v2/generic-truss-search';gui=base/'gui-wu5dikte'
rows=[json.loads(l) for l in (gui/'gui-native-interaction.jsonl').read_text().splitlines()];begin=next(x for x in rows if x['event']=='begin');end=next(x for x in rows if x['event']=='release');clip=[x for x in rows if begin['time_s']<=x['time_s']<=end['time_s'] and x['event'] in ['begin','move','release']]
files=sorted(gui.glob('gui-trace-*.npz'));s=np.concatenate([np.load(f)['state'] for f in files]);paths=list(np.load(files[0])['paths']);times=np.load(gui/'gui-metrics.npz')['samples'][:,0];idx=int(np.argmin(abs(times-begin['time_s'])));center=s[idx,paths.index(begin['body']),:3].tolist()
for name,source,mode,coef in [('retained-scheduled-force50','tgs-scheduled-native','force',50),('retained-default-force50','tgs-hold','force',50),('retained-scheduled-joint10','tgs-scheduled-native','joint',10),('retained-scheduled-force10','tgs-scheduled-native','force',10),('retained-scheduled-force25','tgs-scheduled-native','force',25),('retained-scheduled-force35','tgs-scheduled-native','force',35),('retained-scheduled-joint50','tgs-scheduled-native','joint',50),('retained-pgs-force50','pgs-hold','force',50)]:
 out=base/name;out.mkdir(exist_ok=False);shutil.copyfile(base/source/'scene.usda',out/'scene.usda');c=json.loads((base/source/'config.json').read_text());c.update(run_dir=str(out),duration=60,interaction='native',force_start=30,force_target=begin['body'],mouse_grab_mode=mode,mouse_force_coefficient=coef,native_replay_hold_after_break=True,diagnostic_all_body_speed_limit_m_s=1000,allow_experimental_native_coefficient=coef>10)
 for key in ['drag_profile','hold_seconds','hold_force']:c.pop(key,None)
 c['native_recording']=dict(source=str(gui),source_log_sha256=hashlib.sha256((gui/'gui-native-interaction.jsonl').read_bytes()).hexdigest(),reference_body_position=center,source_fruit=begin['body'],events=[dict(relative_time_s=x['time_s']-begin['time_s'],event=x['event'],origin=x['origin'],direction=x['direction']) for x in clip])
 for rel in ['src/exporterV2/fruit_interaction.py','src/exporterV2/fruit_diagnostics.py']:c['implementation_sha256'][rel]=hashlib.sha256((root/rel).read_bytes()).hexdigest()
 (out/'config.json').write_text(json.dumps(c,indent=2)+'\n')
print('Prepared eight matched retained-native cases; clip duration',end['time_s']-begin['time_s'])
