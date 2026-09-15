"""Repeat native PhysX mouse rays on one support, with no custom force controller."""
import json
from pathlib import Path
import numpy as np


class NativeBranchDrag:
    def __init__(self, view, config, output):
        from omni.physx import get_physx_interface, get_physx_scene_query_interface
        from omni.physx.bindings._physx import PhysicsInteractionEvent
        import carb.settings
        self.native=get_physx_interface();self.query=get_physx_scene_query_interface().raycast_closest
        self.events=PhysicsInteractionEvent;self.view=view;self.config=config;self.output=Path(output)
        self.index={p:i for i,p in enumerate(view.prim_paths)}
        self.replay=json.loads(Path(config['replay']).read_text()) if config.get('replay') else None
        self.rows=[];self.eye=None;self.cursor=0;self.released=False
        settings=carb.settings.get_settings()
        for key,v in dict(mouseInteractionEnabled=True,mouseGrab=True,mouseGrabIgnoreInvisible=False,forceGrab=True,pickingForce=50.).items():
            settings.set('/physics/'+key,v)

    def send(self,row):
        types={'begin':self.events.MOUSE_DRAG_BEGAN,'move':self.events.MOUSE_DRAG_CHANGED,'release':self.events.MOUSE_DRAG_ENDED}
        if row['event']=='begin':
            hit=self.query(tuple(row['origin']),tuple(row['direction']),10.)
            if not hit.get('hit') or str(hit.get('rigidBody'))!=self.config['body_a']:
                raise ValueError(f'Native branch selection mismatch: {hit}')
            row=dict(row, selected_body=str(hit['rigidBody']),selected_collider=str(hit['collision']))
        self.native.update_interaction(tuple(row['origin']),tuple(row['direction']),types[row['event']])
        self.rows.append(row)
        if row['event']=='release':self.released=True

    def step(self,t):
        from exporterV2.fruit_interaction import pick_ray,unit
        from exporterV2.fruit_diagnostics import rotate
        if t<30 or self.released:return
        if self.replay is not None:
            while self.cursor<len(self.replay) and self.replay[self.cursor]['time_s']<=t+1e-8:
                row=self.replay[self.cursor];self.cursor+=1;self.send(row)
            return
        if self.eye is None:
            pos,rot=self.view.get_world_poses();pos=np.asarray(pos);rot=np.asarray(rot)
            a,b=self.index[self.config['body_a']],self.index[self.config['body_b']]
            source=pos[a]+rotate(rot[a],np.asarray(self.config['local_a']))
            target=pos[b]+rotate(rot[b],np.asarray(self.config['local_b']))
            self.eye,direction,self.hit,_=pick_ray(source,self.config['body_a'],self.query)
            distance=min(.03,max(.005,np.linalg.norm(target-source)-self.config['radii_sum']+.005))
            self.delta=unit(target-source)*distance
            self.send(dict(time_s=t,event='begin',origin=self.eye.tolist(),direction=direction.tolist()))
        direction=unit(self.hit+self.delta*np.clip((t-30)/5,0,1)-self.eye)
        self.send(dict(time_s=t,event='release' if t>=40 else 'move',origin=self.eye.tolist(),direction=direction.tolist()))

    def close(self,t):
        if self.rows and not self.released:
            self.send(dict(self.rows[-1],time_s=t,event='release'))
        self.output.write_text(json.dumps(self.rows,indent=2)+'\n')
