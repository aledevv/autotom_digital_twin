#!/usr/bin/env python3
"""
tools/compare_geometry.py  —  no pxr needed, pure math.

Usage:
    python tools/compare_geometry.py --day 1
"""
import os, sys, math, argparse
SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)
SRC_DIR      = os.path.join(PROJECT_ROOT, "src")
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

from plant_model.loader import load_snapshot
from plant_model.models import InternodeNode, LeafNode
from plant_model.v2.constants import PHYLLOTAXIS

def rot_axis_angle(axis, deg):
    t=math.radians(deg); c,s=math.cos(t),math.sin(t); x,y,z=axis
    return [[c+x*x*(1-c),x*y*(1-c)-z*s,x*z*(1-c)+y*s],
            [y*x*(1-c)+z*s,c+y*y*(1-c),y*z*(1-c)-x*s],
            [z*x*(1-c)-y*s,z*y*(1-c)+x*s,c+z*z*(1-c)]]

def mv3(M,v): return [sum(M[i][k]*v[k] for k in range(3)) for i in range(3)]
def norm3(v): n=math.sqrt(sum(x*x for x in v)); return [x/n for x in v] if n>1e-12 else v

def _base_z(n):
    if hasattr(n,'_bz'): return n._bz
    n._bz = 0.0 if (n.parent is None or not isinstance(n.parent,InternodeNode)) else _base_z(n.parent)+n.parent.length
    return n._bz

def v1_geom(snap):
    for n in snap.organs:
        if isinstance(n,InternodeNode): _base_z(n)
    out=[]
    for n in snap.organs:
        if not isinstance(n,LeafNode): continue
        az = n.ccw_orientation if abs(getattr(n,'ccw_orientation',0))>1e-3 else (n.key.rank*PHYLLOTAXIS)%360
        bz = 0.0 if (n.parent is None or not isinstance(n.parent,InternodeNode)) else _base_z(n.parent)+n.parent.length
        tilt=math.radians(90-n.angle_petiole); az_r=math.radians(az)
        dx,dy,dz=math.cos(az_r)*math.cos(tilt),math.sin(az_r)*math.cos(tilt),math.sin(tilt)
        out.append(dict(id=f"o{n.key.order}_r{n.key.rank}_i{n.key.organ_index}",attach_z=bz,az=az,dx=dx,dy=dy,dz=dz))
    return out

def v2_geom(snap, scale=10.0, tilt=60.0):
    """Reconstruct the direction vector that V2's add_lateral_branch produces.

    V2 convention (organ_params_loader._leaf_azimuth + add_lateral_branch):
      - azimuth passed to builder  = az_csv - 90°
      - tilt_angle passed to builder = angle_petiole   (0°=vertical, 90°=horizontal)
      - tilt_axis = [cos(az_v2), sin(az_v2), 0]  (tangent at azimuth)
      - direction  = rot(tilt_axis, -tilt_v2) * Z

    The --tilt CLI arg is a fallback for leaves where angle_petiole is genuinely
    absent from the model (not when it is 0, which is a valid value).
    """
    for n in snap.organs:
        if isinstance(n,InternodeNode): _base_z(n)
    out=[]
    for n in snap.organs:
        if not isinstance(n,LeafNode): continue
        bz = 0.0 if (n.parent is None or not isinstance(n.parent,InternodeNode)) else (_base_z(n.parent)+n.parent.length)*scale
        az = n.ccw_orientation if abs(getattr(n,'ccw_orientation',0))>1e-3 else (n.key.rank*PHYLLOTAXIS)%360
        # shift az by -90° to match _leaf_azimuth convention
        az_v2 = (az - 90.0) % 360.0
        # tilt_v2 == angle_petiole (both measured from horizontal: 90°=horizontal)
        # Use CLI --tilt only when angle_petiole attribute is truly missing.
        tilt_v2 = n.angle_petiole if hasattr(n,'angle_petiole') else tilt
        ax=[math.cos(math.radians(az_v2)),math.sin(math.radians(az_v2)),0.0]
        d=norm3(mv3(rot_axis_angle(ax,-tilt_v2),[0,0,1]))
        out.append(dict(id=f"o{n.key.order}_r{n.key.rank}_i{n.key.organ_index}",attach_z_m=bz/scale,az=az,dx=d[0],dy=d[1],dz=d[2]))
    return out

def compare(v1,v2,scale):
    v1d={d['id']:d for d in v1}; v2d={d['id']:d for d in v2}
    ids=sorted(set(v1d)|set(v2d),key=lambda s:[int(x[1:]) for x in s.split('_')])
    print(f"\n{'─'*140}")
    print(f"  {'Leaf ID':<28}{'V1 z':>9}{'V2 z':>9}{'dz':>8}  "
          f"{'V1 az':>7}{'V2 az':>7}  "
          f"{'V1 el°':>7}{'V2 el°':>7}  "
          f"{'V1 dx':>7}{'V1 dy':>7}{'V1 dz':>7}  "
          f"{'V2 dx':>7}{'V2 dy':>7}{'V2 dz':>7}  "
          f"{'|Δdir|':>7}")
    print(f"{'─'*140}")
    issues=[]
    for lid in ids:
        a,b=v1d.get(lid),v2d.get(lid)
        if a is None: print(f"  {lid} [missing v1]"); continue
        if b is None: print(f"  {lid} [missing v2]"); continue
        dz=b['attach_z_m']-a['attach_z']; flag=""
        if abs(dz)>0.005: flag+=" dz!"; issues.append(f"{lid} z off {dz:+.4f}m")
        da=abs(((b['az']-a['az'])+180)%360-180)
        if da>5: flag+=f" az{da:.0f}°!"; issues.append(f"{lid} az off {da:.1f}deg")
        # Elevation = angle above horizontal: asin(dz)
        el_v1=math.degrees(math.asin(max(-1.0,min(1.0,a['dz']))))
        el_v2=math.degrees(math.asin(max(-1.0,min(1.0,b['dz']))))
        # Direction vector angle
        dot=max(-1.0,min(1.0,a['dx']*b['dx']+a['dy']*b['dy']+a['dz']*b['dz']))
        dir_err=math.degrees(math.acos(dot))
        if dir_err>2.0: flag+=f" dir{dir_err:.0f}°!"; issues.append(f"{lid} dir off {dir_err:.1f}deg")
        print(f"  {lid:<28}{a['attach_z']:>9.4f}{b['attach_z_m']:>9.4f}{dz:>+8.4f}  "
              f"{a['az']:>7.1f}{b['az']:>7.1f}  "
              f"{el_v1:>7.1f}{el_v2:>7.1f}  "
              f"{a['dx']:>7.3f}{a['dy']:>7.3f}{a['dz']:>7.3f}  "
              f"{b['dx']:>7.3f}{b['dy']:>7.3f}{b['dz']:>7.3f}  "
              f"{dir_err:>7.1f}"
              +(f"  *** {flag.strip()}" if flag else ""))
    print(f"{'─'*140}")
    if issues:
        print(f"\n{len(issues)} issue(s):")
        for i in issues: print(f"  {i}")
    else:
        print("\nAll attachment heights and directions match.")
    print()

def main():
    p=argparse.ArgumentParser()
    p.add_argument("--day",type=int,default=1); p.add_argument("--plant",type=int,default=1)
    p.add_argument("--scale",type=float,default=10.0); p.add_argument("--tilt",type=float,default=60.0)
    p.add_argument("--csv",default=None); a=p.parse_args()
    csv=a.csv or os.path.join(PROJECT_ROOT,"data/simulation_output/dynamic_output/graphs",f"graph_day_{a.day}.csv")
    print(f"Loading day={a.day} plant={a.plant}")
    snap=load_snapshot(csv,day=a.day,plant_id=a.plant)
    v1=v1_geom(snap); v2=v2_geom(snap,a.scale,a.tilt)
    print(f"V1: {len(v1)} leaves  V2: {len(v2)} leaves")
    compare(v1,v2,a.scale)

if __name__=="__main__": main()
