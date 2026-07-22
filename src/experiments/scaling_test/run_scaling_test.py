import math
import numpy as np
import sys
import os
import argparse
import json

# Hardcode headless to True for batch run
headless = True

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": headless})

from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Gf
import omni.usd
from isaacsim.core.api import World

script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(os.path.dirname(os.path.dirname(script_dir)))
sys.path.insert(0, project_root)

from src.plant_model.plant_builder import PlantBuilder

def get_true_mass(radius, length):
    vol = math.pi * (radius ** 2) * length
    return max(vol * 500.0, 1e-10)

def setup_physx(stage, stem_path):
    sc_path = "/World/PhysicsScene"
    if not stage.GetPrimAtPath(sc_path):
        sc = UsdPhysics.Scene.Define(stage, sc_path)
    else:
        sc = UsdPhysics.Scene(stage.GetPrimAtPath(sc_path))
    
    sc.CreateGravityDirectionAttr().Set(Gf.Vec3f(0, 0, -1))
    sc.CreateGravityMagnitudeAttr().Set(9.81)
    px = PhysxSchema.PhysxSceneAPI.Apply(sc.GetPrim())
    px.CreateSolverTypeAttr().Set("TGS")
    px.CreateTimeStepsPerSecondAttr().Set(120)
    px.CreateEnableStabilizationAttr().Set(True)
    
    prim = stage.GetPrimAtPath(stem_path)
    if prim and prim.HasAPI(PhysxSchema.PhysxArticulationAPI):
        art = PhysxSchema.PhysxArticulationAPI(prim)
        art.CreateSolverPositionIterationCountAttr().Set(64)
        art.CreateSolverVelocityIterationCountAttr().Set(8)
        art.CreateEnabledSelfCollisionsAttr().Set(False)
        art.CreateSleepThresholdAttr().Set(0.0)

def run_experiment(S, case):
    omni.usd.get_context().new_stage()
    stage = omni.usd.get_context().get_stage()
    builder = PlantBuilder(stage, "/World/Stem")
    
    if case == 'A':
        # Case A: 4 segments, long
        num_segments = 4
        base_len = 0.04
        base_rad = 0.005
        base_stiffness = 5.0
        base_damping = 0.5
    else:
        # Case B: 10 segments, short
        num_segments = 10
        base_len = 0.004
        base_rad = 0.002
        base_stiffness = 50.0
        base_damping = 5.0
        
    lb_rad = base_rad * S
    lb_len = base_len * S
    
    stiff_scale = base_stiffness * (S ** 4)
    damp_scale = base_damping * (S ** 4.5)
    
    # Root
    builder.create_root("T01", radius=lb_rad, length=0.001*S, mass=1.0*S**3)
    
    # Segments
    prev = builder.add_lateral_branch("T01", "LB01", 
                                      radius=lb_rad, length=lb_len,
                                      z_offset_ratio=1.0, tilt_angle=90, rot_around_parent=0,
                                      mass=get_true_mass(lb_rad, lb_len),
                                      stiffness=stiff_scale, damping=damp_scale)
                                      
    last_node = "LB01"
    for i in range(2, num_segments + 1):
        r = max(0.0001*S, lb_rad - i * (lb_rad / (num_segments * 1.5)))
        stiff = max(1e-5, stiff_scale - (i * (stiff_scale / num_segments)))
        damp = max(1e-5, damp_scale - (i * (damp_scale / num_segments)))
        
        last_node = f"LB{i:02d}"
        prev = builder.add_internode(prev, last_node, 
                                     radius=r, length=lb_len,
                                     mass=get_true_mass(r, lb_len), 
                                     stiffness=stiff, damping=damp)
                                     
    setup_physx(stage, "/World/Stem")
    
    world = World(stage_units_in_meters=1.0)
    world.reset()
    
    root_prim = stage.GetPrimAtPath("/World/Stem/T01")
    tip_prim = stage.GetPrimAtPath(f"/World/Stem/{last_node}")
    
    # Run simulation for 120 steps (1 second)
    for _ in range(120):
        world.step(render=not headless)
        
    # Measure
    root_xform = UsdGeom.Xformable(root_prim)
    tip_xform = UsdGeom.Xformable(tip_prim)
    
    root_pos = root_xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
    tip_pos = tip_xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
    
    total_length = num_segments * lb_len
    
    # Detect explosion
    status = "Stable"
    sag_angle = 0.0
    
    if math.isnan(tip_pos[0]) or math.isnan(tip_pos[1]) or math.isnan(tip_pos[2]):
        status = "Exploded (NaN)"
    else:
        dist = np.linalg.norm(np.array(tip_pos) - np.array(root_pos))
        if dist > total_length * 2.0:
            status = "Exploded (Position bounds)"
        else:
            dx = tip_pos[0] - root_pos[0]
            dy = tip_pos[1] - root_pos[1]
            dz = tip_pos[2] - root_pos[2]
            horiz_dist = math.sqrt(dx**2 + dy**2)
            sag_angle = math.degrees(math.atan2(-dz, horiz_dist))

    # Output JSON result
    result = {
        "scale": S,
        "case": case,
        "num_segments": num_segments,
        "segment_length_real": base_len,
        "total_length_sim": total_length,
        "stiffness_base": base_stiffness,
        "damping_base": base_damping,
        "sag_angle_deg": round(sag_angle, 2),
        "status": status
    }
    
    print("SCALING_TEST_RESULT:" + json.dumps(result))
    return result

if __name__ == "__main__":
    scales = [10.0, 5.0, 1.0, 0.5, 0.1]
    cases = ['A', 'B']
    
    all_results = []
    
    for S in scales:
        for case in cases:
            print(f"Running Case {case} at Scale {S}...")
            res = run_experiment(S, case)
            all_results.append(res)
            
    output_path = os.path.join(project_root, "docs", "scaling_results.json")
    with open(output_path, "w") as f:
        json.dump(all_results, f, indent=4)
    print(f"Saved results to {output_path}")
    
    simulation_app.close()
