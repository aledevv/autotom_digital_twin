import os
import sys

# Setup paths
script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(os.path.dirname(script_dir))
sys.path.insert(0, project_root)

# Bootstrap SimulationApp
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Gf
from isaacsim.core.api import World

from src.plant_model.plant_builder import PlantBuilder

OUTPUT = os.path.join(project_root, "data", "usd_models", "builder_merging_test.usda")
BAKED_SCALE = 10.0

def setup_physx(stage, stem_path):
    sc = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    sc.CreateGravityDirectionAttr().Set(Gf.Vec3f(0, 0, -1))
    sc.CreateGravityMagnitudeAttr().Set(9.81)
    px = PhysxSchema.PhysxSceneAPI.Apply(sc.GetPrim())
    px.CreateSolverTypeAttr().Set("TGS")
    px.CreateTimeStepsPerSecondAttr().Set(120)
    px.CreateEnableStabilizationAttr().Set(True)

    art = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(stem_path))
    art.CreateSolverPositionIterationCountAttr().Set(64)
    art.CreateSolverVelocityIterationCountAttr().Set(8)
    art.CreateEnabledSelfCollisionsAttr().Set(False)
    art.CreateSleepThresholdAttr().Set(0.0)

class DummyInternode:
    def __init__(self, length, radius):
        self.length = length
        self.radius = radius
        self.world_base_z = 0.0

def build_merged_stem(builder: PlantBuilder, internodes: list[DummyInternode], max_segments: int):
    """
    Takes a list of biological internodes and merges them into at most max_segments physical segments.
    Returns a list of dictionaries with segment data (path, base_z, height) so leaves can be attached.
    """
    total_length = sum(n.length for n in internodes)
    target_seg_length = total_length / max_segments if max_segments > 0 else total_length
    
    physical_segments = []
    
    current_physical_len = 0.0
    current_physical_vol = 0.0
    
    # We will build segments as we iterate
    prev_id = None
    seg_idx = 1
    
    # Track the absolute Z of the biological internodes
    current_bio_z = 0.0
    for node in internodes:
        node.world_base_z = current_bio_z
        current_bio_z += node.length
        
    current_phys_base_z = 0.0
    
    for i, node in enumerate(internodes):
        current_physical_len += node.length
        # approximate volume sum to get average radius
        current_physical_vol += (node.radius ** 2) * node.length 
        
        # If we reached the target length for a segment, OR it's the last node
        if current_physical_len >= target_seg_length or i == len(internodes) - 1:
            # Calculate average radius for this physical segment
            avg_radius = (current_physical_vol / current_physical_len) ** 0.5 if current_physical_len > 0 else node.radius
            
            # Scale geometry
            scaled_len = current_physical_len * BAKED_SCALE
            scaled_rad = avg_radius * BAKED_SCALE
            
            seg_id = f"Stem_{seg_idx:03d}"
            
            # Artificial mass for stability
            mass = max(3.1415 * (scaled_rad**2) * scaled_len * 500.0, 0.05)
            
            if prev_id is None:
                # Root segment
                builder.create_root(seg_id, radius=scaled_rad, length=scaled_len, mass=mass)
            else:
                # Internode segment
                # Taper stiffness along the stem
                stiffness = max(500000.0 / (seg_idx), 1000.0)
                damping = max(100.0 / (seg_idx), 10.0)
                builder.add_internode(prev_id, seg_id, radius=scaled_rad, length=scaled_len,
                                      mass=mass, stiffness=stiffness, damping=damping)
            
            physical_segments.append({
                'id': seg_id,
                'path': builder._segments[seg_id]['path'],
                'base_z': current_phys_base_z,
                'height': scaled_len
            })
            
            current_phys_base_z += scaled_len
            prev_id = seg_id
            seg_idx += 1
            
            # Reset accumulators for next physical segment
            current_physical_len = 0.0
            current_physical_vol = 0.0
            
    return physical_segments

def main():
    if os.path.exists(OUTPUT):
        os.remove(OUTPUT)

    stage = Usd.Stage.CreateNew(OUTPUT)
    world_prim = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world_prim.GetPrim())
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

    builder = PlantBuilder(stage, "/World/Stem")
    
    # Let's create 100 biological internodes!
    # They will be very short, e.g., 2mm long each.
    # Total length = 0.2 meters (20 cm).
    bio_internodes = []
    for i in range(100):
        # taper the radius slightly
        r = max(0.005 - (i * 0.00003), 0.001)
        bio_internodes.append(DummyInternode(length=0.002, radius=r))
        
    print(f"Created {len(bio_internodes)} biological internodes.")
    
    # We want to merge them into a maximum of 10 physics segments to keep physics stable
    MAX_BUDGET = 10
    phys_segs = build_merged_stem(builder, bio_internodes, MAX_BUDGET)
    
    print(f"Merged into {len(phys_segs)} physical segments.")
    for seg in phys_segs:
        print(f" - {seg['id']}: height={seg['height']:.3f}m, base_z={seg['base_z']:.3f}m")
        
    setup_physx(stage, "/World/Stem")
    stage.GetRootLayer().Save()
    print(f"Saved USD to {OUTPUT}")
    
    print("Simulating for a bit...")
    omni.usd.get_context().open_stage(OUTPUT)
    w = World(stage_units_in_meters=1.0)
    w.reset()
    
    # Run a few steps to see if it explodes
    for _ in range(100):
        w.step(render=True)
        if not simulation_app.is_running():
            break
            
    print("Simulation stable and finished.")
    simulation_app.close()

if __name__ == "__main__":
    main()
