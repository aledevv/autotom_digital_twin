import os
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
import omni.usd
from pxr import Usd, UsdPhysics, PhysxSchema

stage = omni.usd.get_context().open_stage("src/skinning/experiments/test_6a_curved_dynamic_pedicel/output/03_gravity_elbow_pedicels.usda")
stage = omni.usd.get_context().get_stage()

# Disable collisions on all pedicels and tomatoes
for prim in stage.Traverse():
    if prim.IsA(UsdPhysics.CollisionAPI):
        path_str = str(prim.GetPath())
        if "pedicel" in path_str.lower() or "tomato" in path_str.lower() or "fruit" in path_str.lower():
            col_api = UsdPhysics.CollisionAPI(prim)
            col_api.CreateCollisionEnabledAttr().Set(False)

stage.GetRootLayer().Save()
simulation_app.close()
