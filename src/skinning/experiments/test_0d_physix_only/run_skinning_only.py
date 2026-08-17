"""
run_skinning_only.py — Esegue Test 0D: UsdSkel standalone

Apre lo stage generato da generate_skinning_only.py e riproduce
l'animazione UsdSkel in Isaac Sim. Nessuna fisica coinvolta.

Uso:
    ~/isaacsim/python.sh run_skinning_only.py
"""

import os
import sys

# SimulationApp DEVE essere inizializzato prima di importare pxr
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False, "width": 1280, "height": 720})

import omni.usd
import omni.timeline
from pxr import Gf, UsdGeom

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)
from generate_skinning_only import OUTPUT_USD, build_stage

def setup_camera(stage):
    cam = UsdGeom.Camera.Define(stage, "/World/Camera")
    xf  = UsdGeom.Xformable(cam.GetPrim())
    xf.ClearXformOpOrder()
    # Vista laterale per vedere la piegatura lungo Y
    xf.AddTranslateOp().Set(Gf.Vec3d(-0.35, 0.15, 0.35))
    xf.AddRotateXYZOp().Set(Gf.Vec3f(55.0, 0.0, -35.0))
    cam.CreateFocalLengthAttr().Set(18.0)

def main():
    print("=" * 65)
    print("  Test 0D — UsdSkel standalone (Animation Loop)")
    print("=" * 65)

    # Genera il file USDA chiamando la funzione dal file originale
    path = build_stage(OUTPUT_USD)
    
    ctx = omni.usd.get_context()
    ctx.open_stage(path)
    stage = ctx.get_stage()
    
    setup_camera(stage)
    
    # Avvia la riproduzione dell'animazione
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()

    print("\n[INFO] Isaac Sim avviato. L'animazione dovrebbe riprodursi in loop.")
    print("Chiudi la finestra Isaac Sim per terminare.\n")

    while simulation_app.is_running():
        simulation_app.update()
        
    simulation_app.close()

if __name__ == "__main__":
    main()
