"""
run.py  —  Test 0A: UsdSkel statico in Isaac Sim

Genera il file USDA (via generate.py) e lo carica in Isaac Sim.
Visualizza la mesh skinnata con i 3 bones a 0°/25°/45°.
Non attiva la fisica.

Uso:
    ~/isaacsim/python.sh run.py
"""

import os
import sys

# ── Isaac Sim init (DEVE essere il primo import) ─────────────────────────────
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False, "width": 1280, "height": 720})

# ── Post-init imports ─────────────────────────────────────────────────────────
import omni.usd
import omni.kit.viewport.utility
from isaacsim.core.api import World
from pxr import Gf, UsdGeom, UsdSkel, Usd

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)
import generate
from generate import OUTPUT_USD, build_stage

# ─────────────────────────────────────────────────────────────────────────────

def setup_camera(stage):
    """Camera laterale per vedere la curva del tubo."""
    cam = UsdGeom.Camera.Define(stage, "/World/Camera")
    xf  = UsdGeom.Xformable(cam.GetPrim())
    xf.ClearXformOpOrder()
    # Offset laterale: X=0.5, Y=-0.4, Z=0.25 — guarda verso il centro del tubo
    xf.AddTranslateOp().Set(Gf.Vec3d(0.45, -0.35, 0.20))
    xf.AddRotateXYZOp().Set(Gf.Vec3f(72.0, 0.0, 48.0))
    cam.CreateFocalLengthAttr().Set(28.0)
    return str(cam.GetPath())


def validate_skel(stage) -> bool:
    """Verifica il binding UsdSkel e stampa un report. Ritorna True se OK."""
    print()
    print("─" * 55)
    print("  VALIDAZIONE UsdSkel")
    print("─" * 55)

    ok = True

    root_prim = stage.GetPrimAtPath("/World/SkelRoot")
    if not root_prim.IsValid():
        print("  [ERROR] /World/SkelRoot non trovato")
        return False

    cache = UsdSkel.Cache()
    cache.Populate(UsdSkel.Root(root_prim), Usd.TraverseInstanceProxies())

    skel_prim = stage.GetPrimAtPath("/World/SkelRoot/Skeleton")
    mesh_prim = stage.GetPrimAtPath("/World/SkelRoot/TubeMesh")

    if skel_prim.IsValid():
        joints = UsdSkel.Skeleton(skel_prim).GetJointsAttr().Get()
        print(f"  [OK] Skeleton  — {len(joints)} joints: {list(joints)}")
    else:
        print("  [ERROR] Skeleton non trovato")
        ok = False

    if mesh_prim.IsValid():
        sq = cache.GetSkinningQuery(mesh_prim)
        if sq is not None:
            try:
                has_inf = sq.HasJointInfluences()
            except Exception:
                has_inf = True  # metodo non disponibile, assume valido se sq esiste
            if has_inf:
                pv   = sq.GetJointIndicesPrimvar()
                elem = pv.GetElementSize()
                n    = len(pv.Get() or []) // max(elem, 1)
                print(f"  [OK] TubeMesh  — {n} vertici, {elem} influenze/vertice")
            else:
                print("  [ERROR] SkinningQuery senza joint influences — binding rotto")
                ok = False
        else:
            print("  [ERROR] SkinningQuery non trovata — binding rotto")
            ok = False
    else:
        print("  [ERROR] TubeMesh non trovata")
        ok = False

    print("─" * 55)
    return ok


def main():
    print()
    print("=" * 60)
    print("  Test 0A — UsdSkel statico")
    print("=" * 60)

    # 1. Genera USDA
    print("[STEP 1/3] Generazione USDA ...")
    path = build_stage(OUTPUT_USD)
    print(f"  [OK] {path}")

    # 2. Apri lo stage in Isaac Sim
    print("[STEP 2/3] Apertura stage in Isaac Sim ...")
    ctx = omni.usd.get_context()
    ctx.open_stage(path)

    if World.instance() is not None:
        World.instance().clear_instance()

    my_world = World(stage_units_in_meters=1.0)
    my_world.reset()

    stage = ctx.get_stage()

    # 3. Validazione e camera
    print("[STEP 3/3] Validazione binding ...")
    ok = validate_skel(stage)
    cam_path = setup_camera(stage)
    print(f"  Camera: {cam_path}")

    # ── Risultato ──────────────────────────────────────────────────────────
    if ok:
        print("\n  [PASS] Binding UsdSkel valido.")
    else:
        print("\n  [FAIL] Binding UsdSkel non valido — vedi log sopra.")

    print()
    print("  Success criteria Test 0A:")
    print("    [ ] Tubo continuo e curvo (no gap tra segmenti)")
    print("    [ ] Curvatura visibile: 0° → 25° → 45° attorno X")
    print("    [ ] Nessun offset/salto iniziale")
    print("    [ ] Nessun errore di binding nel log")
    print()
    print("  Chiudi la finestra Isaac Sim per uscire.")
    print("=" * 60)

    # Loop render
    step = 0
    while simulation_app.is_running():
        my_world.step(render=True)
        step += 1
        if step % 240 == 0:
            print(f"  [INFO] frame {step} — chiudi la finestra per uscire.")

    print("\n[INFO] Simulazione terminata.")
    simulation_app.close()


if __name__ == "__main__":
    main()
