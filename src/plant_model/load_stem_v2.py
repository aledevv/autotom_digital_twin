"""
load_stem_v2.py

STEP 1 del piano incrementale: carica i dati reali della pianta per un
determinato giorno, costruisce SOLO lo stelo principale articolato a
segmenti (plant_model.usd_exporterV2.build_stem_stage), inietta la
PhysicsScene + PhysxArticulationAPI e avvia la simulazione in Isaac Sim.

Stessa struttura/pattern di load_articulation_subbranch.py:
    1) costruzione dello stage con funzioni "pure" (nessun import di isaacsim/omni
       dentro usd_exporterV2.py)
    2) iniezione della config PhysX (disponibile solo qui, dentro SimulationApp)
    3) salvataggio dello stage completo
    4) apertura in Isaac Sim e avvio del loop di simulazione

Run con:
    ~/isaacsim/python.sh load_stem_v2.py --csv data/simulation_output/dynamic_output/graphs/graph_day_160.csv --day 160 --plant 1

Nota: questo script assume che il pacchetto `plant_model` (models.py,
constants.py, usd_helpers.py, loader.py, usd_exporterV2.py) sia importabile,
cioe' che venga lanciato dalla root del progetto (dove vive anche main.py).
"""

import os
import sys
import argparse

SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
SRC_DIR      = os.path.dirname(SCRIPT_DIR)
PROJECT_ROOT = os.path.dirname(SRC_DIR)

if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from pxr import UsdPhysics, PhysxSchema, Gf
import omni.usd
from isaacsim.core.api import World
import omni.kit.actions.core

from plant_model.loader import load_snapshot
from plant_model.usd_exporter_v2 import build_stem_stage


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args():
    parser = argparse.ArgumentParser(description="Carica lo stelo articolato v2 (solo stelo) in Isaac Sim.")
    parser.add_argument("--csv", default="data/simulation_output/dynamic_output/graphs/graph_day_160.csv",
                         help="Path al CSV del giorno di simulazione da caricare.")
    parser.add_argument("--day", type=int, default=160, help="Giorno di simulazione.")
    parser.add_argument("--plant", type=int, default=1, help="ID della pianta.")
    parser.add_argument("--out", default=None, help="Path di output .usda (default: ./output/day_<day>/plant_day<day>_stem_v2.usda)")
    return parser.parse_args()


# ---------------------------------------------------------------------------
# PhysX config (identica a load_articulation_subbranch.py)
# ---------------------------------------------------------------------------

def apply_physx_scene_settings(stage) -> None:
    """Crea/configura la PhysicsScene con parametri PhysX adatti a drive rigidi."""
    scene_path = "/World/PhysicsScene"
    usd_scene = UsdPhysics.Scene.Define(stage, scene_path)
    usd_scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    usd_scene.CreateGravityMagnitudeAttr().Set(9.81)

    physx_scene_api = PhysxSchema.PhysxSceneAPI.Apply(usd_scene.GetPrim())
    physx_scene_api.CreateSolverTypeAttr().Set("TGS")
    physx_scene_api.CreateTimeStepsPerSecondAttr().Set(120)
    physx_scene_api.CreateEnableCCDAttr().Set(True)
    physx_scene_api.CreateEnableStabilizationAttr().Set(True)
    physx_scene_api.CreateEnableGPUDynamicsAttr().Set(True)
    physx_scene_api.CreateBroadphaseTypeAttr().Set("MBP")


def apply_physx_articulation_settings(stage, stem_path: str) -> None:
    """Configura le iterazioni del solver sull'ArticulationRoot per stabilita' con drive rigidi."""
    stem_prim = stage.GetPrimAtPath(stem_path)
    physx_art_api = PhysxSchema.PhysxArticulationAPI.Apply(stem_prim)
    physx_art_api.CreateSolverPositionIterationCountAttr().Set(64)
    physx_art_api.CreateSolverVelocityIterationCountAttr().Set(8)
    physx_art_api.CreateEnabledSelfCollisionsAttr().Set(False)
    physx_art_api.CreateSleepThresholdAttr().Set(0.0)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    args = parse_args()
    out_path = args.out or os.path.join(
        SCRIPT_DIR, "output", f"day_{args.day}", f"plant_day{args.day}_stem_v2.usda"
    )
    os.makedirs(os.path.dirname(out_path), exist_ok=True)

    # -----------------------------------------------------------------------
    # 1. Carica i dati reali della pianta e costruisce lo stage (funzioni pure)
    # -----------------------------------------------------------------------
    print(f"[INFO] Carico snapshot day={args.day} plant={args.plant} da {args.csv}")
    snapshot = load_snapshot(args.csv, day=args.day, plant_id=args.plant)

    print("[INFO] Costruzione stage stelo articolato v2...")
    stage, stem_path = build_stem_stage(snapshot, out_path)

    # -----------------------------------------------------------------------
    # 2. Inietta la config PhysX (disponibile solo qui, dentro SimulationApp)
    # -----------------------------------------------------------------------
    apply_physx_scene_settings(stage)
    apply_physx_articulation_settings(stage, stem_path)

    # -----------------------------------------------------------------------
    # 3. Salva lo stage completo (geometria + fisica + PhysX) per riuso/debug
    # -----------------------------------------------------------------------
    stage.GetRootLayer().Save()
    print(f"[OK] Stage generato e salvato con config PhysX: {out_path}")

    # -----------------------------------------------------------------------
    # 4. Apre lo stage nel contesto di Isaac Sim e avvia la simulazione
    # -----------------------------------------------------------------------
    omni.usd.get_context().open_stage(out_path)
    print(f"[OK] Stage aperto in Isaac Sim: {out_path}")

    try:
        action_registry = omni.kit.actions.core.get_action_registry()
        action = action_registry.get_action("omni.kit.viewport.menubar.lighting", "set_lighting_mode_camera")
        if action:
            action.execute()
    except Exception as e:
        print(f"[WARN] Lighting non impostato: {e}")

    my_world = World(stage_units_in_meters=1.0)
    my_world.reset()
    print("[OK] Simulazione avviata — chiudi la finestra per uscire.")

    while simulation_app.is_running():
        my_world.step(render=True)

    print("Simulazione terminata.")
    simulation_app.close()


if __name__ == "__main__":
    main()
