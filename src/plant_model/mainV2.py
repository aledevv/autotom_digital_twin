"""
main_v2.py

UNICO entry-point per il flusso V2 (stelo articolato a segmenti):
1) bootstrap SimulationApp (deve stare prima di ogni import pxr/omni)
2) carica lo snapshot della pianta per un dato giorno (loader.py)
3) costruisce lo stage USD con lo stelo articolato (usd_exporterV2.py)
4) inietta PhysicsScene + PhysxArticulationAPI (physx_utils.py)
5) salva, apre in Isaac Sim, avvia il loop di simulazione

Run con:
~/isaacsim/python.sh <path>/main_v2.py --day 160 --plant 1
"""

import os
import sys
import argparse

SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
SRC_DIR      = os.path.dirname(SCRIPT_DIR)
PROJECT_ROOT = os.path.dirname(SRC_DIR)

if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

# ── Bootstrap Isaac Sim runtime — DEVE stare prima di ogni import pxr/omni ────
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import omni.usd
from isaacsim.core.api import World
import omni.kit.actions.core

from plant_model.loader import load_snapshot
from plant_model.usd_exporter_v2 import build_stem_stage
from plant_model.physx_utils import (
    apply_physx_scene_settings,
    apply_physx_articulation_settings,
)


def parse_args():
    parser = argparse.ArgumentParser(description="Costruisce e simula lo stelo articolato V2 in Isaac Sim.")
    parser.add_argument("--day", type=int, default=160, help="Giorno di simulazione.")
    parser.add_argument("--plant", type=int, default=1, help="ID della pianta.")
    parser.add_argument("--csv", default=None, help="Path CSV (default: risolto da day).")
    parser.add_argument("--out", default=None, help="Path .usda di output (default: risolto da day).")
    return parser.parse_args()


def main():
    args = parse_args()

    csv_path = args.csv or os.path.join(
        PROJECT_ROOT, "data/simulation_output/dynamic_output/graphs", f"graph_day_{args.day}.csv"
    )
    out_path = args.out or os.path.join(
        PROJECT_ROOT, "output", f"day_{args.day}", f"plant_day{args.day}_stem_v2.usda"
    )
    os.makedirs(os.path.dirname(out_path), exist_ok=True)

    print(f"[INFO] Carico snapshot day={args.day} plant={args.plant} da {csv_path}")
    snapshot = load_snapshot(csv_path, day=args.day, plant_id=args.plant)

    print("[INFO] Costruzione stage stelo articolato v2...")
    stage, stem_path = build_stem_stage(snapshot, out_path)

    print("[INFO] Iniezione config PhysX...")
    apply_physx_scene_settings(stage)
    apply_physx_articulation_settings(stage, stem_path)

    stage.GetRootLayer().Save()
    print(f"[OK] Stage salvato con config PhysX: {out_path}")

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