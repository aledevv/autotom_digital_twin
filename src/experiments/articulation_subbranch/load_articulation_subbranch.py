"""
load_articulation_subbranch.py

Carica il file USD generato da generate_articulation_usda.py in Isaac Sim.
Variante isolata di load_plant.py — non tocca la codebase principale.

Esegui con:
    ~/isaacsim/python.sh src/experiments/articulation_subbranch/load_articulation_subbranch.py
"""

import os
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import omni.usd
from isaacsim.core.api import World
from isaacsim.core.utils.stage import open_stage
import omni.kit.actions.core

# ---------------------------------------------------------------------------
# Percorsi
# ---------------------------------------------------------------------------
SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(SCRIPT_DIR)))
USD_PATH     = os.path.join(PROJECT_ROOT, "data", "usd_models", "generated_subbranch.usda")

# ---------------------------------------------------------------------------
# Apri lo stage direttamente (il file USD è già self-contained con World)
# ---------------------------------------------------------------------------
if not os.path.exists(USD_PATH):
    print(f"[ERROR] File USD non trovato: {USD_PATH}")
    print("        Esegui prima generate_articulation_usda.py")
    simulation_app.close()
    raise SystemExit(1)

open_stage(usd_path=USD_PATH)
print(f"[OK] Stage aperto: {USD_PATH}")

# ---------------------------------------------------------------------------
# Luce di default
# ---------------------------------------------------------------------------
try:
    action_registry = omni.kit.actions.core.get_action_registry()
    action = action_registry.get_action(
        "omni.kit.viewport.menubar.lighting", "set_lighting_mode_camera"
    )
    if action:
        action.execute()
except Exception as e:
    print(f"[WARN] Luce non impostata: {e}")

# ---------------------------------------------------------------------------
# World e loop di simulazione
# ---------------------------------------------------------------------------
my_world = World(stage_units_in_meters=1.0)
my_world.reset()
print("[OK] Simulazione avviata — chiudi la finestra per uscire.")

while simulation_app.is_running():
    my_world.step(render=True)

print("Simulazione terminata.")
simulation_app.close()
