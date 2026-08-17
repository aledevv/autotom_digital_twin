"""
run.py  —  Test 0B: UsdSkel runtime animation

Genera il USDA (via generate.py) con posa neutra, lo carica in Isaac Sim,
poi aggiorna i bones ogni frame con un'oscillazione sinusoidale:

    Bone1: θ1(t) = SWING_DEG · sin(ω · t)
    Bone2: θ2(t) = SWING_DEG · sin(ω · t + π/3)   [sfasato di 60°]

Tutto il batch update avviene in una sola chiamata .Set() per rotazioni
(non loop bone-per-bone), come da raccomandazione del piano.

Success criteria Test 0B:
  - Mesh oscilla continuamente senza gap
  - Nessun conflitto tra evaluator USD e aggiornamento manuale
  - Hydra aggiorna la mesh ogni frame (no freeze)
  - Nessun twist inatteso o artefatto di volume evidente

Uso:
    ~/isaacsim/python.sh run.py
"""

import math
import os
import sys
import time

# ── Isaac Sim init ────────────────────────────────────────────────────────────
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False, "width": 1280, "height": 720})

# ── Post-init imports ─────────────────────────────────────────────────────────
import omni.usd
from isaacsim.core.api import World
from pxr import Gf, Usd, UsdGeom, UsdSkel, Vt

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)
import generate
from generate import OUTPUT_USD, BONE_LENGTH, NUM_BONES, build_stage

# ─────────────────────────────────────────────────────────────────────────────
# PARAMETRI ANIMAZIONE
# ─────────────────────────────────────────────────────────────────────────────

OMEGA_RAD_S  = 1.5        # velocità angolare (rad/s) — ~0.24 Hz
SWING_DEG    = 30.0       # ampiezza massima oscillazione (gradi)
PHASE_BONE2  = math.pi / 3.0  # sfasamento Bone2 rispetto Bone1 (60°)

# ─────────────────────────────────────────────────────────────────────────────
# HELPERS
# ─────────────────────────────────────────────────────────────────────────────

def rot_x_quatf(deg: float) -> Gf.Quatf:
    """Quaternione per rotazione attorno all'asse X locale."""
    rad = deg * math.pi / 180.0
    c   = math.cos(rad / 2.0)
    s   = math.sin(rad / 2.0)
    return Gf.Quatf(float(c), Gf.Vec3f(float(s), 0.0, 0.0))


IDENTITY_QUATF = Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0))

# Traslazioni locali: fisse per tutta la simulazione
# Bone0: root → (0,0,0), Bone1 e Bone2: (0,0,BONE_LENGTH) in spazio locale
STATIC_TRANSLATIONS = Vt.Vec3fArray([
    Gf.Vec3f(0.0, 0.0, 0.0),
    Gf.Vec3f(0.0, 0.0, float(BONE_LENGTH)),
    Gf.Vec3f(0.0, 0.0, float(BONE_LENGTH)),
])


# ─────────────────────────────────────────────────────────────────────────────

def setup_camera(stage):
    cam = UsdGeom.Camera.Define(stage, "/World/Camera")
    xf  = UsdGeom.Xformable(cam.GetPrim())
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(0.50, -0.40, 0.20))
    xf.AddRotateXYZOp().Set(Gf.Vec3f(72.0, 0.0, 50.0))
    cam.CreateFocalLengthAttr().Set(28.0)
    return str(cam.GetPath())


def get_anim_attrs(stage):
    """Ritorna gli attributi rotations e translations della SkelAnimation."""
    anim_prim = stage.GetPrimAtPath("/World/SkelRoot/SkelAnim")
    if not anim_prim.IsValid():
        raise RuntimeError("[ERROR] /World/SkelRoot/SkelAnim non trovato")
    anim = UsdSkel.Animation(anim_prim)
    rot_attr   = anim.GetRotationsAttr()
    trans_attr = anim.GetTranslationsAttr()
    return rot_attr, trans_attr


# ─────────────────────────────────────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────────────────────────────────────

def main():
    print()
    print("=" * 65)
    print("  Test 0B — UsdSkel Runtime Animation")
    print("=" * 65)
    print(f"  Oscillazione: ±{SWING_DEG}° a {OMEGA_RAD_S:.1f} rad/s")
    print(f"  Sfasamento Bone2: {math.degrees(PHASE_BONE2):.0f}°")
    print()

    # 1. Genera USDA
    print("[STEP 1/3] Generazione USDA ...")
    path = build_stage(OUTPUT_USD)
    print(f"  [OK] {path}")

    # 2. Apri stage
    print("[STEP 2/3] Apertura stage ...")
    ctx = omni.usd.get_context()
    ctx.open_stage(path)

    if World.instance() is not None:
        World.instance().clear_instance()
    my_world = World(stage_units_in_meters=1.0)
    my_world.reset()

    stage = ctx.get_stage()
    setup_camera(stage)

    # 3. Recupera attributi animazione
    print("[STEP 3/3] Avvio loop animazione ...")
    rot_attr, trans_attr = get_anim_attrs(stage)

    # Scrivi le traduzioni statiche una volta sola
    trans_attr.Set(STATIC_TRANSLATIONS)

    print()
    print("  Success criteria Test 0B:")
    print("    [ ] Mesh oscilla continuamente (no freeze)")
    print("    [ ] Nessun gap durante la deformazione")
    print("    [ ] Hydra aggiorna la mesh ogni frame")
    print("    [ ] Nessun twist inatteso o perdita di volume evidente")
    print()
    print("  Chiudi la finestra Isaac Sim per uscire.")
    print("=" * 65)

    # ── Loop render + animazione ──────────────────────────────────────────────
    t_start = time.time()
    step    = 0
    last_log_step = 0

    while simulation_app.is_running():
        t = time.time() - t_start

        # Calcola angoli correnti
        angle1 = SWING_DEG * math.sin(OMEGA_RAD_S * t)
        angle2 = SWING_DEG * math.sin(OMEGA_RAD_S * t + PHASE_BONE2)

        # Batch update: una sola chiamata .Set() per le rotazioni
        rotations = Vt.QuatfArray([
            IDENTITY_QUATF,          # Bone0: fisso
            rot_x_quatf(angle1),     # Bone1: oscilla
            rot_x_quatf(angle2),     # Bone2: oscilla con sfasamento
        ])
        rot_attr.Set(rotations)

        my_world.step(render=True)
        step += 1

        # Log ogni 5 secondi circa (a ~60fps = 300 step)
        if step - last_log_step >= 300:
            print(f"  [frame {step:6d}] t={t:.1f}s  "
                  f"Bone1={angle1:+6.1f}°  Bone2={angle2:+6.1f}°")
            last_log_step = step

    elapsed = time.time() - t_start
    fps = step / elapsed if elapsed > 0 else 0
    print(f"\n[INFO] Simulazione terminata — {step} frames in {elapsed:.1f}s "
          f"({fps:.1f} fps medio)")
    print()
    print("  Risultati da verificare:")
    print("    [ ] Mesh oscillava senza gap")
    print("    [ ] Hydra aggiornava ogni frame (fps stabile)")
    print("    [ ] Nessun conflitto USD evaluator")

    simulation_app.close()


if __name__ == "__main__":
    main()
