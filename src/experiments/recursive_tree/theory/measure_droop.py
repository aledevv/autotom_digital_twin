"""
measure_droop.py

Measure branch tip droop when simulation starts in Isaac Sim.

Prerequisites:
    - USD file must be already generated: data/usd_models/recursive_tree.usda
    - Run generate_recursive_tree_usda.py first if needed

Run:
    ~/isaacsim/python.sh src/experiments/recursive_tree/measure_droop.py
"""

import os
import sys
import csv
import time
import numpy as np

# Bootstrap Isaac Sim FIRST
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

# NOW we can import pxr and Isaac modules
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Gf
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.prims import RigidPrim

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(SCRIPT_DIR))

# This only imports Python data, no pxr dependency
from tree_config import BRANCHES, BioConfig, scaled

# Paths
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(SCRIPT_DIR)))
USD_PATH = os.path.join(PROJECT_ROOT, "data", "usd_models", "recursive_tree.usda")
OUTPUT_DIR = os.path.join(PROJECT_ROOT, "data")
os.makedirs(OUTPUT_DIR, exist_ok=True)


def apply_physx_scene_settings(stage) -> None:
    scene_path = "/World/PhysicsScene"
    usd_scene = UsdPhysics.Scene.Define(stage, scene_path)
    usd_scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    usd_scene.CreateGravityMagnitudeAttr().Set(9.81)

    physx = PhysxSchema.PhysxSceneAPI.Apply(usd_scene.GetPrim())
    physx.CreateSolverTypeAttr().Set("TGS")
    physx.CreateTimeStepsPerSecondAttr().Set(480)
    physx.CreateEnableCCDAttr().Set(True)
    physx.CreateEnableStabilizationAttr().Set(True)
    physx.CreateEnableGPUDynamicsAttr().Set(True)
    physx.CreateBroadphaseTypeAttr().Set("MBP")


def apply_physx_articulation_settings(stage, stem_path: str) -> None:
    prim = stage.GetPrimAtPath(stem_path)
    art = PhysxSchema.PhysxArticulationAPI.Apply(prim)
    art.CreateSolverPositionIterationCountAttr().Set(64)
    art.CreateSolverVelocityIterationCountAttr().Set(8)
    art.CreateEnabledSelfCollisionsAttr().Set(False)
    art.CreateSleepThresholdAttr().Set(0.0)


def get_tip_link_path(branch_def: dict) -> str:
    """Return USD path of the last link (tip) of a branch."""
    bid = branch_def["id"]
    n_links = branch_def["n_links"]
    return f"/World/Stem/{bid}_Link_{n_links:02d}"


def measure_droop():
    print(f"[INFO] Loading USD: {USD_PATH}", flush=True)
    if not os.path.exists(USD_PATH):
        print(f"[ERROR] USD file not found. Run generate_recursive_tree_usda.py first.", flush=True)
        return

    # Open existing USD stage
    omni.usd.get_context().open_stage(USD_PATH)
    stage = omni.usd.get_context().get_stage()

    # Apply PhysX settings
    print("[INFO] Applying PhysX settings...", flush=True)
    apply_physx_scene_settings(stage)
    apply_physx_articulation_settings(stage, "/World/Stem")

    # Read initial positions from USD (before physics)
    print("[INFO] Capturing initial tip positions from USD...", flush=True)
    initial_positions = {}
    for b in BRANCHES:
        tip_path = get_tip_link_path(b)
        tip_xform = UsdGeom.Xform(stage.GetPrimAtPath(tip_path))
        if not tip_xform:
            print(f"[WARN] Tip link not found: {tip_path}")
            continue
        # Get translate op
        xform_ops = tip_xform.GetOrderedXformOps()
        if len(xform_ops) == 0:
            print(f"[WARN] No xform ops on {tip_path}")
            continue
        translate_op = xform_ops[0]  # first op is translate
        pos = translate_op.Get()
        z = float(pos[2])
        initial_positions[b["id"]] = z
        print(f"  {b['id']:<12} z_initial = {z:.6f} m", flush=True)

    # Create World and reset (starts physics)
    print("[INFO] Starting simulation...", flush=True)
    my_world = World(stage_units_in_meters=1.0)
    my_world.reset()

    # Build RigidPrim references for tip links
    print("[INFO] Initializing RigidPrims for tip tracking...", flush=True)
    tip_prims = {}
    for b in BRANCHES:
        tip_path = get_tip_link_path(b)
        tip_prims[b["id"]] = RigidPrim(tip_path)
    
    for prim in tip_prims.values():
        prim.initialize()

    # Settling phase - run 2000 silent steps
    SETTLING_STEPS = 2000
    print(f"[INFO] Running {SETTLING_STEPS} settling steps...", flush=True)
    for i in range(SETTLING_STEPS):
        my_world.step(render=False)
        if (i + 1) % 500 == 0:
            print(f"  ...step {i+1}/{SETTLING_STEPS}", flush=True)

    # Capture final positions
    print("[INFO] Capturing final tip positions (after settling)...", flush=True)
    final_positions = {}
    for bid, prim in tip_prims.items():
        pos, _ = prim.get_world_poses()
        z = float(np.squeeze(pos)[2])
        final_positions[bid] = z
        print(f"  {bid:<12} z_settled = {z:.6f} m", flush=True)

    # Compute droop
    print("\n[RESULTS]", flush=True)
    results = []
    for b in BRANCHES:
        bid = b["id"]
        z_init = initial_positions.get(bid)
        z_final = final_positions.get(bid)
        
        if z_init is None or z_final is None:
            print(f"  {bid:<12} SKIPPED (missing data)", flush=True)
            continue
            
        droop_m = z_init - z_final
        droop_mm = droop_m * 1000.0

        # Compute branch properties
        parent = b.get("parent") or "-"
        attach = b.get("attach_link") or "-"
        n_links = b["n_links"]
        length_m = n_links * scaled(b["height"])
        radius_m = scaled(b["radius"])
        tilt = b.get("tilt", 0.0)
        E = BioConfig.YOUNG_MODULUS

        print(f"  {bid:<12} droop = {droop_mm:>6.2f} mm  "
              f"(z: {z_init:.3f} → {z_final:.3f} m)", flush=True)

        results.append({
            "branch_id": bid,
            "parent": parent,
            "attach_link": attach,
            "n_links": n_links,
            "length_m": length_m,
            "radius_m": radius_m,
            "tilt_deg": tilt,
            "E_Pa": E,
            "z_initial_m": z_init,
            "z_settled_m": z_final,
            "droop_mm": droop_mm,
        })

    # Save CSV
    csv_path = os.path.join(OUTPUT_DIR, "droop_measurement.csv")
    with open(csv_path, "w", newline="") as f:
        fieldnames = ["branch_id", "parent", "attach_link", "n_links",
                      "length_m", "radius_m", "tilt_deg", "E_Pa",
                      "z_initial_m", "z_settled_m", "droop_mm"]
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(results)

    print(f"\n[OK] Results saved to: {csv_path}", flush=True)


if __name__ == "__main__":
    import traceback
    try:
        measure_droop()
    except Exception as e:
        print(f"\n[ERROR] {e}")
        traceback.print_exc()
    finally:
        simulation_app.close()
