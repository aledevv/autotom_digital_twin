"""Find the weight that breaks an external FixedJoint, by bisection on hanging mass.

One process per configuration (scene solver settings are fixed per process).
Prediction from PhysX 5.9 source reading (artiConstraintPrep2.cu, TGS joint prep
using stepDt): GPU + TGS + joint on an articulation link breaks at about
breakForce / position_iterations; the controls break near breakForce.
Run with ~/isaacsim/python.sh, e.g.:
  python.sh src/experiments/break_threshold_probe/probe_break_threshold.py --solver TGS --gpu 1 --iterations 32 --support articulation
"""
import argparse
import json
import math
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--solver", choices=["TGS", "PGS"], required=True)
parser.add_argument("--gpu", type=int, choices=[0, 1], required=True)
parser.add_argument("--iterations", type=int, default=32)
parser.add_argument("--support", choices=["articulation", "kinematic"], default="articulation")
parser.add_argument("--break-force", type=float, default=6.0)
parser.add_argument("--hz", type=int, default=60)
parser.add_argument("--duration", type=float, default=2.0)
parser.add_argument("--min-mass", type=float, default=0.002)
parser.add_argument("--max-mass", type=float, default=3.0)
parser.add_argument("--steps", type=int, default=9, help="bisection steps")
parser.add_argument("--output", type=Path, required=True, help="jsonl file to append to")
args = parser.parse_args()

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

import omni.timeline
import omni.usd
from omni.physx import get_physx_interface
from omni.physx.bindings._physx import SimulationEvent
from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics

G = 9.81
RADIUS = 0.02


def body(stage, path, pos, mass, kinematic=False):
    prim = UsdGeom.Xform.Define(stage, path).GetPrim()
    UsdGeom.Xformable(prim).AddTranslateOp().Set(Gf.Vec3d(*pos))
    api = UsdPhysics.RigidBodyAPI.Apply(prim)
    if kinematic:
        api.CreateKinematicEnabledAttr(True)
    m = UsdPhysics.MassAPI.Apply(prim)
    m.CreateMassAttr(mass)
    inertia = 0.4 * mass * RADIUS ** 2
    m.CreateDiagonalInertiaAttr(Gf.Vec3f(inertia, inertia, inertia))
    m.CreateCenterOfMassAttr(Gf.Vec3f(0, 0, 0))
    px = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    px.CreateSolverPositionIterationCountAttr(args.iterations)
    px.CreateSolverVelocityIterationCountAttr(1)
    return prim


def fixed(stage, path, b0, b1, local0, break_force=None):
    joint = UsdPhysics.FixedJoint.Define(stage, path)
    if b0:
        joint.CreateBody0Rel().SetTargets([b0])
    joint.CreateBody1Rel().SetTargets([b1])
    joint.CreateLocalPos0Attr(Gf.Vec3f(*local0))
    joint.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    joint.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
    joint.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    if break_force is not None:
        joint.CreateBreakForceAttr(break_force)
        joint.CreateBreakTorqueAttr(1e9)
        joint.CreateExcludeFromArticulationAttr(True)
    return joint


def build(mass):
    ctx = omni.usd.get_context()
    ctx.new_stage()
    stage = ctx.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    scene = UsdPhysics.Scene.Define(stage, "/World/Physics")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr(G)
    px = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    px.CreateSolverTypeAttr(args.solver)
    px.CreateEnableGPUDynamicsAttr(bool(args.gpu))
    px.CreateBroadphaseTypeAttr("GPU" if args.gpu else "MBP")
    px.CreateTimeStepsPerSecondAttr(args.hz)
    if args.support == "articulation":
        art = UsdGeom.Xform.Define(stage, "/World/Art").GetPrim()
        UsdPhysics.ArticulationRootAPI.Apply(art)
        PhysxSchema.PhysxArticulationAPI.Apply(art).CreateSolverPositionIterationCountAttr(args.iterations)
        PhysxSchema.PhysxArticulationAPI.Apply(art).CreateSolverVelocityIterationCountAttr(1)
        PhysxSchema.PhysxArticulationAPI.Apply(art).CreateEnabledSelfCollisionsAttr(False)
        body(stage, "/World/Art/link0", (0, 0, 1), 0.05)
        body(stage, "/World/Art/link1", (0.1, 0, 1), 0.05)
        fixed(stage, "/World/Art/link0/root_to_world", None, "/World/Art/link0", (0, 0, 1))
        fixed(stage, "/World/Art/link1/link0_link1", "/World/Art/link0", "/World/Art/link1", (0.1, 0, 0))
        support = "/World/Art/link1"
    else:
        body(stage, "/World/support", (0.1, 0, 1), 0.05, kinematic=True)
        support = "/World/support"
    body(stage, "/World/fruit", (0.1, 0, 0.9), mass)
    fixed(stage, "/World/fruit_joint", support, "/World/fruit", (0, 0, -0.1), args.break_force)
    return stage


def breaks(mass):
    stage = build(mass)
    state = {"broken": False}

    def on_event(event):
        if event.type == int(SimulationEvent.JOINT_BREAK):
            state["broken"] = True

    sub = get_physx_interface().get_simulation_event_stream_v2().create_subscription_to_pop(on_event)
    timeline = omni.timeline.get_timeline_interface()
    timeline.set_target_framerate(args.hz)
    timeline.play()
    for _ in range(int(args.duration * args.hz)):
        app.update()
        if state["broken"]:
            break
    timeline.stop()
    app.update()
    del sub
    return state["broken"]


lo, hi = args.min_mass, args.max_mass
broke_hi, broke_lo = breaks(hi), breaks(lo)
history = [(lo, broke_lo), (hi, broke_hi)]
if broke_lo or not broke_hi:
    result_mass = None
else:
    for _ in range(args.steps):
        mid = math.sqrt(lo * hi)
        outcome = breaks(mid)
        history.append((mid, outcome))
        lo, hi = (lo, mid) if outcome else (mid, hi)
    result_mass = math.sqrt(lo * hi)

weight = None if result_mass is None else result_mass * G
record = dict(solver=args.solver, gpu=bool(args.gpu), iterations=args.iterations, support=args.support,
              break_force_n=args.break_force, hz=args.hz, threshold_mass_kg=result_mass,
              threshold_weight_n=weight, bracket_kg=[lo, hi],
              ratio_to_break_force=None if weight is None else weight / args.break_force,
              prediction_break_force_over_n=args.break_force / args.iterations,
              history=history)
args.output.parent.mkdir(parents=True, exist_ok=True)
with args.output.open("a") as f:
    f.write(json.dumps(record) + "\n")
print("[PROBE]", json.dumps({k: v for k, v in record.items() if k != "history"}), flush=True)
app.close()
