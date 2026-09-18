"""Surface-aligned illustrative leaf rig; no live physics stepping."""

import argparse
import json
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--headless", action="store_true")
parser.add_argument("--capture", type=Path)
parser.add_argument(
    "--mode", choices=("surface", "segments", "joints"), default="joints"
)
parser.add_argument("--time", type=float, default=0.0)
args = parser.parse_args()

from isaacsim import SimulationApp

app = SimulationApp({"headless": args.headless, "width": 1280, "height": 720})
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.utils.viewports import set_camera_view
from model_view_geometry import connected_poses, surface_pivots
from omni import ui
from pxr import Gf, UsdGeom, UsdLux, UsdPhysics, UsdSkel, Vt
from soft_leaf import build_soft

from model import rotation

source = ROOT / "artifacts/leaf61/soft-leaf-base-hinge-contact"
row = json.loads((source / "layout.json").read_text())[0]
meshes = np.load(source / "meshes.npz")
archive = np.load(source / "poses.npz")
poses, times = archive["poses"], archive["times"]
points, faces, centers = [meshes[k + "_0"] for k in ("points", "faces", "centers")]
world = World(stage_units_in_meters=1)
stage = world.stage
_, _, anim, _, _ = build_soft(
    world, SimpleNamespace(**row["config"]), points, faces, "/World/Model"
)
# This is a new presentation rig, not a replay of the old physical translations.
old_centers = centers.copy()
pivots = surface_pivots(points, faces, np.arange(7) * row["config"]["length"] / 7)
centers = centers.copy()
centers[1:] = surface_pivots(points, faces, centers[1:, 0])
centers[0, 2] = pivots[0, 2]
for j in range(8):
    path = "/World/Model/" + ("Petiole" if j == 0 else f"Link{j - 1}")
    UsdGeom.Xformable(stage.GetPrimAtPath(path)).GetOrderedXformOps()[0].Set(
        Gf.Vec3d(*centers[j])
    )
    if j:
        mesh = UsdGeom.Mesh(stage.GetPrimAtPath(path + "/Collider"))
        cp = np.asarray(mesh.GetPointsAttr().Get()) + old_centers[j] - centers[j]
        mesh.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(cp.astype(np.float32)))
for j, pivot in enumerate(pivots):
    joint = UsdPhysics.Joint(stage.GetPrimAtPath(f"/World/Model/Joint{j}"))
    joint.GetLocalPos0Attr().Set(Gf.Vec3f(*(pivot - centers[j])))
    joint.GetLocalPos1Attr().Set(Gf.Vec3f(*(pivot - centers[j + 1])))
UsdPhysics.FixedJoint(
    stage.GetPrimAtPath("/World/Model/FixedPetiole")
).GetLocalPos0Attr().Set(Gf.Vec3f(*centers[0]))
skeleton = UsdSkel.Skeleton(stage.GetPrimAtPath("/World/Model/Leaf/Skeleton"))
binds = Vt.Matrix4dArray([Gf.Matrix4d().SetTranslate(Gf.Vec3d(*v)) for v in centers])
skeleton.GetBindTransformsAttr().Set(binds)
skeleton.GetRestTransformsAttr().Set(binds)
world.pause()
light = UsdLux.DomeLight.Define(stage, "/World/Light")
light.CreateIntensityAttr().Set(1200)
colors = [
    (0.35, 0.12, 0.55),
    (0.23, 0.3, 0.65),
    (0.12, 0.45, 0.63),
    (0.1, 0.6, 0.55),
    (0.2, 0.7, 0.42),
    (0.5, 0.8, 0.25),
    (0.8, 0.88, 0.12),
]
link_ops, markers = [], []
UsdGeom.Xform.Define(stage, "/World/JointMarkers")
for j in range(7):
    prim = stage.GetPrimAtPath(f"/World/Model/Link{j}")
    link_ops.append(UsdGeom.Xformable(prim).GetOrderedXformOps())
    UsdGeom.Mesh(
        stage.GetPrimAtPath(f"/World/Model/Link{j}/Collider")
    ).CreateDisplayColorAttr().Set([Gf.Vec3f(*colors[j])])
    sphere = UsdGeom.Sphere.Define(stage, f"/World/JointMarkers/J{j}")
    sphere.CreateRadiusAttr().Set(0.0012)
    sphere.CreateDisplayColorAttr().Set([Gf.Vec3f(1, 0.12, 0.05)])
    markers.append(sphere.AddTranslateOp())
support = UsdGeom.Cube.Define(stage, "/World/FixedSupport")
support.AddTranslateOp().Set(Gf.Vec3d(-0.006, 0, -0.003))
support.AddScaleOp().Set(Gf.Vec3f(0.005, 0.004, 0.004))
support.CreateDisplayColorAttr().Set([Gf.Vec3f(0.35)])

rot = rotation(poses[..., [6, 3, 4, 5]].reshape(-1, 4)).reshape(*poses.shape[:2], 3, 3)
host_rot = rot[:, row["host_id"]]
frame = host_rot @ np.asarray(row["host_frame"])
anchor = poses[:, row["host_id"], :3] + np.einsum("tij,j->ti", host_rot, row["anchor"])
positions = np.einsum("tji,tkj->tki", frame, poses[:, row["ids"], :3] - anchor[:, None])
rotations = np.einsum("tji,tkjl->tkil", frame, rot[:, row["ids"]])
positions = connected_poses(rotations, centers, pivots)
quats = [
    [
        Gf.Quatf(Gf.Matrix3d(*r.T.ravel().tolist()).ExtractRotation().GetQuat())
        for r in sample
    ]
    for sample in rotations
]
state = {"playing": False, "time": args.time, "mode": 2}


def display_mode(mode):
    state["mode"] = mode
    UsdGeom.Imageable(stage.GetPrimAtPath("/World/Model/Leaf")).GetVisibilityAttr().Set(
        "invisible" if mode == 1 else "inherited"
    )
    for j in range(7):
        UsdGeom.Imageable(
            stage.GetPrimAtPath(f"/World/Model/Link{j}")
        ).GetVisibilityAttr().Set("inherited" if mode == 1 else "invisible")
    UsdGeom.Imageable(
        stage.GetPrimAtPath("/World/JointMarkers")
    ).GetVisibilityAttr().Set("invisible" if mode == 0 else "inherited")


def jump(t):
    state.update(time=t, playing=False)


def select_joint(j):
    omni.usd.get_context().get_selection().set_selected_prim_paths(
        [f"/World/Model/Joint{j}"], True
    )


window = ui.Window("Leaf modelling | curved joint chain", width=430, height=330)
with window.frame:  # noqa: SIM117 - explicit UI nesting
    with ui.VStack(spacing=6):
        ui.Label("7 rigid strips + elastic bend/twist joints", height=22)
        ui.Label("ILLUSTRATIVE RIG - no live physics", height=22)
        ui.Label(
            "Pivots follow the curved surface. Recorded angles retargeted to this new rig.",
            word_wrap=True,
            height=35,
        )
        with ui.HStack(height=28):
            ui.Button("Surface", clicked_fn=lambda: display_mode(0))
            ui.Button("Segments", clicked_fn=lambda: display_mode(1))
            ui.Button("Surface + joints", clicked_fn=lambda: display_mode(2))
        with ui.HStack(height=28):
            ui.Button(
                "Play / Pause",
                clicked_fn=lambda: state.update(playing=not state["playing"]),
            )
            ui.Button("Rest shape", clicked_fn=lambda: jump(0))
            ui.Button("Peak contact", clicked_fn=lambda: jump(9.5))
            ui.Button("Recovered", clicked_fn=lambda: jump(20))
        time_label = ui.Label("", height=25)
        ui.Label(
            "Red markers: J0 at the base, J1-J6 towards the tip.",
            word_wrap=True,
            height=32,
        )
        ui.Label("Select a real USD joint to inspect its properties:", height=22)
        with ui.HStack(height=28):
            for j in range(7):
                ui.Button(f"J{j}", clicked_fn=lambda j=j: select_joint(j))
        ui.Label(
            "Spring returns the angle; damping reduces oscillation.\nGreen mesh blends neighbouring rigid-strip poses.",
            word_wrap=True,
            height=40,
        )


def update():
    i = int(np.argmin(abs(times - state["time"])))
    bp = [Gf.Vec3f(*centers[0])] + [Gf.Vec3f(*p) for p in positions[i]]
    anim.GetTranslationsAttr().Set(Vt.Vec3fArray(bp))
    anim.GetRotationsAttr().Set(Vt.QuatfArray([Gf.Quatf(1)] + quats[i]))
    for j, ops in enumerate(link_ops):
        ops[0].Set(Gf.Vec3d(*positions[i, j]))
        ops[1].Set(quats[i][j])
        pivot = pivots[j]
        marker = positions[i, j] + rotations[i, j] @ (pivot - centers[j + 1])
        markers[j].Set(Gf.Vec3d(*marker))
    time_label.text = f"Source motion time: {times[i]:.1f} s | samples: 10 Hz"


display_mode({"surface": 0, "segments": 1, "joints": 2}[args.mode])
set_camera_view(eye=np.array([0.125, -0.14, 0.135]), target=np.array([0.045, 0, 0.008]))
update()
stage.GetRootLayer().Export(
    str(ROOT / "artifacts/leaf61/notion-conclusions/leaf-model-view-curved.usda")
)
if args.capture:
    import asyncio

    from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport

    for _ in range(30):
        app.update()
    capture = capture_viewport_to_file(
        get_active_viewport(), str(args.capture.resolve())
    )
    task = asyncio.ensure_future(capture.wait_for_result())
    for _ in range(300):
        app.update()
        if task.done():
            break
    task.result()
else:
    previous = time.perf_counter()
    while app.is_running():
        now = time.perf_counter()
        if state["playing"]:
            state["time"] += min(now - previous, 0.1)
            if state["time"] > 20:
                state["time"] = 8
        previous = now
        update()
        app.update()
        time.sleep(1 / 60)
app.close()
