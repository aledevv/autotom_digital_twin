"""Generate isolated USD stages for evaluating truss optimization steps.

This module deliberately keeps every transformation local to the visual lab. It
uses the production truss/config builders as inputs, but does not alter their
behaviour or the official optimizer.

Run from the repository root with::

    UV_CACHE_DIR=/tmp/uv-cache uv run python \
        src/exporterV2/adapters/groimp_csv/tests/truss_visual_lab/generate_truss_visual_lab.py
"""

from __future__ import annotations

from copy import deepcopy
from pathlib import Path
import math
import sys
from typing import Iterable


sys.dont_write_bytecode = True
LAB_DIR = Path(__file__).resolve().parent
SRC_DIR = Path(__file__).resolve().parents[5]
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from pxr import Gf, Usd, UsdGeom, UsdPhysics

from exporterV2.adapters.groimp_csv.truss_builder import truss_to_complete_config
from exporterV2.core.usd.stage import build_stage


OUTPUT_DIR = LAB_DIR / "generated"
PEDICEL_BEND_LIMIT_DEG = 25.0
PEDICEL_DRIVE_STIFFNESS_SCALE = 0.30
STATIC_CURVE_SEGMENTS = 5
STATIC_BEND_PER_SEGMENT_DEG = 8.0
STATIC_PEDICEL_DROOP_DEG = 10.0
STATIC_ROOT_BEND_LIMIT_DEG = 18.0
STATIC_ROOT_DRIVE_STIFFNESS_SCALE = 0.40

STAGE_FILENAMES = (
    "00_current_simplified.usda",
    "01_dynamic_pedicels.usda",
    "02_opt_fixed_pedicels.usda",
    "03_opt_static_prebent_truss.usda",
)

STALE_STAGE_FILENAMES = (
    "03_opt_rachis_one_link.usda",
    "04_static_prebent_truss.usda",
)


def _is_rachis(branch: dict) -> bool:
    return "_rachis" in branch["id"] and "_pedicel_" not in branch["id"]


def _is_pedicel(branch: dict) -> bool:
    return "_pedicel_" in branch["id"]


def make_dynamic_pedicels(branches: list[dict]) -> list[dict]:
    """Return a copy where pedicel attachment joints are flexible D6 joints."""
    modified = deepcopy(branches)
    for branch in modified:
        if _is_pedicel(branch):
            branch["joint_type"] = "d6"
    return modified


def fix_pedicels(branches: list[dict]) -> list[dict]:
    """Return a copy where every pedicel is rigidly attached to the rachis."""
    modified = deepcopy(branches)
    for branch in modified:
        if _is_pedicel(branch):
            branch["joint_type"] = "fixed"
    return modified


def reduce_rachis_to_one_link(branches: list[dict]) -> list[dict]:
    """Collapse the rachis to one link and retain pedicel axial positions."""
    modified = deepcopy(branches)
    rachides = [branch for branch in modified if _is_rachis(branch)]
    if len(rachides) != 1:
        raise ValueError(f"Expected one rachis branch, found {len(rachides)}")

    rachis = rachides[0]
    old_link_count = int(rachis["n_links"])
    if old_link_count < 1:
        raise ValueError("Rachis must contain at least one link")

    total_length = float(rachis["height"]) * old_link_count
    rachis["n_links"] = 1
    rachis["height"] = total_length

    for branch in modified:
        if branch.get("parent") != rachis["id"]:
            continue
        old_attach_link = int(branch["attach_link"])
        old_attach_frac = float(branch.get("attach_frac", 1.0))
        axial_fraction = (old_attach_link - 1 + old_attach_frac) / old_link_count
        branch["attach_link"] = 1
        branch["attach_frac"] = min(max(axial_fraction, 0.0), 1.0)

    return modified


def make_static_prebent_truss(
    branches: list[dict],
    *,
    curve_segments: int = STATIC_CURVE_SEGMENTS,
    bend_per_segment_deg: float = STATIC_BEND_PER_SEGMENT_DEG,
    pedicel_droop_deg: float = STATIC_PEDICEL_DROOP_DEG,
) -> list[dict]:
    """Replace a one-link rachis with a fixed, piecewise-curved approximation."""
    if curve_segments < 2:
        raise ValueError("Static curvature requires at least two segments")

    source = deepcopy(branches)
    rachides = [branch for branch in source if _is_rachis(branch)]
    if len(rachides) != 1:
        raise ValueError(f"Expected one rachis branch, found {len(rachides)}")

    rachis = rachides[0]
    total_length = float(rachis["height"]) * int(rachis["n_links"])
    segment_length = total_length / curve_segments
    curved_ids = [f"{rachis['id']}_curve_{index + 1:02d}" for index in range(curve_segments)]

    curved_rachis = []
    for index, curved_id in enumerate(curved_ids):
        segment = deepcopy(rachis)
        segment.update(
            {
                "id": curved_id,
                "parent": rachis["parent"] if index == 0 else curved_ids[index - 1],
                "attach_link": rachis["attach_link"] if index == 0 else 1,
                "n_links": 1,
                "height": segment_length,
                "tilt": rachis["tilt"] if index == 0 else bend_per_segment_deg,
                "rot": rachis["rot"] if index == 0 else 0.0,
                # The first segment keeps one D6 attachment so the otherwise
                # rigid truss can move as a single block under fruit weight.
                "joint_type": "d6" if index == 0 else "fixed",
            }
        )
        segment.pop("attach_frac", None)
        curved_rachis.append(segment)

    remapped_pedicels = []
    for branch in source:
        if branch.get("parent") != rachis["id"]:
            continue

        pedicel = deepcopy(branch)
        axial_fraction = min(max(float(pedicel.get("attach_frac", 1.0)), 0.0), 1.0)
        scaled_position = axial_fraction * curve_segments
        segment_index = min(max(math.ceil(scaled_position) - 1, 0), curve_segments - 1)
        local_fraction = scaled_position - segment_index

        pedicel["parent"] = curved_ids[segment_index]
        pedicel["attach_link"] = 1
        pedicel["attach_frac"] = min(max(local_fraction, 0.0), 1.0)
        pedicel["joint_type"] = "fixed"
        pedicel["tilt"] = float(pedicel["tilt"]) + pedicel_droop_deg
        remapped_pedicels.append(pedicel)

    untouched = [
        deepcopy(branch)
        for branch in source
        if branch is not rachis and branch.get("parent") != rachis["id"]
    ]
    root_branches = [branch for branch in untouched if branch.get("parent") is None]
    other_branches = [branch for branch in untouched if branch.get("parent") is not None]
    return root_branches + curved_rachis + remapped_pedicels + other_branches


def create_synthetic_truss() -> tuple[list[dict], list[dict]]:
    """Create the common trunk, truss, pedicels and tomato definitions."""
    trunk = {
        "id": "trunk",
        "parent": None,
        "attach_link": None,
        "n_links": 5,
        "radius": 0.009,
        "height": 0.16,
        "tilt": 0.0,
        "rot": 0.0,
        "joint_type": "fixed",
    }
    truss = {
        "rachis_length": 0.24,
        "rachis_radius": 0.0025,
        "n_fruits": 7,
        "pedicel_length": 0.035,
        "pedicel_radius": 0.0015,
        "pedicel_angle": 90.0,
        "parent_rank": 2,
        "tilt_deg": 58.0,
        "azimuth_deg": 90.0,
        "tomato_radii": [0.018, 0.020, 0.017, 0.021, 0.019, 0.018, 0.020],
        "maturation": [0.0, 0.2, 0.5, 0.8, 1.0, 0.35, 0.9],
    }

    truss_branches, tomatoes = truss_to_complete_config(
        truss,
        parent_trunk_id=trunk["id"],
        rank=2,
    )
    terminal_bodies = [
        {
            "id": tomato["id"],
            "parent_branch_id": tomato["pedicel_id"],
            "shape": "sphere",
            "radius": tomato["radius"],
            "mass": tomato["mass"],
            "maturation": tomato["maturation"],
        }
        for tomato in tomatoes
    ]
    return [trunk, *truss_branches], terminal_bodies


def _patch_d6_joint(
    prim: Usd.Prim,
    *,
    bend_limit_deg: float,
    stiffness_scale: float,
    scale_existing_drive: bool = True,
) -> None:
    """Set lab-only limits and soften an existing D6 joint drive."""
    prim.SetCustomDataByKey("trussLab:bendLimitDeg", bend_limit_deg)
    prim.SetCustomDataByKey("trussLab:driveStiffnessScale", stiffness_scale)
    damping_scale = math.sqrt(stiffness_scale)
    for axis in ("rotX", "rotY"):
        limit = UsdPhysics.LimitAPI.Apply(prim, axis)
        limit.CreateLowAttr().Set(-bend_limit_deg)
        limit.CreateHighAttr().Set(bend_limit_deg)

        drive = UsdPhysics.DriveAPI.Get(prim, axis)
        stiffness = drive.GetStiffnessAttr().Get()
        damping = drive.GetDampingAttr().Get()
        if stiffness is not None and scale_existing_drive:
            drive.CreateStiffnessAttr().Set(stiffness * stiffness_scale)
        if damping is not None and scale_existing_drive:
            drive.CreateDampingAttr().Set(damping * damping_scale)


def patch_pedicel_d6_physics(
    stage: Usd.Stage,
    bend_limit_deg: float = PEDICEL_BEND_LIMIT_DEG,
    stiffness_scale: float = PEDICEL_DRIVE_STIFFNESS_SCALE,
) -> int:
    """Make generated pedicel D6 joints visibly compliant under gravity."""
    patched = 0
    for prim in stage.Traverse():
        if prim.GetTypeName() != "PhysicsJoint" or "_pedicel_" not in str(prim.GetPath()):
            continue
        _patch_d6_joint(
            prim,
            bend_limit_deg=bend_limit_deg,
            stiffness_scale=stiffness_scale,
            # Production pedicel branches already carry this scale now.
            scale_existing_drive=False,
        )
        patched += 1
    return patched


def patch_static_truss_root_physics(stage: Usd.Stage) -> int:
    """Tune the sole D6 at the root of the internally fixed curved truss."""
    patched = 0
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if (
            prim.GetTypeName() == "PhysicsJoint"
            and "_rachis_curve_01_Link_01/AttachJoint" in path
        ):
            _patch_d6_joint(
                prim,
                bend_limit_deg=STATIC_ROOT_BEND_LIMIT_DEG,
                stiffness_scale=STATIC_ROOT_DRIVE_STIFFNESS_SCALE,
            )
            patched += 1
    return patched


def count_stage_d6_joints(stage: Usd.Stage) -> int:
    """Count generic PhysicsJoint prims, excluding PhysicsFixedJoint prims."""
    return sum(1 for prim in stage.Traverse() if prim.GetTypeName() == "PhysicsJoint")


def rachis_total_length(branches: Iterable[dict]) -> float:
    """Return the summed unscaled length of all rachis geometry pieces."""
    return sum(
        float(branch["height"]) * int(branch["n_links"])
        for branch in branches
        if _is_rachis(branch)
    )


def _apply_lab_colors(stage: Usd.Stage, terminal_bodies: list[dict]) -> None:
    maturation_by_id = {body["id"]: float(body.get("maturation", 0.0)) for body in terminal_bodies}
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Gprim):
            continue

        path = str(prim.GetPath())
        if prim.GetTypeName() == "Sphere":
            tomato_id = prim.GetPath().GetParentPath().name
            maturation = maturation_by_id.get(tomato_id, 0.0)
            color = Gf.Vec3f(0.25 + 0.65 * maturation, 0.65 - 0.48 * maturation, 0.08)
        elif "_pedicel_" in path:
            color = Gf.Vec3f(0.52, 0.72, 0.22)
        elif "_rachis" in path:
            color = Gf.Vec3f(0.20, 0.55, 0.16)
        elif "trunk" in path:
            color = Gf.Vec3f(0.30, 0.42, 0.16)
        else:
            continue
        UsdGeom.Gprim(prim).CreateDisplayColorAttr().Set([color])


def _set_lab_metadata(stage: Usd.Stage, stage_name: str) -> None:
    world = stage.GetPrimAtPath("/World")
    world.SetCustomDataByKey("trussLab:stage", stage_name)
    world.SetCustomDataByKey("trussLab:d6JointCount", count_stage_d6_joints(stage))


def build_stage_configurations() -> dict[str, list[dict]]:
    """Build all five branch configurations without creating files."""
    base, _ = create_synthetic_truss()
    current = fix_pedicels(base)
    dynamic = make_dynamic_pedicels(current)
    fixed = fix_pedicels(dynamic)
    one_link = reduce_rachis_to_one_link(fixed)
    static_prebent = make_static_prebent_truss(one_link)
    return dict(zip(STAGE_FILENAMES, (current, dynamic, fixed, static_prebent)))


def generate_visual_stages(output_dir: Path | str = OUTPUT_DIR) -> dict[str, Path]:
    """Generate and save all visual-lab USD files."""
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    for stale_filename in STALE_STAGE_FILENAMES:
        stale_path = output_dir / stale_filename
        if stale_path.exists():
            stale_path.unlink()

    _, terminal_bodies = create_synthetic_truss()
    configurations = build_stage_configurations()
    generated = {}

    for filename, branches in configurations.items():
        output_path = output_dir / filename
        stage, _ = build_stage(
            str(output_path),
            branches=branches,
            locked_joints=False,
            skip_limit_check=True,
            terminal_bodies=terminal_bodies,
        )
        if filename == "01_dynamic_pedicels.usda":
            patched = patch_pedicel_d6_physics(stage)
            if patched != len(terminal_bodies):
                raise RuntimeError(
                    f"Expected to patch {len(terminal_bodies)} pedicel joints, patched {patched}"
                )
        elif filename == "03_opt_static_prebent_truss.usda":
            patched = patch_static_truss_root_physics(stage)
            if patched != 1:
                raise RuntimeError(f"Expected to patch one static truss root joint, patched {patched}")

        _apply_lab_colors(stage, terminal_bodies)
        _set_lab_metadata(stage, filename)
        stage.GetRootLayer().Save()
        generated[filename] = output_path

    return generated


def main() -> int:
    generated = generate_visual_stages()
    print("\nVisual Truss Lab generated:")
    for filename, path in generated.items():
        stage = Usd.Stage.Open(str(path))
        print(f"  {filename}: {count_stage_d6_joints(stage)} D6 joints")
    print(f"\nOutput: {OUTPUT_DIR}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
