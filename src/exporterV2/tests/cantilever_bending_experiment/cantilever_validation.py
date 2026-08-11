"""
Quantitative cantilever validation for exporterV2 plant physics.

This script deliberately separates two claims:
  1. pre/post behavior: legacy_current vs new_physics under identical inputs
  2. physical realism: measured deflection vs a declared beam-theory benchmark

Pure formula/audit checks can run with:
    uv run python src/exporterV2/tests/cantilever_bending_experiment/cantilever_validation.py formula-check

Generation and simulation must run with Isaac Sim:
    ~/isaacsim/python.sh src/exporterV2/tests/cantilever_bending_experiment/cantilever_validation.py all
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import re
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[3]
SRC_DIR = REPO_ROOT / "src"
DATA_DIR = REPO_ROOT / "data" / "usd_models" / "physics_tests"
RESULTS_DIR = SCRIPT_DIR / "results"

if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

GRAVITY = 9.81
DEG_GAIN = math.pi / 180.0
DEFAULT_N_LINKS = (3, 5, 10, 20)
DEFAULT_MODELS = ("legacy_current", "new_physics")


@dataclass(frozen=True)
class Benchmark:
    name: str
    world_length_m: float
    outer_radius_m: float
    inner_radius_m: float
    young_modulus_pa: float
    density_kg_m3: float
    reference: str

    @property
    def area_m2(self) -> float:
        return math.pi * (self.outer_radius_m**2 - self.inner_radius_m**2)

    @property
    def second_moment_m4(self) -> float:
        return math.pi * (self.outer_radius_m**4 - self.inner_radius_m**4) / 4.0

    @property
    def flexural_rigidity_nm2(self) -> float:
        return self.young_modulus_pa * self.second_moment_m4

    @property
    def distributed_weight_npm(self) -> float:
        return self.density_kg_m3 * self.area_m2 * GRAVITY

    @property
    def expected_self_weight_mm(self) -> float:
        return (
            self.distributed_weight_npm
            * self.world_length_m**4
            / (8.0 * self.flexural_rigidity_nm2)
            * 1000.0
        )

    def expected_tip_force_mm(self, force_n: float) -> float:
        return force_n * self.world_length_m**3 / (3.0 * self.flexural_rigidity_nm2) * 1000.0

    def expected_discrete_self_weight_mm(self, n_links: int, base_hinge: bool = False) -> float:
        """Small-angle rigid-link/torsional-spring reference for the generated topology."""
        link_length = self.world_length_m / n_links
        k_rad = self.flexural_rigidity_nm2 / link_length
        delta_m = 0.0
        hinge_indices = range(0, n_links) if base_hinge else range(1, n_links)
        for hinge_index in hinge_indices:
            hinge_x = hinge_index * link_length
            moment = 0.0
            for link_index in range(n_links):
                load_x = (link_index + 0.5) * link_length
                if load_x > hinge_x:
                    moment += self.distributed_weight_npm * link_length * (load_x - hinge_x)
            theta = moment / k_rad
            delta_m += theta * (self.world_length_m - hinge_x)
        return delta_m * 1000.0

    def expected_discrete_tip_force_mm(
        self,
        n_links: int,
        force_n: float,
        base_hinge: bool = False,
    ) -> float:
        """Small-angle rigid-link/torsional-spring tip-load reference."""
        link_length = self.world_length_m / n_links
        k_rad = self.flexural_rigidity_nm2 / link_length
        delta_m = 0.0
        hinge_indices = range(0, n_links) if base_hinge else range(1, n_links)
        for hinge_index in hinge_indices:
            hinge_x = hinge_index * link_length
            lever = self.world_length_m - hinge_x
            theta = abs(force_n) * lever / k_rad
            delta_m += theta * lever
        return delta_m * 1000.0

    def expected_internal_stiffness_usd(self, n_links: int) -> float:
        segment_length = self.world_length_m / n_links
        return self.flexural_rigidity_nm2 / segment_length * DEG_GAIN

    def expected_base_stiffness_usd(self, n_links: int) -> float:
        segment_length = self.world_length_m / n_links
        return 2.0 * self.flexural_rigidity_nm2 / segment_length * DEG_GAIN


BENCHMARKS: dict[str, Benchmark] = {
    "synthetic_solid_40cm": Benchmark(
        name="synthetic_solid_40cm",
        world_length_m=0.400,
        outer_radius_m=0.010,
        inner_radius_m=0.0,
        young_modulus_pa=100.0e6,
        density_kg_m3=1000.0,
        reference="continuity benchmark from the earlier solid-cylinder test",
    ),
    "tomato_gao_20cm": Benchmark(
        name="tomato_gao_20cm",
        world_length_m=0.200,
        outer_radius_m=0.0111 / 2.0,
        inner_radius_m=0.00382 / 2.0,
        young_modulus_pa=50.64e6,
        density_kg_m3=769.96,
        reference="Gao et al. 2024 harvested tomato stalk, hollow circular section",
    ),
}


@dataclass(frozen=True)
class Scenario:
    name: str
    gravity_mps2: float
    tip_force_n: float

    def expected_mm(self, benchmark: Benchmark) -> float:
        if self.name == "self_weight":
            return benchmark.expected_self_weight_mm
        return benchmark.expected_tip_force_mm(abs(self.tip_force_n))

    def expected_discrete_mm(self, benchmark: Benchmark, n_links: int, base_hinge: bool = False) -> float:
        if self.name == "self_weight":
            return benchmark.expected_discrete_self_weight_mm(n_links, base_hinge=base_hinge)
        return benchmark.expected_discrete_tip_force_mm(n_links, abs(self.tip_force_n), base_hinge=base_hinge)


SCENARIOS: dict[str, Scenario] = {
    "self_weight": Scenario(name="self_weight", gravity_mps2=GRAVITY, tip_force_n=0.0),
    "tip_force_0p05N": Scenario(name="tip_force_0p05N", gravity_mps2=0.0, tip_force_n=-0.05),
}


def _split_csv(value: str | None, default: tuple[str, ...]) -> tuple[str, ...]:
    if not value:
        return default
    return tuple(part.strip() for part in value.split(",") if part.strip())


def _split_int_csv(value: str | None, default: tuple[int, ...]) -> tuple[int, ...]:
    if not value:
        return default
    return tuple(int(part.strip()) for part in value.split(",") if part.strip())


def model_uses_legacy(model: str) -> bool:
    if model == "legacy_current":
        return True
    if model == "new_physics":
        return False
    raise ValueError(f"Unknown model: {model}")


def usd_path(benchmark: str, model: str, n_links: int) -> Path:
    return DATA_DIR / f"cantilever_{benchmark}_{model}_N{n_links}.usda"


def result_json_path() -> Path:
    return RESULTS_DIR / "cantilever_validation_results.json"


def result_csv_path() -> Path:
    return RESULTS_DIR / "cantilever_validation_measurements.csv"


def report_path() -> Path:
    return RESULTS_DIR / "cantilever_validation_report.md"


def branch_defs_for_benchmark(benchmark: Benchmark, n_links: int) -> list[dict[str, Any]]:
    from exporterV2.core.tree_config import GLOBAL_SCALE

    return [
        {
            "id": "root_anchor",
            "parent": None,
            "attach_link": None,
            "n_links": 1,
            "radius": 0.02,
            "height": 0.05,
            "tilt": 0.0,
            "rot": 0.0,
            "joint_type": "fixed",
        },
        {
            "id": f"cantilever_{benchmark.name}_N{n_links}",
            "parent": "root_anchor",
            "attach_link": 1,
            "n_links": n_links,
            "radius": benchmark.outer_radius_m / GLOBAL_SCALE,
            "inner_radius": benchmark.inner_radius_m / GLOBAL_SCALE,
            "height": benchmark.world_length_m / GLOBAL_SCALE / n_links,
            "density": benchmark.density_kg_m3,
            "young_modulus": benchmark.young_modulus_pa,
            "tilt": 90.0,
            "rot": 0.0,
            "attachment_joint_type": "fixed",
        },
    ]


def generate_usd_files(benchmarks: tuple[str, ...], models: tuple[str, ...], n_links_values: tuple[int, ...]) -> list[dict[str, Any]]:
    from exporterV2.core.physics import apply_physx_articulation_settings, apply_physx_scene_settings
    from exporterV2.core.usd.stage import build_stage

    DATA_DIR.mkdir(parents=True, exist_ok=True)
    generated = []
    for benchmark_name in benchmarks:
        benchmark = BENCHMARKS[benchmark_name]
        for model in models:
            legacy = model_uses_legacy(model)
            for n_links in n_links_values:
                path = usd_path(benchmark_name, model, n_links)
                branches = branch_defs_for_benchmark(benchmark, n_links)
                stage, stem_path = build_stage(
                    output_path=str(path),
                    branches=branches,
                    skip_limit_check=True,
                    legacy_physics=legacy,
                )
                apply_physx_scene_settings(stage)
                apply_physx_articulation_settings(stage, stem_path)
                stage.GetRootLayer().Save()
                generated.append(
                    {
                        "benchmark": benchmark_name,
                        "model": model,
                        "n_links": n_links,
                        "usd_path": str(path),
                    }
                )
                print(f"[generate] {path}")
    return generated


def _read_float(pattern: str, text: str) -> float | None:
    match = re.search(pattern, text)
    return float(match.group(1)) if match else None


def _read_all_float(pattern: str, text: str) -> list[float]:
    return [float(value) for value in re.findall(pattern, text)]


def audit_usd_file(path: Path, benchmark: Benchmark, model: str, n_links: int) -> dict[str, Any]:
    text = path.read_text()
    prefix = f"cantilever_{benchmark.name}_N{n_links}_Link_"
    link_blocks = re.findall(
        rf'def Xform "({re.escape(prefix)}\d{{2}})".*?\n        \}}',
        text,
        flags=re.DOTALL,
    )

    masses = []
    heights = []
    radii = []
    link_block_texts = []
    for link_name in link_blocks:
        block_match = re.search(
            rf'def Xform "{re.escape(link_name)}".*?\n        \}}',
            text,
            flags=re.DOTALL,
        )
        if not block_match:
            continue
        block = block_match.group(0)
        link_block_texts.append(block)
        mass = _read_float(r"float physics:mass = ([0-9.eE+-]+)", block)
        height = _read_float(r"double height = ([0-9.eE+-]+)", block)
        radius = _read_float(r"double radius = ([0-9.eE+-]+)", block)
        if mass is not None:
            masses.append(mass)
        if height is not None:
            heights.append(height)
        if radius is not None:
            radii.append(radius)

    all_stiffness = _read_all_float(r"float drive:rotX:physics:stiffness = ([0-9.eE+-]+)", text)
    all_damping = _read_all_float(r"float drive:rotX:physics:damping = ([0-9.eE+-]+)", text)
    rot_low = _read_all_float(r"float limit:rotX:physics:low = ([0-9.eE+-]+)", text)
    rot_high = _read_all_float(r"float limit:rotX:physics:high = ([0-9.eE+-]+)", text)

    branch_mass = sum(masses)
    total_length = sum(heights)
    expected_mass = benchmark.density_kg_m3 * benchmark.area_m2 * benchmark.world_length_m
    expected_internal = benchmark.expected_internal_stiffness_usd(n_links)
    has_driven_attachment = len(all_stiffness) == n_links
    internal_stiffness = all_stiffness[1:] if has_driven_attachment else all_stiffness

    return {
        "usd_path": str(path),
        "exists": path.exists(),
        "benchmark": benchmark.name,
        "model": model,
        "n_links": n_links,
        "meters_per_unit": _read_float(r"metersPerUnit = ([0-9.eE+-]+)", text),
        "kilograms_per_unit": _read_float(r"kilogramsPerUnit = ([0-9.eE+-]+)", text),
        "tip_path": f"/World/Stem/{prefix}{n_links:02d}",
        "link_count": len(link_blocks),
        "total_length_m": total_length,
        "expected_length_m": benchmark.world_length_m,
        "length_error_m": total_length - benchmark.world_length_m,
        "total_branch_mass_kg": branch_mass,
        "expected_branch_mass_kg": expected_mass,
        "mass_error_kg": branch_mass - expected_mass,
        "link_height_m": heights[0] if heights else None,
        "outer_radius_m": radii[0] if radii else None,
        "inner_radius_m": benchmark.inner_radius_m,
        "support_type": "elastic_attachment" if has_driven_attachment else "fixed_attachment",
        "base_stiffness_usd": all_stiffness[0] if has_driven_attachment else None,
        "internal_stiffness_usd_mean": sum(internal_stiffness) / len(internal_stiffness) if internal_stiffness else None,
        "expected_internal_stiffness_usd": expected_internal,
        "internal_stiffness_rel_error": (
            (sum(internal_stiffness) / len(internal_stiffness) - expected_internal) / expected_internal
            if internal_stiffness and expected_internal
            else None
        ),
        "damping_usd_values": all_damping,
        "rot_x_limit_low_min": min(rot_low) if rot_low else None,
        "rot_x_limit_high_max": max(rot_high) if rot_high else None,
        "filtered_pair_count": text.count("physics:filteredPairs"),
        "legacy_note": (
            "legacy_current is generated through current code with legacy_physics=True; "
            "it is not a frozen historical implementation."
            if model == "legacy_current"
            else None
        ),
    }


def audit_usd_files(benchmarks: tuple[str, ...], models: tuple[str, ...], n_links_values: tuple[int, ...]) -> list[dict[str, Any]]:
    audits = []
    for benchmark_name in benchmarks:
        benchmark = BENCHMARKS[benchmark_name]
        for model in models:
            for n_links in n_links_values:
                path = usd_path(benchmark_name, model, n_links)
                if not path.exists():
                    audits.append(
                        {
                            "usd_path": str(path),
                            "exists": False,
                            "benchmark": benchmark_name,
                            "model": model,
                            "n_links": n_links,
                        }
                    )
                    continue
                audit = audit_usd_file(path, benchmark, model, n_links)
                audits.append(audit)
                print(
                    "[audit] "
                    f"{path.name}: L={audit['total_length_m']:.6f} m, "
                    f"m={audit['total_branch_mass_kg']:.8f} kg, "
                    f"Kint={audit['internal_stiffness_usd_mean']}"
                )
    return audits


def set_gravity(stage: Any, gravity_mps2: float) -> None:
    from pxr import Gf, UsdPhysics

    scene = UsdPhysics.Scene.Get(stage, "/World/PhysicsScene")
    if not scene:
        scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(float(gravity_mps2))


def quat_rotate_wxyz(quat: Any, vec: Any):
    import numpy as np

    q = np.asarray(quat, dtype=np.float64)
    v = np.asarray(vec, dtype=np.float64)
    w = q[0]
    qv = q[1:4]
    t = 2.0 * np.cross(qv, v)
    return v + w * t + np.cross(qv, t)


def squeeze_pose(value: Any):
    import numpy as np

    return np.squeeze(np.asarray(value, dtype=np.float64))


def tip_world_z(tip_prim: Any, link_height_m: float) -> float:
    pos, quat = tip_prim.get_world_poses()
    pos_v = squeeze_pose(pos)
    quat_v = squeeze_pose(quat)
    local_tip = [0.0, 0.0, link_height_m]
    tip_pos = pos_v + quat_rotate_wxyz(quat_v, local_tip)
    return float(tip_pos[2])


def simulate_one(
    simulation_app: Any,
    benchmark_name: str,
    model: str,
    n_links: int,
    scenario_name: str,
    max_seconds: float,
    physics_hz: float,
    substeps: int,
    settle_window_steps: int,
    settle_tolerance_m: float,
    render: bool,
) -> dict[str, Any]:
    print(
        "[simulate] starting "
        f"{benchmark_name}/{model}/N{n_links}/{scenario_name}",
        flush=True,
    )
    import numpy as np
    import omni
    from isaacsim.core.api import World
    from isaacsim.core.prims import Articulation, RigidPrim

    benchmark = BENCHMARKS[benchmark_name]
    scenario = SCENARIOS[scenario_name]
    path = usd_path(benchmark_name, model, n_links)
    audit = audit_usd_file(path, benchmark, model, n_links)
    tip_path = audit["tip_path"]
    link_height = audit["link_height_m"]

    if World.instance() is not None:
        World.instance().clear_instance()

    omni.usd.get_context().open_stage(str(path))
    simulation_app.update()
    stage = omni.usd.get_context().get_stage()
    set_gravity(stage, 0.0)

    physics_dt = 1.0 / physics_hz
    world = World(
        physics_dt=physics_dt,
        rendering_dt=1.0 / 60.0,
        stage_units_in_meters=1.0,
        physics_prim_path="/World/PhysicsScene",
        sim_params={
            "dt": physics_dt,
            "substeps": substeps,
            "gravity": [0.0, 0.0, 0.0],
            "solver_type": 1,
            "enable_solver_residuals": True,
        },
    )
    world.get_physics_context().set_gravity(0.0)
    articulation = Articulation("/World/Stem", name=f"stem_{benchmark_name}_{model}_{n_links}")
    world.scene.add(articulation)
    world.reset()
    articulation.initialize()

    tip = RigidPrim(tip_path)
    tip.initialize()
    z0 = tip_world_z(tip, link_height)
    set_gravity(stage, scenario.gravity_mps2)
    world.get_physics_context().set_gravity(-abs(scenario.gravity_mps2) if scenario.gravity_mps2 else 0.0)
    simulation_app.update()

    max_steps = int(max_seconds * physics_hz)
    min_steps = max(60, settle_window_steps)
    deflections = []
    samples = []
    settled = False
    nan_detected = False
    force_vec = np.array([[0.0, 0.0, scenario.tip_force_n]], dtype=np.float32)

    for step in range(max_steps):
        if scenario.tip_force_n != 0.0:
            tip.apply_forces(forces=force_vec, is_global=True)
        world.step(render=render)

        z = tip_world_z(tip, link_height)
        if math.isnan(z):
            nan_detected = True
            break
        deflection_m = z0 - z
        deflections.append(deflection_m)

        if step % int(physics_hz) == 0:
            samples.append({"step": step, "time_s": step / physics_hz, "deflection_mm": deflection_m * 1000.0})

        if step >= min_steps and len(deflections) >= settle_window_steps:
            recent = deflections[-settle_window_steps:]
            if max(recent) - min(recent) <= settle_tolerance_m:
                settled = True
                break

    final_deflection_m = deflections[-1] if deflections else float("nan")
    expected_mm = scenario.expected_mm(benchmark)
    expected_discrete_mm = scenario.expected_discrete_mm(benchmark, n_links, base_hinge=False)
    expected_discrete_base_hinge_mm = scenario.expected_discrete_mm(benchmark, n_links, base_hinge=True)
    measured_mm = final_deflection_m * 1000.0
    error_pct = abs(measured_mm - expected_mm) / expected_mm * 100.0 if expected_mm else float("nan")
    effective_ei_nm2 = (
        benchmark.flexural_rigidity_nm2 * expected_mm / measured_mm
        if measured_mm and not math.isnan(measured_mm)
        else float("nan")
    )
    discrete_error_pct = (
        abs(measured_mm - expected_discrete_mm) / expected_discrete_mm * 100.0
        if expected_discrete_mm
        else float("nan")
    )

    world.stop()
    world.clear_instance()
    omni.usd.get_context().close_stage()

    result = {
        "benchmark": benchmark_name,
        "model": model,
        "n_links": n_links,
        "scenario": scenario_name,
        "usd_path": str(path),
        "tip_path": tip_path,
        "initial_tip_z_m": z0,
        "final_deflection_m": final_deflection_m,
        "final_deflection_mm": measured_mm,
        "expected_deflection_mm": expected_mm,
        "expected_discrete_deflection_mm": expected_discrete_mm,
        "expected_discrete_base_hinge_deflection_mm": expected_discrete_base_hinge_mm,
        "error_pct": error_pct,
        "effective_ei_nm2": effective_ei_nm2,
        "target_ei_nm2": benchmark.flexural_rigidity_nm2,
        "discrete_error_pct": discrete_error_pct,
        "settled": settled,
        "not_settled": not settled,
        "nan_detected": nan_detected,
        "steps": len(deflections),
        "sim_time_s": len(deflections) / physics_hz,
        "physics_hz": physics_hz,
        "physics_dt": physics_dt,
        "substeps": substeps,
        "samples": samples,
    }
    print(
        "[simulate] "
        f"{benchmark_name}/{model}/N{n_links}/{scenario_name}: "
        f"{measured_mm:.4f} mm vs {expected_mm:.4f} mm "
        f"err={error_pct:.1f}% settled={settled}"
    )
    return result


def simulate_all(
    simulation_app: Any,
    benchmarks: tuple[str, ...],
    models: tuple[str, ...],
    n_links_values: tuple[int, ...],
    scenarios: tuple[str, ...],
    max_seconds: float,
    physics_hz: float,
    substeps: int,
    settle_window_steps: int,
    settle_tolerance_m: float,
    render: bool,
) -> list[dict[str, Any]]:
    results = []
    for benchmark_name in benchmarks:
        for model in models:
            for n_links in n_links_values:
                for scenario_name in scenarios:
                    try:
                        results.append(
                            simulate_one(
                                simulation_app,
                                benchmark_name,
                                model,
                                n_links,
                                scenario_name,
                                max_seconds,
                                physics_hz,
                                substeps,
                                settle_window_steps,
                                settle_tolerance_m,
                                render,
                            )
                        )
                    except KeyboardInterrupt:
                        raise
                    except BaseException as exc:
                        print(
                            "[simulate] error "
                            f"{benchmark_name}/{model}/N{n_links}/{scenario_name}: "
                            f"{type(exc).__name__}: {exc}",
                            flush=True,
                        )
                        results.append(
                            {
                                "benchmark": benchmark_name,
                                "model": model,
                                "n_links": n_links,
                                "scenario": scenario_name,
                                "error": f"{type(exc).__name__}: {exc}",
                                "settled": False,
                                "not_settled": True,
                            }
                        )
    return results


def formula_check() -> dict[str, Any]:
    checks = {}
    for name, benchmark in BENCHMARKS.items():
        checks[name] = {
            "area_m2": benchmark.area_m2,
            "I_m4": benchmark.second_moment_m4,
            "EI_Nm2": benchmark.flexural_rigidity_nm2,
            "w_Npm": benchmark.distributed_weight_npm,
            "self_weight_mm": benchmark.expected_self_weight_mm,
            "tip_force_0p05N_mm": benchmark.expected_tip_force_mm(0.05),
            "internal_stiffness_usd_by_n": {
                str(n): benchmark.expected_internal_stiffness_usd(n) for n in DEFAULT_N_LINKS
            },
        }

    assert abs(checks["synthetic_solid_40cm"]["self_weight_mm"] - 12.5568) < 0.01
    assert abs(checks["tomato_gao_20cm"]["self_weight_mm"] - 3.4637) < 0.01
    assert abs(checks["tomato_gao_20cm"]["tip_force_0p05N_mm"] - 3.5836) < 0.01
    print(json.dumps(checks, indent=2))
    return checks


def evaluate_acceptance(
    audits: list[dict[str, Any]],
    measurements: list[dict[str, Any]],
    n_links_values: tuple[int, ...],
) -> dict[str, Any]:
    acceptance: dict[str, Any] = {
        "length_mass_invariant": {},
        "n10_n20_new_within_15pct": {},
        "synthetic_n20_new_within_25pct": {},
        "tomato_realism_claim_allowed": BENCHMARKS["tomato_gao_20cm"].inner_radius_m > 0.0,
    }

    for benchmark_name in BENCHMARKS:
        for model in DEFAULT_MODELS:
            subset = [
                row
                for row in audits
                if row.get("exists") and row["benchmark"] == benchmark_name and row["model"] == model
            ]
            if not subset:
                continue
            lengths = [row["total_length_m"] for row in subset]
            masses = [row["total_branch_mass_kg"] for row in subset]
            length_ok = max(lengths) - min(lengths) <= 1e-9
            mass_span = max(masses) - min(masses)
            mass_tol = max(1e-8, 1e-6 * max(abs(mass) for mass in masses))
            mass_ok = mass_span <= mass_tol
            acceptance["length_mass_invariant"][f"{benchmark_name}/{model}"] = {
                "ok": length_ok and mass_ok,
                "length_span_m": max(lengths) - min(lengths),
                "mass_span_kg": mass_span,
                "mass_tolerance_kg": mass_tol,
            }

    for benchmark_name in BENCHMARKS:
        for scenario_name in SCENARIOS:
            n10 = next(
                (
                    row
                    for row in measurements
                    if row["benchmark"] == benchmark_name
                    and row["model"] == "new_physics"
                    and row["scenario"] == scenario_name
                    and row["n_links"] == 10
                    and "final_deflection_mm" in row
                ),
                None,
            )
            n20 = next(
                (
                    row
                    for row in measurements
                    if row["benchmark"] == benchmark_name
                    and row["model"] == "new_physics"
                    and row["scenario"] == scenario_name
                    and row["n_links"] == 20
                    and "final_deflection_mm" in row
                ),
                None,
            )
            if not n10 or not n20:
                continue
            denom = max(abs(n20["final_deflection_mm"]), 1e-12)
            rel = abs(n10["final_deflection_mm"] - n20["final_deflection_mm"]) / denom
            acceptance["n10_n20_new_within_15pct"][f"{benchmark_name}/{scenario_name}"] = {
                "ok": rel <= 0.15,
                "relative_difference": rel,
            }

    for scenario_name in SCENARIOS:
        n20 = next(
            (
                row
                for row in measurements
                if row["benchmark"] == "synthetic_solid_40cm"
                and row["model"] == "new_physics"
                and row["scenario"] == scenario_name
                and row["n_links"] == 20
                and "error_pct" in row
            ),
            None,
        )
        if not n20:
            continue
        acceptance["synthetic_n20_new_within_25pct"][scenario_name] = {
            "ok": n20["error_pct"] <= 25.0 and n20["settled"],
            "error_pct": n20["error_pct"],
            "settled": n20["settled"],
        }

    return acceptance


def write_json_results(payload: dict[str, Any]) -> None:
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    with result_json_path().open("w") as handle:
        json.dump(payload, handle, indent=2)
    print(f"[results] {result_json_path()}")


def write_measurement_csv(measurements: list[dict[str, Any]]) -> None:
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    fields = [
        "benchmark",
        "model",
        "n_links",
        "scenario",
        "final_deflection_mm",
        "expected_deflection_mm",
        "error_pct",
        "expected_discrete_deflection_mm",
        "discrete_error_pct",
        "effective_ei_nm2",
        "target_ei_nm2",
        "settled",
        "steps",
        "sim_time_s",
        "usd_path",
    ]
    with result_csv_path().open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in measurements:
            writer.writerow({field: row.get(field) for field in fields})
    print(f"[results] {result_csv_path()}")


def write_report(payload: dict[str, Any]) -> None:
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    lines = [
        "# Cantilever Validation Report",
        "",
        "This report separates pre/post simulation behavior from physical realism claims.",
        "",
        "## Benchmarks",
        "",
    ]
    for benchmark_name, benchmark_data in payload["benchmarks"].items():
        lines.extend(
            [
                f"### {benchmark_name}",
                "",
                f"- Reference: {benchmark_data['reference']}",
                f"- Length: {benchmark_data['world_length_m']:.3f} m",
                f"- Outer radius: {benchmark_data['outer_radius_m']:.6f} m",
                f"- Inner radius: {benchmark_data['inner_radius_m']:.6f} m",
                f"- EI: {benchmark_data['flexural_rigidity_nm2']:.8f} N*m^2",
                f"- Expected self-weight deflection: {benchmark_data['expected_self_weight_mm']:.4f} mm",
                f"- Expected 0.05 N tip-force deflection: {benchmark_data['expected_tip_force_0p05N_mm']:.4f} mm",
                "",
            ]
        )

    lines.extend(["## Measurements", ""])
    lines.append("| Benchmark | Model | N | Scenario | Measured mm | Expected mm | Error % | Settled |")
    lines.append("| --- | --- | ---: | --- | ---: | ---: | ---: | --- |")
    for row in payload.get("measurements", []):
        if row.get("error"):
            lines.append(
                "| "
                f"{row['benchmark']} | {row['model']} | {row['n_links']} | {row['scenario']} | "
                f"error | error | error | {row.get('settled', False)} |"
            )
            continue
        lines.append(
            "| "
            f"{row['benchmark']} | {row['model']} | {row['n_links']} | {row['scenario']} | "
            f"{row['final_deflection_mm']:.4f} | {row['expected_deflection_mm']:.4f} | "
            f"{row['error_pct']:.1f} | {row['settled']} |"
        )

    lines.extend(["", "## Discrete Reference", ""])
    lines.append("| Benchmark | Model | N | Scenario | Measured mm | Discrete mm | Discrete Error % |")
    lines.append("| --- | --- | ---: | --- | ---: | ---: | ---: |")
    for row in payload.get("measurements", []):
        if row.get("error") or "expected_discrete_deflection_mm" not in row:
            continue
        lines.append(
            "| "
            f"{row['benchmark']} | {row['model']} | {row['n_links']} | {row['scenario']} | "
            f"{row['final_deflection_mm']:.4f} | {row['expected_discrete_deflection_mm']:.4f} | "
            f"{row['discrete_error_pct']:.1f} |"
        )

    lines.extend(["", "## Effective EI", ""])
    lines.append("| Benchmark | Model | N | Scenario | Target EI | Effective EI |")
    lines.append("| --- | --- | ---: | --- | ---: | ---: |")
    for row in payload.get("measurements", []):
        if row.get("error") or "effective_ei_nm2" not in row:
            continue
        lines.append(
            "| "
            f"{row['benchmark']} | {row['model']} | {row['n_links']} | {row['scenario']} | "
            f"{row['target_ei_nm2']:.6f} | {row['effective_ei_nm2']:.6f} |"
        )

    lines.extend(["", "## Acceptance", ""])
    lines.append("```json")
    lines.append(json.dumps(payload.get("acceptance", {}), indent=2))
    lines.append("```")
    lines.extend(
        [
            "",
            "## Notes",
            "",
            "- `legacy_current` is generated by the current code path with `legacy_physics=True`; it is not a frozen historical implementation.",
            "- Tomato/Gao is an initial harvested-stalk reference, not a universal ground truth for a living greenhouse plant.",
            "- Failed or non-settled cases should be reported as failures, not reinterpreted as validation.",
        ]
    )

    report_path().write_text("\n".join(lines) + "\n")
    print(f"[results] {report_path()}")


def benchmark_payload() -> dict[str, Any]:
    payload = {}
    for name, benchmark in BENCHMARKS.items():
        data = asdict(benchmark)
        data.update(
            {
                "area_m2": benchmark.area_m2,
                "second_moment_m4": benchmark.second_moment_m4,
                "flexural_rigidity_nm2": benchmark.flexural_rigidity_nm2,
                "distributed_weight_npm": benchmark.distributed_weight_npm,
                "expected_self_weight_mm": benchmark.expected_self_weight_mm,
                "expected_tip_force_0p05N_mm": benchmark.expected_tip_force_mm(0.05),
            }
        )
        payload[name] = data
    return payload


def build_payload(
    generated: list[dict[str, Any]],
    audits: list[dict[str, Any]],
    measurements: list[dict[str, Any]],
    n_links_values: tuple[int, ...],
) -> dict[str, Any]:
    return {
        "benchmarks": benchmark_payload(),
        "generated": generated,
        "audits": audits,
        "measurements": measurements,
        "acceptance": evaluate_acceptance(audits, measurements, n_links_values),
        "sources": {
            "gao_2024": "https://www.mdpi.com/2077-0472/14/4/531",
            "openusd_drive_api": "https://openusd.org/release/api/class_usd_physics_drive_a_p_i.html",
            "coutand_2000": "https://pubmed.ncbi.nlm.nih.gov/11113160/",
            "plant_methods_flexural_stiffness": "https://ouci.dntb.gov.ua/en/works/l1pMrXo4/",
        },
    }


def load_existing_payload() -> dict[str, Any]:
    with result_json_path().open() as handle:
        return json.load(handle)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Cantilever physics validation runner")
    parser.add_argument(
        "command",
        choices=("formula-check", "generate", "audit", "simulate", "report", "all"),
        help="Action to run. `all` generates, audits, simulates, and writes reports.",
    )
    parser.add_argument("--benchmarks", default=",".join(BENCHMARKS.keys()))
    parser.add_argument("--models", default=",".join(DEFAULT_MODELS))
    parser.add_argument("--n-links", default=",".join(str(n) for n in DEFAULT_N_LINKS))
    parser.add_argument("--scenarios", default=",".join(SCENARIOS.keys()))
    parser.add_argument("--max-seconds", type=float, default=20.0)
    parser.add_argument("--physics-hz", type=float, default=480.0)
    parser.add_argument("--substeps", type=int, default=1)
    parser.add_argument("--settle-window-steps", type=int, default=240)
    parser.add_argument("--settle-tolerance-m", type=float, default=2.0e-5)
    parser.add_argument("--gui", action="store_true", help="Render simulation frames.")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    print(f"[runner] command={args.command}", flush=True)
    benchmarks = _split_csv(args.benchmarks, tuple(BENCHMARKS.keys()))
    models = _split_csv(args.models, DEFAULT_MODELS)
    n_links_values = _split_int_csv(args.n_links, DEFAULT_N_LINKS)
    scenarios = _split_csv(args.scenarios, tuple(SCENARIOS.keys()))

    unknown_benchmarks = sorted(set(benchmarks) - set(BENCHMARKS))
    unknown_models = sorted(set(models) - set(DEFAULT_MODELS))
    unknown_scenarios = sorted(set(scenarios) - set(SCENARIOS))
    if unknown_benchmarks or unknown_models or unknown_scenarios:
        raise ValueError(
            f"Unknown choices: benchmarks={unknown_benchmarks}, "
            f"models={unknown_models}, scenarios={unknown_scenarios}"
        )

    if args.command == "formula-check":
        formula_check()
        return 0

    # SimulationApp forwards remaining argv to Kit. Some command names, notably
    # "simulate", can be interpreted by Kit before our script continues.
    sys.argv = [sys.argv[0]]

    generated: list[dict[str, Any]] = []
    measurements: list[dict[str, Any]] = []
    simulation_app = None

    needs_isaac = args.command in {"generate", "simulate", "all"}
    if needs_isaac:
        from isaacsim import SimulationApp

        simulation_app = SimulationApp({"headless": not args.gui})
        print("[runner] SimulationApp ready", flush=True)

    try:
        if args.command in {"generate", "all"}:
            print("[runner] generating USD files", flush=True)
            generated = generate_usd_files(benchmarks, models, n_links_values)

        print("[runner] auditing USD files", flush=True)
        audits = audit_usd_files(benchmarks, models, n_links_values)

        if args.command in {"simulate", "all"}:
            print("[runner] running simulations", flush=True)
            measurements = simulate_all(
                simulation_app,
                benchmarks,
                models,
                n_links_values,
                scenarios,
                max_seconds=args.max_seconds,
                physics_hz=args.physics_hz,
                substeps=args.substeps,
                settle_window_steps=args.settle_window_steps,
                settle_tolerance_m=args.settle_tolerance_m,
                render=args.gui,
            )

        if args.command == "report":
            payload = load_existing_payload()
        else:
            payload = build_payload(generated, audits, measurements, n_links_values)
            write_json_results(payload)
            if measurements:
                write_measurement_csv(measurements)

        if args.command in {"report", "all", "simulate"}:
            write_report(payload)
    finally:
        if simulation_app is not None:
            simulation_app.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
