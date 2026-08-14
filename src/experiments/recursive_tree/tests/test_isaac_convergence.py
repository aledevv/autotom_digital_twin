#!/usr/bin/env python3
"""
Task 3: Isaac Sim Convergence Tests

Test which scalability configs are stable in Isaac Sim physics simulation.

Metrics:
- Position drift: how much does the plant move over time?
- Velocity oscillations: continuous jittering/vibration?
- Joint health: any NaN, inf, or extreme values?

Classification:
- STABLE: minimal drift, low velocity, converges quickly
- MARGINAL: some drift/oscillations but doesn't diverge
- UNSTABLE: continuous oscillations, drift, or crashes
"""
import os
import sys
import json
import numpy as np
from pathlib import Path
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass, asdict

# Add parent to path for imports
SCRIPT_DIR = Path(__file__).parent.resolve()
RECURSIVE_TREE_DIR = SCRIPT_DIR.parent
sys.path.insert(0, str(RECURSIVE_TREE_DIR))

# Isaac Sim imports (only after adding to path)
from omni.isaac.kit import SimulationApp

# Config for headless mode
config = {
    "headless": True,
    "width": 640,
    "height": 480,
}
simulation_app = SimulationApp(config)

from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema
from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.stage import get_current_stage
import carb


@dataclass
class ConvergenceMetrics:
    """Metrics for stability assessment."""
    config_name: str
    duration_s: float
    
    # Position metrics
    position_drift_mm: float  # Total displacement of root
    max_link_displacement_mm: float  # Worst case link movement
    
    # Velocity metrics
    mean_velocity_magnitude: float  # Average |v| over time
    max_velocity_magnitude: float  # Peak |v|
    velocity_settling_time: float  # Time to reach low velocity
    
    # Oscillation detection
    has_persistent_oscillation: bool  # Still moving at end?
    oscillation_frequency_hz: Optional[float]  # If oscillating
    
    # Joint health
    has_nan_or_inf: bool  # Any numerical issues
    max_joint_velocity: float  # rad/s
    
    # Overall classification
    classification: str  # STABLE / MARGINAL / UNSTABLE / ERROR
    reason: str  # Why classified this way


class ConvergenceTest:
    """Run convergence test on a single config."""
    
    def __init__(self, config_name: str, usd_path: str, duration_s: float = 30.0):
        self.config_name = config_name
        self.usd_path = usd_path
        self.duration_s = duration_s
        
        # Thresholds for classification
        self.STABLE_DRIFT_MM = 5.0  # < 5mm total drift
        self.STABLE_VELOCITY = 0.01  # < 1cm/s average velocity
        self.MARGINAL_DRIFT_MM = 20.0  # < 20mm drift
        self.MARGINAL_VELOCITY = 0.05  # < 5cm/s average velocity
        
        # Data collection
        self.timestamps = []
        self.root_positions = []
        self.root_velocities = []
        self.joint_positions = []
        self.joint_velocities = []
        
    def setup_world(self):
        """Create Isaac Sim world and load USD."""
        self.world = World(stage_units_in_meters=1.0, physics_dt=1/480.0, rendering_dt=1/60.0)
        self.world.scene.add_default_ground_plane()
        
        # Load USD
        stage = get_current_stage()
        tree_prim_path = f"/World/tomato_{self.config_name}"
        tree_prim = stage.DefinePrim(tree_prim_path, "Xform")
        tree_prim.GetReferences().AddReference(self.usd_path)
        
        # Wait for stage to update
        self.world.reset()
        
        # Get articulation
        articulation_path = f"{tree_prim_path}/stem"
        self.articulation = Articulation(articulation_path)
        self.articulation.initialize()
        
        print(f"  Loaded: {self.config_name}")
        print(f"  Articulation: {self.articulation.num_dof} DOFs")
        
    def run_simulation(self):
        """Run physics simulation and collect metrics."""
        print(f"  Running {self.duration_s}s simulation...")
        
        # Reset to initial state
        self.world.reset()
        
        # Get timestep
        dt = self.world.get_physics_dt()
        num_steps = int(self.duration_s / dt)
        
        # Sample rate (collect data every 0.1s)
        sample_interval = max(1, int(0.1 / dt))
        
        for step in range(num_steps):
            # Step physics
            self.world.step(render=False)
            
            # Collect data periodically
            if step % sample_interval == 0:
                self._collect_data(step * dt)
            
            # Check for catastrophic failure
            if self._check_failure():
                print(f"  ⚠️  Simulation diverged at t={step*dt:.2f}s")
                break
        
        print(f"  Simulation complete ({len(self.timestamps)} samples)")
        
    def _collect_data(self, t: float):
        """Collect articulation state at current timestep."""
        try:
            # Root position (stem base)
            root_pos, root_rot = self.articulation.get_world_pose()
            
            # Root velocity
            root_vel = self.articulation.get_linear_velocity()
            
            # Joint states
            joint_pos = self.articulation.get_joint_positions()
            joint_vel = self.articulation.get_joint_velocities()
            
            # Store
            self.timestamps.append(t)
            self.root_positions.append(root_pos)
            self.root_velocities.append(root_vel)
            self.joint_positions.append(joint_pos)
            self.joint_velocities.append(joint_vel)
            
        except Exception as e:
            print(f"  ⚠️  Data collection error at t={t:.2f}s: {e}")
            
    def _check_failure(self) -> bool:
        """Check for catastrophic simulation failure."""
        try:
            root_pos, _ = self.articulation.get_world_pose()
            
            # Check for NaN/inf
            if np.any(np.isnan(root_pos)) or np.any(np.isinf(root_pos)):
                return True
            
            # Check for explosion (>10m displacement)
            if np.linalg.norm(root_pos) > 10.0:
                return True
            
            return False
            
        except:
            return True
    
    def analyze_metrics(self) -> ConvergenceMetrics:
        """Analyze collected data and classify stability."""
        if len(self.timestamps) < 10:
            return ConvergenceMetrics(
                config_name=self.config_name,
                duration_s=self.duration_s,
                position_drift_mm=0,
                max_link_displacement_mm=0,
                mean_velocity_magnitude=0,
                max_velocity_magnitude=0,
                velocity_settling_time=0,
                has_persistent_oscillation=False,
                oscillation_frequency_hz=None,
                has_nan_or_inf=True,
                max_joint_velocity=0,
                classification="ERROR",
                reason="Insufficient data (simulation crashed early)"
            )
        
        # Convert to numpy arrays
        positions = np.array(self.root_positions)  # (N, 3)
        velocities = np.array(self.root_velocities)  # (N, 3)
        joint_vels = np.array(self.joint_velocities)  # (N, DOF)
        
        # Position drift
        initial_pos = positions[0]
        final_pos = positions[-1]
        drift = np.linalg.norm(final_pos - initial_pos) * 1000  # mm
        
        # Max displacement from initial
        displacements = np.linalg.norm(positions - initial_pos, axis=1) * 1000  # mm
        max_displacement = np.max(displacements)
        
        # Velocity metrics
        velocity_magnitudes = np.linalg.norm(velocities, axis=1)
        mean_vel = np.mean(velocity_magnitudes)
        max_vel = np.max(velocity_magnitudes)
        
        # Settling time (time to reach low velocity)
        settling_threshold = 0.01  # 1cm/s
        settled_mask = velocity_magnitudes < settling_threshold
        if np.any(settled_mask):
            settling_idx = np.where(settled_mask)[0][0]
            settling_time = self.timestamps[settling_idx]
        else:
            settling_time = self.duration_s  # Never settled
        
        # Oscillation detection (last 5s still moving?)
        last_5s_mask = np.array(self.timestamps) > (self.duration_s - 5.0)
        if np.any(last_5s_mask):
            last_5s_vel = velocity_magnitudes[last_5s_mask]
            has_oscillation = np.mean(last_5s_vel) > settling_threshold
        else:
            has_oscillation = False
        
        # Joint health
        has_nan_inf = np.any(np.isnan(joint_vels)) or np.any(np.isinf(joint_vels))
        max_joint_vel = np.max(np.abs(joint_vels)) if not has_nan_inf else float('inf')
        
        # Classification
        classification, reason = self._classify(
            drift, mean_vel, has_oscillation, has_nan_inf, settling_time
        )
        
        return ConvergenceMetrics(
            config_name=self.config_name,
            duration_s=self.duration_s,
            position_drift_mm=float(drift),
            max_link_displacement_mm=float(max_displacement),
            mean_velocity_magnitude=float(mean_vel),
            max_velocity_magnitude=float(max_vel),
            velocity_settling_time=float(settling_time),
            has_persistent_oscillation=bool(has_oscillation),
            oscillation_frequency_hz=None,  # TODO: FFT analysis
            has_nan_or_inf=bool(has_nan_inf),
            max_joint_velocity=float(max_joint_vel),
            classification=classification,
            reason=reason
        )
    
    def _classify(
        self, drift: float, mean_vel: float, has_osc: bool, 
        has_nan: bool, settling_time: float
    ) -> Tuple[str, str]:
        """Classify stability based on metrics."""
        
        # ERROR cases
        if has_nan:
            return "ERROR", "NaN/inf detected in joint states"
        
        # UNSTABLE cases
        if drift > 100:  # >10cm drift
            return "UNSTABLE", f"Excessive drift: {drift:.1f}mm"
        
        if mean_vel > 0.1:  # >10cm/s average
            return "UNSTABLE", f"High average velocity: {mean_vel*1000:.1f}mm/s"
        
        if has_osc and settling_time > self.duration_s * 0.8:
            return "UNSTABLE", "Persistent oscillations (never settled)"
        
        # STABLE cases
        if drift < self.STABLE_DRIFT_MM and mean_vel < self.STABLE_VELOCITY:
            if settling_time < 5.0:
                return "STABLE", f"Quick convergence: settled in {settling_time:.1f}s"
            else:
                return "STABLE", f"Converged (drift={drift:.1f}mm, vel={mean_vel*1000:.1f}mm/s)"
        
        # MARGINAL cases
        if drift < self.MARGINAL_DRIFT_MM and mean_vel < self.MARGINAL_VELOCITY:
            if has_osc:
                return "MARGINAL", f"Some oscillations (drift={drift:.1f}mm)"
            else:
                return "MARGINAL", f"Slow settling (time={settling_time:.1f}s)"
        
        # Default MARGINAL
        return "MARGINAL", f"Borderline (drift={drift:.1f}mm, vel={mean_vel*1000:.1f}mm/s)"
    
    def run(self) -> ConvergenceMetrics:
        """Run full test: setup → simulate → analyze."""
        try:
            self.setup_world()
            self.run_simulation()
            metrics = self.analyze_metrics()
            return metrics
            
        except Exception as e:
            print(f"  ❌ Test failed: {e}")
            import traceback
            traceback.print_exc()
            
            return ConvergenceMetrics(
                config_name=self.config_name,
                duration_s=self.duration_s,
                position_drift_mm=0,
                max_link_displacement_mm=0,
                mean_velocity_magnitude=0,
                max_velocity_magnitude=0,
                velocity_settling_time=0,
                has_persistent_oscillation=False,
                oscillation_frequency_hz=None,
                has_nan_or_inf=True,
                max_joint_velocity=0,
                classification="ERROR",
                reason=f"Exception: {str(e)}"
            )


def get_test_configs() -> List[Tuple[str, str]]:
    """Get list of (config_name, usd_path) to test."""
    usd_dir = SCRIPT_DIR / "scalability_usds"
    
    configs = []
    for usd_file in sorted(usd_dir.glob("*.usda")):
        config_name = usd_file.stem
        configs.append((config_name, str(usd_file)))
    
    return configs


def main():
    """Run convergence tests on all scalability configs."""
    print("=" * 80)
    print(" " * 20 + "TASK 3: ISAAC SIM CONVERGENCE TESTS")
    print("=" * 80)
    print()
    print("Testing which scalability configs are stable in Isaac Sim.")
    print(f"Duration: 30s per config")
    print(f"Physics: 480Hz, position_iterations=64, velocity_iterations=8")
    print()
    
    # Get configs to test
    configs = get_test_configs()
    print(f"Found {len(configs)} configs to test")
    print()
    
    # Run tests
    results = []
    for i, (config_name, usd_path) in enumerate(configs, 1):
        print(f"[{i}/{len(configs)}] Testing: {config_name}")
        print("-" * 80)
        
        test = ConvergenceTest(config_name, usd_path, duration_s=30.0)
        metrics = test.run()
        results.append(metrics)
        
        # Print summary
        status_emoji = {
            "STABLE": "✅",
            "MARGINAL": "⚠️",
            "UNSTABLE": "❌",
            "ERROR": "💥"
        }
        emoji = status_emoji.get(metrics.classification, "❓")
        
        print(f"  {emoji} {metrics.classification}: {metrics.reason}")
        print(f"  Drift: {metrics.position_drift_mm:.1f}mm, "
              f"Vel: {metrics.mean_velocity_magnitude*1000:.1f}mm/s, "
              f"Settled: {metrics.velocity_settling_time:.1f}s")
        print()
    
    # Final report
    print("=" * 80)
    print(" " * 30 + "FINAL REPORT")
    print("=" * 80)
    print()
    
    # Count by classification
    stable = [r for r in results if r.classification == "STABLE"]
    marginal = [r for r in results if r.classification == "MARGINAL"]
    unstable = [r for r in results if r.classification == "UNSTABLE"]
    errors = [r for r in results if r.classification == "ERROR"]
    
    print(f"✅ STABLE:   {len(stable)}/{len(results)}")
    print(f"⚠️  MARGINAL: {len(marginal)}/{len(results)}")
    print(f"❌ UNSTABLE: {len(unstable)}/{len(results)}")
    print(f"💥 ERROR:    {len(errors)}/{len(results)}")
    print()
    
    # Detailed table
    print(f"{'Config':<30} {'Status':<10} {'Drift(mm)':<12} {'Vel(mm/s)':<12} {'Settled(s)':<10}")
    print("-" * 80)
    for r in results:
        emoji = status_emoji.get(r.classification, "❓")
        print(f"{r.config_name:<30} {emoji} {r.classification:<8} "
              f"{r.position_drift_mm:<11.1f} {r.mean_velocity_magnitude*1000:<11.1f} "
              f"{r.velocity_settling_time:<10.1f}")
    print()
    
    # Save results
    output_file = SCRIPT_DIR / "convergence_results.json"
    with open(output_file, 'w') as f:
        json.dump([asdict(r) for r in results], f, indent=2)
    print(f"Results saved: {output_file}")
    print()
    
    # Recommendations
    print("=" * 80)
    print("RECOMMENDATIONS FOR TASK 4 (Force Resistance Tests)")
    print("=" * 80)
    if stable:
        print("✅ Use these STABLE configs for robot interaction:")
        for r in stable:
            print(f"   - {r.config_name}")
    else:
        print("⚠️  No fully stable configs found!")
        if marginal:
            print("   Consider these MARGINAL configs (with caution):")
            for r in marginal[:3]:  # Top 3
                print(f"   - {r.config_name} (drift={r.position_drift_mm:.1f}mm)")
    
    print()
    simulation_app.close()


if __name__ == "__main__":
    main()
