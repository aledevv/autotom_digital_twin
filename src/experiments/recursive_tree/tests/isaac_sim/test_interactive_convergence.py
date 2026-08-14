#!/usr/bin/env python3
"""
Task 3: Interactive Isaac Sim Convergence Tests

Load each config in Isaac Sim with GUI and let user manually classify stability.

Usage:
    ./run_experiment.sh recursive_tree tests/test_interactive_convergence.py

Controls:
    Press PLAY to start simulation
    Observe for 10-30s
    Press ESC to stop and classify
    
Classification prompt:
    1 = STABLE (quick settle, no oscillations)
    2 = MARGINAL (some drift/oscillations but converges)
    3 = UNSTABLE (continuous oscillations, divergence)
    0 = SKIP (not tested yet)
"""
import os
import sys
import json
import time
from pathlib import Path
from dataclasses import dataclass, asdict
from typing import Dict, List, Optional

# Add parent to path for imports
SCRIPT_DIR = Path(__file__).parent.resolve()
RECURSIVE_TREE_DIR = SCRIPT_DIR.parent.parent
sys.path.insert(0, str(RECURSIVE_TREE_DIR))

# Isaac Sim imports
from omni.isaac.kit import SimulationApp

# Config for GUI mode (non-headless)
config = {
    "headless": False,
    "width": 1920,
    "height": 1080,
}
simulation_app = SimulationApp(config)

from pxr import Usd, UsdGeom
from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.stage import get_current_stage
import carb


@dataclass
class InteractiveResult:
    """Result from manual classification."""
    config_name: str
    classification: str  # STABLE / MARGINAL / UNSTABLE / SKIPPED
    notes: str  # User observations
    test_duration_s: float  # How long user observed
    timestamp: str  # When tested


class InteractiveTester:
    """Interactive convergence test runner."""
    
    def __init__(self):
        self.world = None
        self.articulation = None
        self.results_file = SCRIPT_DIR / "convergence_results_interactive.json"
        self.results = []
        
        # Load previous results if they exist
        self._load_previous_results()
    
    def _load_previous_results(self):
        """Load previously tested configs."""
        if self.results_file.exists():
            with open(self.results_file, 'r') as f:
                data = json.load(f)
                self.results = [InteractiveResult(**r) for r in data]
            print(f"Loaded {len(self.results)} previous results")
    
    def _save_results(self):
        """Save results to JSON after each test."""
        with open(self.results_file, 'w') as f:
            json.dump([asdict(r) for r in self.results], f, indent=2)
        print(f"✅ Results saved: {self.results_file}")
    
    def _get_tested_configs(self) -> set:
        """Get set of already tested config names."""
        return {r.config_name for r in self.results if r.classification != "SKIPPED"}
    
    def setup_world(self):
        """Create Isaac Sim world."""
        if self.world is not None:
            self.world.clear()
        
        self.world = World(stage_units_in_meters=1.0, physics_dt=1/480.0, rendering_dt=1/60.0)
        self.world.scene.add_default_ground_plane()
        print("✅ World initialized")
    
    def load_config(self, config_name: str, usd_path: str):
        """Load a specific USD config."""
        # Reset world
        if self.world is None:
            self.setup_world()
        else:
            # Clear previous articulation
            stage = get_current_stage()
            if stage:
                # Remove previous tomato if exists
                for prim_path in ["/World/tomato"]:
                    prim = stage.GetPrimAtPath(prim_path)
                    if prim.IsValid():
                        stage.RemovePrim(prim_path)
        
        # Load USD
        stage = get_current_stage()
        tree_prim_path = "/World/tomato"
        tree_prim = stage.DefinePrim(tree_prim_path, "Xform")
        tree_prim.GetReferences().AddReference(usd_path)
        
        # Reset to apply changes
        self.world.reset()
        
        # Get articulation
        articulation_path = f"{tree_prim_path}/stem"
        self.articulation = Articulation(articulation_path)
        self.articulation.initialize()
        
        print(f"\n{'='*80}")
        print(f"CONFIG: {config_name}")
        print(f"{'='*80}")
        print(f"USD: {Path(usd_path).name}")
        print(f"DOFs: {self.articulation.num_dof}")
        print(f"\n📌 INSTRUCTIONS:")
        print(f"   1. Press PLAY ▶️  to start simulation")
        print(f"   2. Observe for 10-30 seconds")
        print(f"   3. Check this terminal when ready to classify")
        print(f"{'='*80}\n")
    
    def get_user_classification(self, config_name: str, start_time: float) -> InteractiveResult:
        """Get classification from user via terminal input."""
        duration = time.time() - start_time
        
        print(f"\n{'='*80}")
        print(f"CLASSIFICATION - {config_name}")
        print(f"{'='*80}")
        print(f"You observed for {duration:.1f} seconds")
        print()
        print("How would you classify this configuration?")
        print()
        print("  1 = ✅ STABLE")
        print("      - Settles quickly (< 5s)")
        print("      - Minimal drift/oscillations")
        print("      - Safe for robot interaction")
        print()
        print("  2 = ⚠️  MARGINAL")
        print("      - Takes time to settle (5-20s)")
        print("      - Some oscillations but converges")
        print("      - Borderline usable")
        print()
        print("  3 = ❌ UNSTABLE")
        print("      - Continuous oscillations/jitter")
        print("      - Significant drift")
        print("      - Never settles or diverges")
        print()
        print("  0 = SKIP (test later)")
        print()
        
        while True:
            choice = input("Enter choice (0/1/2/3): ").strip()
            
            if choice == "1":
                classification = "STABLE"
                break
            elif choice == "2":
                classification = "MARGINAL"
                break
            elif choice == "3":
                classification = "UNSTABLE"
                break
            elif choice == "0":
                classification = "SKIPPED"
                break
            else:
                print("❌ Invalid choice. Please enter 0, 1, 2, or 3")
        
        # Get optional notes
        print()
        notes = input("Optional notes (press Enter to skip): ").strip()
        if not notes:
            notes = "No notes"
        
        from datetime import datetime
        timestamp = datetime.now().isoformat()
        
        return InteractiveResult(
            config_name=config_name,
            classification=classification,
            notes=notes,
            test_duration_s=duration,
            timestamp=timestamp
        )
    
    def run_test(self, config_name: str, usd_path: str):
        """Run interactive test for one config."""
        try:
            # Load config
            self.load_config(config_name, usd_path)
            
            # Record start time
            start_time = time.time()
            
            # Wait for user to observe
            print("Waiting for you to test... (terminal will prompt when ready)")
            print("(You can also press Ctrl+C to skip this config)")
            print()
            
            # Simple wait - user controls when to stop via terminal
            input("Press ENTER when ready to classify (or Ctrl+C to skip)...")
            
            # Get classification
            result = self.get_user_classification(config_name, start_time)
            
            # Save result
            self.results.append(result)
            self._save_results()
            
            emoji = {
                "STABLE": "✅",
                "MARGINAL": "⚠️",
                "UNSTABLE": "❌",
                "SKIPPED": "⏭️"
            }
            print(f"\n{emoji.get(result.classification, '❓')} Recorded: {result.classification}")
            if result.notes != "No notes":
                print(f"   Notes: {result.notes}")
            print()
            
        except KeyboardInterrupt:
            print("\n⏭️  Skipped by user (Ctrl+C)")
            result = InteractiveResult(
                config_name=config_name,
                classification="SKIPPED",
                notes="Skipped via Ctrl+C",
                test_duration_s=0,
                timestamp=""
            )
            self.results.append(result)
            self._save_results()
        except Exception as e:
            print(f"\n❌ Error testing {config_name}: {e}")
            import traceback
            traceback.print_exc()


def get_test_configs() -> List[tuple]:
    """Get list of (config_name, usd_path) to test."""
    usd_dir = SCRIPT_DIR / "scalability_usds"
    
    configs = []
    for usd_file in sorted(usd_dir.glob("*.usda")):
        config_name = usd_file.stem
        configs.append((config_name, str(usd_file)))
    
    return configs


def print_summary(tester: InteractiveTester):
    """Print summary of all results."""
    print("\n" + "="*80)
    print(" " * 25 + "FINAL SUMMARY")
    print("="*80)
    print()
    
    # Count by classification
    stable = [r for r in tester.results if r.classification == "STABLE"]
    marginal = [r for r in tester.results if r.classification == "MARGINAL"]
    unstable = [r for r in tester.results if r.classification == "UNSTABLE"]
    skipped = [r for r in tester.results if r.classification == "SKIPPED"]
    
    total_tested = len(stable) + len(marginal) + len(unstable)
    
    print(f"✅ STABLE:   {len(stable)}")
    print(f"⚠️  MARGINAL: {len(marginal)}")
    print(f"❌ UNSTABLE: {len(unstable)}")
    print(f"⏭️  SKIPPED:  {len(skipped)}")
    print(f"📊 TESTED:   {total_tested}/{len(tester.results)}")
    print()
    
    # Detailed table
    if tester.results:
        print(f"{'Config':<35} {'Status':<12} {'Duration(s)':<12} {'Notes':<30}")
        print("-"*80)
        for r in tester.results:
            emoji = {
                "STABLE": "✅",
                "MARGINAL": "⚠️",
                "UNSTABLE": "❌",
                "SKIPPED": "⏭️"
            }
            status = f"{emoji.get(r.classification, '❓')} {r.classification}"
            notes_short = r.notes[:27] + "..." if len(r.notes) > 30 else r.notes
            print(f"{r.config_name:<35} {status:<12} {r.test_duration_s:<11.1f} {notes_short:<30}")
    
    print()
    print(f"Results saved: {tester.results_file}")
    print()
    
    # Recommendations
    if stable:
        print("="*80)
        print("✅ STABLE CONFIGS (safe for Task 4 - Force Resistance Tests):")
        print("="*80)
        for r in stable:
            print(f"   - {r.config_name}")
    
    if marginal:
        print()
        print("="*80)
        print("⚠️  MARGINAL CONFIGS (use with caution):")
        print("="*80)
        for r in marginal:
            print(f"   - {r.config_name}")
            if r.notes != "No notes":
                print(f"     Note: {r.notes}")
    
    print()


def main():
    """Run interactive convergence tests."""
    print("="*80)
    print(" " * 15 + "TASK 3: INTERACTIVE CONVERGENCE TESTS")
    print("="*80)
    print()
    print("This tool helps you manually classify each config's stability.")
    print()
    print("For each config:")
    print("  1. Config loads in Isaac Sim")
    print("  2. You press PLAY and observe (10-30s)")
    print("  3. You classify: STABLE / MARGINAL / UNSTABLE")
    print()
    print("Results are saved after each test (can resume if interrupted).")
    print()
    
    # Create tester
    tester = InteractiveTester()
    
    # Get configs
    all_configs = get_test_configs()
    tested_configs = tester._get_tested_configs()
    
    # Filter to untested
    configs_to_test = [(name, path) for name, path in all_configs if name not in tested_configs]
    
    print(f"Found {len(all_configs)} total configs")
    print(f"Already tested: {len(tested_configs)}")
    print(f"Remaining: {len(configs_to_test)}")
    print()
    
    if not configs_to_test:
        print("✅ All configs already tested!")
        print_summary(tester)
        simulation_app.close()
        return
    
    print("Starting tests...")
    print("(Press Ctrl+C to skip a config, or exit completely)")
    print()
    
    try:
        for i, (config_name, usd_path) in enumerate(configs_to_test, 1):
            print(f"\n{'='*80}")
            print(f"TEST {i}/{len(configs_to_test)}: {config_name}")
            print(f"{'='*80}")
            
            tester.run_test(config_name, usd_path)
            
            # Ask if user wants to continue
            if i < len(configs_to_test):
                print()
                cont = input("Continue to next config? (y/n, default=y): ").strip().lower()
                if cont == 'n':
                    print("Stopping tests (results saved)")
                    break
    
    except KeyboardInterrupt:
        print("\n\n⚠️  Testing interrupted by user")
    
    # Final summary
    print_summary(tester)
    
    simulation_app.close()


if __name__ == "__main__":
    main()
