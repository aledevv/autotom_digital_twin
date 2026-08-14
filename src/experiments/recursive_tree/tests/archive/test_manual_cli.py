#!/usr/bin/env python3
"""
Task 3: Manual CLI Convergence Test

Simple workflow:
1. Script loads config in Isaac Sim
2. You press PLAY, observe, close Isaac Sim
3. Script asks classification in terminal
4. Repeat

Usage:
    cd ~/isaacsim/autotom_digital_twin
    python3 src/experiments/recursive_tree/tests/test_manual_cli.py
"""
import os
import sys
import json
import subprocess
from pathlib import Path
from dataclasses import dataclass, asdict
from typing import List
from datetime import datetime


SCRIPT_DIR = Path(__file__).parent.resolve()
RESULTS_FILE = SCRIPT_DIR / "convergence_results.json"


@dataclass
class Result:
    config_name: str
    classification: str
    notes: str
    timestamp: str


def load_results() -> List[Result]:
    """Load previous results."""
    if RESULTS_FILE.exists():
        with open(RESULTS_FILE, 'r') as f:
            data = json.load(f)
            return [Result(**r) for r in data]
    return []


def save_results(results: List[Result]):
    """Save results to JSON."""
    with open(RESULTS_FILE, 'w') as f:
        json.dump([asdict(r) for r in results], f, indent=2)
    print(f"✅ Saved to: {RESULTS_FILE}")


def get_tested_configs(results: List[Result]) -> set:
    """Get set of tested config names."""
    return {r.config_name for r in results if r.classification != "SKIPPED"}


def get_configs() -> List[tuple]:
    """Get list of (name, path) configs."""
    usd_dir = SCRIPT_DIR / "scalability_usds"
    return [(f.stem, str(f)) for f in sorted(usd_dir.glob("*.usda"))]


def load_in_isaac_sim(usd_path: str):
    """Load USD in Isaac Sim and wait for user to close it."""
    # Create temporary loader script
    loader_script = SCRIPT_DIR / "_temp_loader.py"
    
    script_content = f'''
from omni.isaac.kit import SimulationApp
config = {{"headless": False, "width": 1920, "height": 1080}}
simulation_app = SimulationApp(config)

from omni.isaac.core import World
from pxr import Usd

# Create world
world = World(stage_units_in_meters=1.0, physics_dt=1/480.0, rendering_dt=1/60.0)
world.scene.add_default_ground_plane()

# Load USD
stage = world.stage
tree_prim = stage.DefinePrim("/World/tomato", "Xform")
tree_prim.GetReferences().AddReference("{usd_path}")

world.reset()

print("\\n" + "="*80)
print("Config loaded! Press PLAY to test, then CLOSE Isaac Sim when done.")
print("="*80 + "\\n")

# Run until user closes
while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()
'''
    
    with open(loader_script, 'w') as f:
        f.write(script_content)
    
    # Run Isaac Sim
    isaacsim_python = Path.home() / "isaacsim" / "python.sh"
    subprocess.run([str(isaacsim_python), str(loader_script)])
    
    # Clean up
    loader_script.unlink()


def get_classification() -> tuple:
    """Ask user for classification."""
    print("\n" + "="*80)
    print("CLASSIFICATION")
    print("="*80)
    print()
    print("How would you classify this configuration?")
    print()
    print("  1 = ✅ STABLE      (quick settle, no oscillations)")
    print("  2 = ⚠️  MARGINAL   (slow settle, some oscillations)")
    print("  3 = ❌ UNSTABLE    (never settles, continuous jitter)")
    print("  0 = ⏭️  SKIP       (test later)")
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
            print("❌ Invalid. Enter 0, 1, 2, or 3")
    
    notes = input("\nOptional notes (press Enter to skip): ").strip()
    if not notes:
        notes = "No notes"
    
    return classification, notes


def print_summary(results: List[Result]):
    """Print summary."""
    print("\n" + "="*80)
    print("SUMMARY")
    print("="*80)
    
    stable = [r for r in results if r.classification == "STABLE"]
    marginal = [r for r in results if r.classification == "MARGINAL"]
    unstable = [r for r in results if r.classification == "UNSTABLE"]
    skipped = [r for r in results if r.classification == "SKIPPED"]
    
    print(f"\n✅ STABLE:   {len(stable)}")
    print(f"⚠️  MARGINAL: {len(marginal)}")
    print(f"❌ UNSTABLE: {len(unstable)}")
    print(f"⏭️  SKIPPED:  {len(skipped)}")
    print(f"📊 TESTED:   {len(stable)+len(marginal)+len(unstable)}/{len(results)}")
    
    if results:
        print(f"\n{'Config':<35} {'Status':<12} {'Notes':<40}")
        print("-"*80)
        for r in results:
            emoji = {"STABLE": "✅", "MARGINAL": "⚠️", "UNSTABLE": "❌", "SKIPPED": "⏭️"}
            status = f"{emoji.get(r.classification, '❓')} {r.classification}"
            notes = r.notes[:37] + "..." if len(r.notes) > 40 else r.notes
            print(f"{r.config_name:<35} {status:<12} {notes:<40}")
    
    print(f"\n📁 Results: {RESULTS_FILE}\n")
    
    if stable:
        print("="*80)
        print("✅ STABLE CONFIGS (ready for Task 4):")
        for r in stable:
            print(f"   - {r.config_name}")
        print()


def main():
    print("="*80)
    print("TASK 3: MANUAL CLI CONVERGENCE TEST")
    print("="*80)
    print()
    print("Simple workflow:")
    print("  1. Script loads config in Isaac Sim")
    print("  2. You press PLAY, observe, then CLOSE Isaac Sim")
    print("  3. Script asks for classification")
    print("  4. Repeat")
    print()
    
    # Load existing results
    results = load_results()
    
    # Get configs - TEST ONLY baseline_x10_scale by default
    all_configs = get_configs()
    tested = get_tested_configs(results)
    
    # Priority: test baseline_x10_scale first
    baseline_x10 = [(n, p) for n, p in all_configs if n == "baseline_x10_scale"]
    other_configs = [(n, p) for n, p in all_configs if n != "baseline_x10_scale" and n not in tested]
    
    # Start with baseline_x10, then others
    to_test = baseline_x10 + other_configs
    
    print(f"Total configs: {len(all_configs)}")
    print(f"Already tested: {len(tested)}")
    print(f"Remaining: {len(to_test)}")
    print()
    
    if not to_test:
        print("✅ All configs already tested!")
        print_summary(results)
        return
    
    try:
        for i, (config_name, usd_path) in enumerate(to_test, 1):
            print("\n" + "="*80)
            print(f"TEST {i}/{len(to_test)}: {config_name}")
            print("="*80)
            print(f"USD: {Path(usd_path).name}")
            print()
            input("Press ENTER to load in Isaac Sim...")
            
            # Load in Isaac Sim
            print("\nLoading Isaac Sim...")
            print("(Press PLAY to test, then CLOSE Isaac Sim when done)")
            load_in_isaac_sim(usd_path)
            
            # Get classification
            classification, notes = get_classification()
            
            # Save result
            result = Result(
                config_name=config_name,
                classification=classification,
                notes=notes,
                timestamp=datetime.now().isoformat()
            )
            results.append(result)
            save_results(results)
            
            emoji = {"STABLE": "✅", "MARGINAL": "⚠️", "UNSTABLE": "❌", "SKIPPED": "⏭️"}
            print(f"\n{emoji.get(classification, '❓')} Recorded: {classification}")
            
            # Continue?
            if i < len(to_test):
                cont = input("\nContinue to next? (y/n, default=y): ").strip().lower()
                if cont == 'n':
                    print("Stopping (results saved)")
                    break
    
    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupted by user")
    
    # Summary
    print_summary(results)


if __name__ == "__main__":
    main()
