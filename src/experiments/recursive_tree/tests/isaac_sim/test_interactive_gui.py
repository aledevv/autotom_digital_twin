#!/usr/bin/env python3
"""
Task 3: Interactive Convergence Test with GUI Buttons

GUI buttons in Isaac Sim viewport for classification - no terminal input needed!

Usage:
    ./run_experiment.sh recursive_tree tests/test_interactive_gui.py
"""
import os
import sys
import json
import time
from pathlib import Path
from dataclasses import dataclass, asdict
from typing import Dict, List, Optional

# Add parent to path
SCRIPT_DIR = Path(__file__).parent.resolve()
RECURSIVE_TREE_DIR = SCRIPT_DIR.parent.parent
sys.path.insert(0, str(RECURSIVE_TREE_DIR))

# Isaac Sim
from omni.isaac.kit import SimulationApp
config = {"headless": False, "width": 1920, "height": 1080}
simulation_app = SimulationApp(config)

from pxr import Usd, UsdGeom
from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.stage import get_current_stage
import omni.ui as ui
import carb


@dataclass
class Result:
    config_name: str
    classification: str
    notes: str
    test_duration_s: float
    timestamp: str


class InteractiveGUITest:
    def __init__(self):
        self.world = None
        self.articulation = None
        self.results_file = SCRIPT_DIR / "convergence_results_interactive.json"
        self.results = []
        
        # Current test state
        self.current_config_name = None
        self.current_usd_path = None
        self.test_start_time = None
        self.configs_to_test = []
        self.current_index = 0
        
        # GUI
        self.window = None
        self.status_label = None
        self.config_label = None
        self.notes_field = None
        
        # Load previous
        self._load_results()
        
    def _load_results(self):
        if self.results_file.exists():
            with open(self.results_file, 'r') as f:
                data = json.load(f)
                self.results = [Result(**r) for r in data]
            print(f"✅ Loaded {len(self.results)} previous results", flush=True)
    
    def _save_results(self):
        with open(self.results_file, 'w') as f:
            json.dump([asdict(r) for r in self.results], f, indent=2)
        print(f"✅ Results saved", flush=True)
    
    def _get_tested_configs(self) -> set:
        return {r.config_name for r in self.results if r.classification != "SKIPPED"}
    
    def create_gui(self):
        """Create GUI window with classification buttons."""
        self.window = ui.Window("Convergence Test Controls", width=400, height=500)
        
        with self.window.frame:
            with ui.VStack(spacing=10):
                ui.Label("CONVERGENCE TEST", height=30, 
                        style={"font_size": 20, "color": 0xFFFFFFFF})
                
                ui.Spacer(height=10)
                
                # Status
                with ui.HStack():
                    ui.Label("Status:", width=80)
                    self.status_label = ui.Label("Initializing...", 
                                                 style={"color": 0xFFFFFF00})
                
                # Current config
                with ui.HStack():
                    ui.Label("Config:", width=80)
                    self.config_label = ui.Label("None", 
                                                 style={"color": 0xFFFFFFFF})
                
                ui.Spacer(height=20)
                ui.Line(style={"color": 0xFF666666})
                ui.Spacer(height=10)
                
                # Instructions
                ui.Label("1. Press PLAY to test", height=20)
                ui.Label("2. Observe 10-30s", height=20)
                ui.Label("3. Click classification:", height=20)
                
                ui.Spacer(height=10)
                
                # Classification buttons
                with ui.VStack(spacing=10):
                    ui.Button("✅ STABLE - Quick settle, no oscillation", 
                             height=50, clicked_fn=lambda: self.classify("STABLE"),
                             style={"background_color": 0xFF00AA00})
                    
                    ui.Button("⚠️  MARGINAL - Slow settle, some oscillation", 
                             height=50, clicked_fn=lambda: self.classify("MARGINAL"),
                             style={"background_color": 0xFFFF8800})
                    
                    ui.Button("❌ UNSTABLE - Never settles, continuous jitter", 
                             height=50, clicked_fn=lambda: self.classify("UNSTABLE"),
                             style={"background_color": 0xFFAA0000})
                    
                    ui.Button("⏭️  SKIP - Test later", 
                             height=40, clicked_fn=lambda: self.classify("SKIPPED"),
                             style={"background_color": 0xFF444444})
                
                ui.Spacer(height=10)
                ui.Line(style={"color": 0xFF666666})
                ui.Spacer(height=10)
                
                # Notes
                ui.Label("Optional Notes:", height=20)
                self.notes_field = ui.StringField(height=30)
                
                ui.Spacer(height=20)
                
                # Progress
                self.progress_label = ui.Label("Progress: 0/0", 
                                              style={"color": 0xFFAAAAFF})
        
        print("✅ GUI created", flush=True)
    
    def setup_world(self):
        """Create world."""
        if self.world is not None:
            self.world.clear()
        
        self.world = World(stage_units_in_meters=1.0, 
                          physics_dt=1/480.0, 
                          rendering_dt=1/60.0)
        self.world.scene.add_default_ground_plane()
        print("✅ World ready", flush=True)
    
    def load_config(self, config_name: str, usd_path: str):
        """Load USD config."""
        self.current_config_name = config_name
        self.current_usd_path = usd_path
        self.test_start_time = time.time()
        
        # Clear previous
        if self.world is None:
            self.setup_world()
        else:
            stage = get_current_stage()
            if stage:
                prim = stage.GetPrimAtPath("/World/tomato")
                if prim.IsValid():
                    stage.RemovePrim("/World/tomato")
        
        # Load
        stage = get_current_stage()
        tree_prim = stage.DefinePrim("/World/tomato", "Xform")
        tree_prim.GetReferences().AddReference(usd_path)
        
        self.world.reset()
        
        # Get articulation
        self.articulation = Articulation("/World/tomato/stem")
        self.articulation.initialize()
        
        # Update GUI
        self.config_label.text = config_name
        self.status_label.text = "Ready - Press PLAY"
        self.status_label.style = {"color": 0xFF00FF00}
        self.progress_label.text = f"Progress: {self.current_index + 1}/{len(self.configs_to_test)}"
        
        print(f"\n{'='*60}", flush=True)
        print(f"CONFIG: {config_name}", flush=True)
        print(f"DOFs: {self.articulation.num_dof}", flush=True)
        print(f"{'='*60}\n", flush=True)
    
    def classify(self, classification: str):
        """Handle classification button click."""
        if self.current_config_name is None:
            print("⚠️  No config loaded!", flush=True)
            return
        
        duration = time.time() - self.test_start_time
        notes = self.notes_field.model.get_value_as_string()
        if not notes:
            notes = "No notes"
        
        from datetime import datetime
        result = Result(
            config_name=self.current_config_name,
            classification=classification,
            notes=notes,
            test_duration_s=duration,
            timestamp=datetime.now().isoformat()
        )
        
        self.results.append(result)
        self._save_results()
        
        emoji = {"STABLE": "✅", "MARGINAL": "⚠️", "UNSTABLE": "❌", "SKIPPED": "⏭️"}
        print(f"\n{emoji.get(classification, '❓')} Recorded: {classification}", flush=True)
        print(f"   Config: {self.current_config_name}", flush=True)
        print(f"   Duration: {duration:.1f}s", flush=True)
        if notes != "No notes":
            print(f"   Notes: {notes}", flush=True)
        
        # Clear notes
        self.notes_field.model.set_value("")
        
        # Load next
        self.current_index += 1
        if self.current_index < len(self.configs_to_test):
            config_name, usd_path = self.configs_to_test[self.current_index]
            self.load_config(config_name, usd_path)
        else:
            self.finish_testing()
    
    def finish_testing(self):
        """All tests complete."""
        self.status_label.text = "ALL DONE! ✅"
        self.status_label.style = {"color": 0xFF00FFFF}
        self.config_label.text = "Testing complete"
        
        print("\n" + "="*60, flush=True)
        print("ALL TESTS COMPLETE!", flush=True)
        print("="*60 + "\n", flush=True)
        
        self.print_summary()
    
    def print_summary(self):
        """Print results summary."""
        stable = [r for r in self.results if r.classification == "STABLE"]
        marginal = [r for r in self.results if r.classification == "MARGINAL"]
        unstable = [r for r in self.results if r.classification == "UNSTABLE"]
        skipped = [r for r in self.results if r.classification == "SKIPPED"]
        
        print(f"✅ STABLE:   {len(stable)}", flush=True)
        print(f"⚠️  MARGINAL: {len(marginal)}", flush=True)
        print(f"❌ UNSTABLE: {len(unstable)}", flush=True)
        print(f"⏭️  SKIPPED:  {len(skipped)}", flush=True)
        print(f"\n📊 Results: {self.results_file}\n", flush=True)
        
        if stable:
            print("✅ STABLE configs:", flush=True)
            for r in stable:
                print(f"   - {r.config_name}", flush=True)
        
        if unstable:
            print("\n❌ UNSTABLE configs:", flush=True)
            for r in unstable:
                print(f"   - {r.config_name}", flush=True)
    
    def run(self):
        """Main loop."""
        # Get configs
        usd_dir = SCRIPT_DIR / "scalability_usds"
        all_configs = [(f.stem, str(f)) for f in sorted(usd_dir.glob("*.usda"))]
        tested = self._get_tested_configs()
        self.configs_to_test = [(n, p) for n, p in all_configs if n not in tested]
        
        print(f"\nTotal configs: {len(all_configs)}", flush=True)
        print(f"Already tested: {len(tested)}", flush=True)
        print(f"Remaining: {len(self.configs_to_test)}\n", flush=True)
        
        if not self.configs_to_test:
            print("✅ All configs already tested!", flush=True)
            self.print_summary()
            return
        
        # Create GUI
        self.create_gui()
        
        # Load first config
        self.current_index = 0
        config_name, usd_path = self.configs_to_test[0]
        self.load_config(config_name, usd_path)
        
        # Run simulation loop
        while simulation_app.is_running():
            self.world.step(render=True)
        
        simulation_app.close()


def main():
    print("="*60, flush=True)
    print("TASK 3: INTERACTIVE CONVERGENCE TEST (GUI)", flush=True)
    print("="*60, flush=True)
    print("\nLook for GUI window 'Convergence Test Controls'", flush=True)
    print("in Isaac Sim interface\n", flush=True)
    
    tester = InteractiveGUITest()
    tester.run()


if __name__ == "__main__":
    main()
