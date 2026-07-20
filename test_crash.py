import os
import sys

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import generate_generalized_articulation_usda

# Set high segments to reproduce the crash
generate_generalized_articulation_usda.Config.MAX_STEM_SEGMENTS = 50
generate_generalized_articulation_usda.Config.MAX_BRANCH_SEGMENTS = 20

stage, stem_path = generate_generalized_articulation_usda.build_stage(generate_generalized_articulation_usda.get_output_usd_path())

import load_generalized_articulation
load_generalized_articulation.apply_physx_scene_settings(stage)
load_generalized_articulation.apply_physx_articulation_settings(stage, stem_path)

from isaacsim.core.api import World
my_world = World(stage_units_in_meters=1.0)
my_world.reset()

print("Stepping simulation to reproduce crash...")
for i in range(10):
    my_world.step(render=False)

simulation_app.close()
