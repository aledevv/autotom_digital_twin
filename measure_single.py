import os, sys, argparse, math
parser = argparse.ArgumentParser()
parser.add_argument("--usd", required=True)
parser.add_argument("--label", required=True)
args = parser.parse_args()

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
import omni
from isaacsim.core.api import World
from isaacsim.core.api.articulations import ArticulationView

omni.usd.get_context().open_stage(args.usd)
simulation_app.update()

world = World(stage_units_in_meters=1.0)
world.reset()

art_view = ArticulationView(prim_paths_expr="/World/Stem", name="stem_view")
world.scene.add(art_view)
world.reset()

# 0-indexed: last body = tip
n_links = int(args.label.split("=")[1])
tip_body_idx = n_links # root_anchor + n_links, tip is index n_links

# Settle for 10 seconds (600 steps)
for _ in range(600):
    world.step(render=False)

pos_f, _ = art_view.get_world_poses()
if pos_f.ndim == 3:
    tip_f = pos_f[0, tip_body_idx, :]
else:
    tip_f = pos_f[tip_body_idx, :]

zf = float(tip_f[2])
yf = float(tip_f[1])
z0 = 0.10
print(f"MEASUREMENT|{args.label}|{zf:.5f}|{(z0-zf)*1000:.2f}|{yf:.5f}")

world.stop()
world.clear_instance()
omni.usd.get_context().close_stage()
simulation_app.close()
