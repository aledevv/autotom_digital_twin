from exporterV1.loader import load_snapshot
from exporterV1.debug_viz import visualize_snapshot
from exporterV1.usd_exporter import export_plant_usd
from exporterV1.graph_export import export_graph_json

day = 1
plant_id = 1

# for day in range(1, 161):
snapshot = load_snapshot(f"data/simulation_output/dynamic_output/graphs/graph_day_{day}.csv", day=day, plant_id=plant_id)
# visualize_snapshot(snapshot, f"./output/day_{day}/plant_day{day}.html")
# export_graph_json(snapshot, f"./output/day_{day}/plant_day{day}.json") # just for debug
export_plant_usd(snapshot, f"./output/day_{day}/plant_day{day}.usda")

"""
python -m plant_usd_exporter --csv plant_organs.csv --day 10 --plant 1 --out plant_day10.usda
"""
