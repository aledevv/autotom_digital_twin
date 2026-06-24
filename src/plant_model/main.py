# main.py
from plant_model.loader import load_snapshot
from plant_model.debug_viz import visualize_snapshot
from plant_model.usd_exporter import export_plant_usd
from plant_model.graph_export import export_graph_json

day = 10
plant_id = 1

snapshot = load_snapshot(f"data/dynamic_output/graphs/graph_day_{day}.csv", day=day, plant_id=plant_id)
visualize_snapshot(snapshot, f"./output/day_{day}/plant_day{day}.html")

export_plant_usd(snapshot, f"./output/day_{day}/plant_day{day}.usda")

export_graph_json(snapshot, f"./output/day_{day}/plant_day{day}.json")

"""
python -m plant_usd_exporter --csv plant_organs.csv --day 10 --plant 1 --out plant_day10.usda
"""
