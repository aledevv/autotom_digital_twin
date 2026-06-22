# main.py
from plant_model.loader import load_snapshot
from plant_model.debug_viz import visualize_snapshot
from plant_model.usd_exporter import export_plant_usd

snapshot = load_snapshot("data/dynamic_output/graphs/dummy.csv", day=1, plant_id=1)
visualize_snapshot(snapshot, "plant_day01.html")

export_plant_usd(snapshot, "plant_day01.usda")

"""
python -m plant_usd_exporter --csv plant_organs.csv --day 10 --plant 1 --out plant_day10.usda
"""
