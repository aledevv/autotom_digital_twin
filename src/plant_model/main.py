# main.py
from plant_model.loader import load_snapshot
from plant_model.debug_viz import visualize_snapshot

snapshot = load_snapshot("data/dynamic_output/graphs/dummy.csv", day=1, plant_id=1)
visualize_snapshot(snapshot, "plant_day1.html")