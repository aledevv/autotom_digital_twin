import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = SCRIPT_DIR

sys.path.insert(0, os.path.join(PROJECT_ROOT, "src"))

from plant_model.loader import load_snapshot
from plant_model.models import LeafNode, InternodeNode

def main():
    csv_path = os.path.join(PROJECT_ROOT, "data/simulation_output/dynamic_output/graphs", "graph_day_1.csv")
    
    print(f"Loading snapshot day 1...")
    snapshot = load_snapshot(csv_path, day=1, plant_id=1)
    
    print("\n--- INTERNODES ---")
    internodes = [n for n in snapshot.organs if isinstance(n, InternodeNode)]
    for n in internodes:
        print(f"[{n.key.organ_class} o={n.key.order} r={n.key.rank} i={n.key.organ_index}] "
              f"length={n.length:.5f}m, width={n.width_m:.5f}m")
              
    print("\n--- LEAVES ---")
    leaves = [n for n in snapshot.organs if isinstance(n, LeafNode)]
    for n in leaves:
        print(f"[{n.key.organ_class} o={n.key.order} r={n.key.rank} i={n.key.organ_index}] "
              f"length={n.length:.5f}m, length_petiole={n.length_petiole:.5f}m, "
              f"rachis_length={n.rachis_length:.5f}m, "
              f"area_total={n.area_blades_total:.5f}m2")

if __name__ == "__main__":
    main()
