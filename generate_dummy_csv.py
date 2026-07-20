import csv
import random

CSV_PATH = "data/usd_models/generated_generalized_articulation_config.csv"

# 30 internodes, length random between 0.1 and 0.2
num_internodes = 30
rows = []
for i in range(num_internodes):
    ilength = random.uniform(0.1, 0.2)
    iid = f"Internode_{i:02d}"
    rows.append({
        "id": iid,
        "organ_class": "Internode",
        "parent_id": f"Internode_{i-1:02d}" if i > 0 else "",
        "length": round(ilength, 4),
        "width_m": 0.2,
        "parent_segment_idx": "",
        "z_offset_ratio": "",
        "tilt_angle": "",
        "rot_angle": ""
    })

# 10 branches
for i in range(10):
    parent_idx = random.randint(1, num_internodes - 1)
    rows.append({
        "id": f"Branch_{i:02d}",
        "organ_class": "Branch",
        "parent_id": f"Internode_{parent_idx:02d}",
        "length": round(random.uniform(1.0, 3.0), 4),
        "width_m": 0.08,
        "parent_segment_idx": "",
        "z_offset_ratio": 1.0,
        "tilt_angle": round(random.uniform(30.0, 75.0), 4),
        "rot_angle": round(random.uniform(0.0, 360.0), 4)
    })

with open(CSV_PATH, "w", newline='') as f:
    writer = csv.DictWriter(f, fieldnames=rows[0].keys())
    writer.writeheader()
    writer.writerows(rows)

print(f"Successfully wrote {len(rows)} rows to {CSV_PATH}")
