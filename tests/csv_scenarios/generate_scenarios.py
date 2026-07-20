import csv
import os

OUT_DIR = os.path.dirname(os.path.abspath(__file__))

def create_csv(filename, internodes, branches):
    path = os.path.join(OUT_DIR, filename)
    rows = internodes + branches
    if not rows:
        return
        
    with open(path, "w", newline='') as f:
        writer = csv.DictWriter(f, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    print(f"Created {path}")

def make_internode(idx, length, parent_idx=None):
    return {
        "id": f"Internode_{idx:02d}",
        "organ_class": "Internode",
        "parent_id": f"Internode_{parent_idx:02d}" if parent_idx is not None else "",
        "length": round(length, 4),
        "width_m": 0.2,
        "parent_segment_idx": "",
        "z_offset_ratio": "",
        "tilt_angle": "",
        "rot_angle": ""
    }

def make_branch(idx, parent_internode_idx, length, z_offset_ratio=1.0, tilt=45.0, rot=0.0):
    return {
        "id": f"Branch_{idx:02d}",
        "organ_class": "Branch",
        "parent_id": f"Internode_{parent_internode_idx:02d}",
        "length": round(length, 4),
        "width_m": 0.08,
        "parent_segment_idx": "",
        "z_offset_ratio": z_offset_ratio,
        "tilt_angle": tilt,
        "rot_angle": rot
    }

def generate_scenario_1():
    # 10 internodes (no merging), 3 branches
    internodes = [make_internode(i, 0.15, i-1 if i>0 else None) for i in range(10)]
    branches = [
        make_branch(1, 3, 1.5, 1.0, 45, 0),
        make_branch(2, 6, 2.0, 1.0, 45, 120),
        make_branch(3, 9, 1.0, 1.0, 45, 240)
    ]
    create_csv("scenario_1_small_tree.csv", internodes, branches)

def generate_scenario_2():
    # 100 internodes (extreme merging), random branches
    internodes = [make_internode(i, 0.05, i-1 if i>0 else None) for i in range(100)]
    branches = []
    import random
    for i in range(20):
        parent_idx = random.randint(10, 95)
        branches.append(make_branch(i+1, parent_idx, random.uniform(1.0, 3.0), 1.0, 45, random.uniform(0, 360)))
    create_csv("scenario_2_huge_tree.csv", internodes, branches)

def generate_scenario_3():
    # Clustered branches to test Z-ratio spacing
    # 40 internodes total. We attach 10 branches to internodes 15, 16, 17.
    # When merged, these will likely fall on the same physical segment!
    internodes = [make_internode(i, 0.1, i-1 if i>0 else None) for i in range(40)]
    branches = []
    branch_idx = 1
    for parent_idx in [15, 16, 17]:
        for b in range(3):
            branches.append(make_branch(branch_idx, parent_idx, 1.5, z_offset_ratio=1.0, tilt=60, rot=b*120))
            branch_idx += 1
    create_csv("scenario_3_clustered_branches.csv", internodes, branches)

def generate_scenario_4():
    # Massive branches up high to stress test D6 joints
    internodes = [make_internode(i, 0.2, i-1 if i>0 else None) for i in range(30)]
    branches = []
    # Very long, heavy branches at the very top (Internode 29)
    for i in range(4):
        branches.append(make_branch(i+1, 29, 6.0, z_offset_ratio=1.0, tilt=80, rot=i*90))
    create_csv("scenario_4_massive_branches.csv", internodes, branches)

if __name__ == "__main__":
    generate_scenario_1()
    generate_scenario_2()
    generate_scenario_3()
    generate_scenario_4()
