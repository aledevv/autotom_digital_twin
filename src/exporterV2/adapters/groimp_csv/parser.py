"""
parser.py - Generic CSV Parsing

Handles loading and filtering of groIMP CSV export data.
"""

import os
import json
import pandas as pd
from datetime import datetime
from typing import List, Dict, Tuple
from pathlib import Path
from collections import defaultdict


def _parse_float_array(val_str: str) -> List[float]:
    """
    Parse float arrays from CSV strings.
    Example: "0.1_0.2_0.3" -> [0.1, 0.2, 0.3]
    Empty or "0" returns empty list.
    """
    s = str(val_str).strip()
    if s in ("0", "0.0", "", "nan", "None"):
        return []
    return [float(x) for x in s.split("_")]


def load_trunk_internodes(csv_path: str, day: int, plant_id: int = 1) -> List[Dict]:
    """
    Load trunk internodes (order=0) from groIMP CSV export.
    
    Args:
        csv_path: Path to CSV file (e.g., graph_day_1.csv)
        day: Simulation day
        plant_id: Plant identifier (default: 1)
    
    Returns:
        List of internode dicts with fields: rank, organ_index, width_m, length
        Sorted by rank (ascending, from base to top)
    
    Raises:
        ValueError: If no trunk internodes found
        FileNotFoundError: If CSV file doesn't exist
    """
    if not os.path.exists(csv_path):
        raise FileNotFoundError(f"CSV file not found: {csv_path}")
    
    # Read CSV with space handling
    df = pd.read_csv(csv_path, skipinitialspace=True)
    
    # Clean column names
    df.columns = df.columns.str.strip()
    
    # Filter for trunk internodes: order=0 (main stem)
    mask = (
        (df["day"] == day) &
        (df["plant_id"] == plant_id) &
        (df["organ_class"].str.strip() == "Internode") &
        (df["order"] == 0)
    )
    trunk_df = df[mask].copy()
    
    if trunk_df.empty:
        raise ValueError(f"No trunk internodes found for day={day}, plant_id={plant_id}, order=0")
    
    # Sort by rank (ascending: base to top)
    trunk_df = trunk_df.sort_values("rank")
    
    # Extract relevant fields
    internodes = []
    for _, row in trunk_df.iterrows():
        internodes.append({
            "rank": int(row["rank"]),
            "organ_index": int(row["organ_index"]),
            "width_m": float(row["internode_width_m"]),
            "length": float(row["length"]),
        })
    
    print(f"[INFO] Found {len(internodes)} trunk internodes for day {day}")
    
    return internodes


def load_lateral_branches(csv_path: str, day: int, plant_id: int = 1) -> List[Dict]:
    """
    Load lateral branches (order=1) from groIMP CSV export with opposite pair filtering.
    
    NOTE: Opposite pair filtering is specific to this tomato cultivar.
          Other cultivars may have different branching patterns.
    
    Filtering logic:
    - Group branches by (rank, parent_rank)
    - Keep both branches if they form an opposite pair (organ_index 0 and 1)
    - Remove clones (same position)
    
    Args:
        csv_path: Path to CSV file (e.g., graph_day_1.csv)
        day: Simulation day
        plant_id: Plant identifier (default: 1)
    
    Returns:
        List of lateral branch dicts with fields: rank, organ_index, parent_rank, 
        width_m, length
        Sorted by parent_rank, then rank, then organ_index (ascending)
    
    Raises:
        FileNotFoundError: If CSV file doesn't exist
    """
    if not os.path.exists(csv_path):
        raise FileNotFoundError(f"CSV file not found: {csv_path}")
    
    # Read CSV with space handling
    df = pd.read_csv(csv_path, skipinitialspace=True)
    
    # Clean column names
    df.columns = df.columns.str.strip()
    
    # Filter for lateral branches: order=1 (attached to trunk internodes)
    mask = (
        (df["day"] == day) &
        (df["plant_id"] == plant_id) &
        (df["organ_class"].str.strip() == "Internode") &
        (df["order"] == 1)
    )
    branch_df = df[mask].copy()
    
    if branch_df.empty:
        print(f"[INFO] No lateral branches found for day={day}, plant_id={plant_id}")
        return []
    
    # Sort by parent_rank, rank, organ_index
    branch_df = branch_df.sort_values(["parent_rank", "rank", "organ_index"])
    
    # Group by (rank, parent_rank) to find opposite pairs
    from collections import defaultdict
    branches_by_key = defaultdict(list)
    
    for _, row in branch_df.iterrows():
        branch_data = {
            "rank": int(row["rank"]),
            "organ_index": int(row["organ_index"]),
            "parent_rank": int(row["parent_rank"]),
            "width_m": float(row["internode_width_m"]),
            "length": float(row["length"]),
        }
        key = (branch_data["rank"], branch_data["parent_rank"])
        branches_by_key[key].append(branch_data)
    
    # Filter: keep only organ_index 0 and 1 (opposite pair)
    # NOTE: This is specific to this cultivar's symmetric branching pattern
    filtered_branches = []
    n_pairs = 0
    n_singles = 0
    
    for key, key_branches in sorted(branches_by_key.items()):
        # Find organ_index 0 and 1
        branch_0 = next((b for b in key_branches if b["organ_index"] == 0), None)
        branch_1 = next((b for b in key_branches if b["organ_index"] == 1), None)
        
        if branch_0 and branch_1:
            # Both present - add as opposite pair
            filtered_branches.append(branch_0)
            filtered_branches.append(branch_1)
            n_pairs += 1
        elif branch_0:
            # Only organ_index 0
            filtered_branches.append(branch_0)
            n_singles += 1
        elif branch_1:
            # Only organ_index 1
            filtered_branches.append(branch_1)
            n_singles += 1
    
    # Sort final list by parent_rank, rank, organ_index
    filtered_branches.sort(key=lambda x: (x["parent_rank"], x["rank"], x["organ_index"]))
    
    print(f"[INFO] Found {len(filtered_branches)} lateral branches for day {day} "
          f"({n_pairs} opposite pairs + {n_singles} singles)")
    
    return filtered_branches


def load_leaves(csv_path: str, day: int, plant_id: int = 1, order: int = 0) -> List[Dict]:
    """
    Load leaves from groIMP CSV export with opposite pair filtering.
    
    NOTE: Opposite pair filtering (180° difference) is specific to this tomato cultivar.
          Other cultivars may have different leaf arrangement patterns (phyllotaxis).
    
    Filtering logic:
    - Group leaves by rank
    - Keep both leaves if they form an opposite pair (180° difference in ccw_orientation)
    - Keep single leaves without opposite pair
    - Remove clones (same position)
    
    Args:
        csv_path: Path to CSV file (e.g., graph_day_1.csv)
        day: Simulation day
        plant_id: Plant identifier (default: 1)
        order: Leaf order (0=trunk leaves, 1=lateral branch leaves)
    
    Returns:
        List of leaf dicts with fields: rank, organ_index, parent_rank, order, length_petiole, 
        diameter_petiole, angle_petiole, ccw_orientation, rachis_length,
        blades_nr, segments_length_array, inclination_array
        Sorted by rank, then organ_index (ascending)
    
    Raises:
        FileNotFoundError: If CSV file doesn't exist
    """
    if not os.path.exists(csv_path):
        raise FileNotFoundError(f"CSV file not found: {csv_path}")
    
    # Read CSV with space handling
    df = pd.read_csv(csv_path, skipinitialspace=True)
    
    # Clean column names
    df.columns = df.columns.str.strip()
    
    # Filter for leaves with specified order
    # For order=1 (lateral branch leaves): CULTIVAR-SPECIFIC filter for organ_index 0+1 only
    # (to match the lateral branch opposite pair filtering)
    if order == 1:
        mask = (
            (df["day"] == day) &
            (df["plant_id"] == plant_id) &
            (df["organ_class"].str.strip() == "Leaf") &
            (df["order"] == order) &
            (df["organ_index"].isin([0, 1]))  # CULTIVAR-SPECIFIC: opposite pairs only
        )
    else:
        # For order=0 (trunk leaves): NO organ_index filter - we need all to find opposite pairs
        mask = (
            (df["day"] == day) &
            (df["plant_id"] == plant_id) &
            (df["organ_class"].str.strip() == "Leaf") &
            (df["order"] == order)
        )
    leaf_df = df[mask].copy()
    
    if leaf_df.empty:
        order_str = "trunk" if order == 0 else f"lateral (order={order})"
        print(f"[INFO] No {order_str} leaves found for day={day}, plant_id={plant_id}")
        return []
    
    # Sort by rank, then organ_index
    leaf_df = leaf_df.sort_values(["rank", "organ_index"])
    
    # Group by rank to find opposite pairs
    leaves_by_rank = defaultdict(list)
    
    for _, row in leaf_df.iterrows():
        leaf_data = {
            "rank": int(row["rank"]),
            "organ_index": int(row["organ_index"]),
            "parent_rank": int(row["parent_rank"]),
            "order": int(row["order"]),
            "length_petiole": float(row["leaf_length_petiole"]),
            "diameter_petiole": float(row["leaf_diameter_petiole"]),
            "angle_petiole": float(row["leaf_angle_petiole"]),
            "ccw_orientation": float(row["leaf_ccw_orientation"]),
            "rachis_length": float(row["leaf_rachis_length"]),
            "blades_nr": int(row["leaf_blades_nr"]),
            "segments_length": _parse_float_array(row["leaf_segments_length"]),
            "inclination_segments": _parse_float_array(row["leaf_inclination_segments"]),
        }
        leaves_by_rank[leaf_data["rank"]].append(leaf_data)
    
    # Filter logic depends on order:
    # - order=0 (trunk leaves): Find opposite pairs (180° difference) or keep singles
    # - order=1 (lateral leaves): Keep all (already filtered to organ_index 0+1)
    #   NOTE: Both filtering strategies are cultivar-specific
    filtered_leaves = []
    n_pairs = 0
    n_singles = 0
    
    if order == 1:
        # Lateral branch leaves: keep all (no opposite pair filtering needed)
        # Already filtered to organ_index 0+1 at CSV load time
        filtered_leaves = list(leaves_by_rank.values())
        filtered_leaves = [leaf for rank_leaves in filtered_leaves for leaf in rank_leaves]
        n_singles = len(filtered_leaves)
    else:
        # Trunk leaves: apply opposite pair filtering
        for rank, rank_leaves in sorted(leaves_by_rank.items()):
            if len(rank_leaves) == 1:
                # Single leaf, always keep
                filtered_leaves.append(rank_leaves[0])
                n_singles += 1
            else:
                # Multiple leaves: find opposite pairs
                used = set()
                for i, leaf_i in enumerate(rank_leaves):
                    if i in used:
                        continue
                        
                    # Look for opposite pair
                    found_pair = False
                    for j, leaf_j in enumerate(rank_leaves):
                        if i >= j or j in used:
                            continue
                        
                        # Check if they form an opposite pair (180° difference)
                        angle_diff = abs(leaf_i["ccw_orientation"] - leaf_j["ccw_orientation"])
                        # Normalize to [0, 360)
                        angle_diff = angle_diff % 360
                        if angle_diff > 180:
                            angle_diff = 360 - angle_diff
                        
                        if abs(angle_diff - 180.0) < 1e-6:  # Exact 180° difference
                            # Found opposite pair
                            filtered_leaves.append(leaf_i)
                            filtered_leaves.append(leaf_j)
                            used.add(i)
                            used.add(j)
                            found_pair = True
                            n_pairs += 1
                            break
                    
                    # If no pair found, check for clones (same position)
                    if not found_pair and i not in used:
                        # Check if this is a clone of an already-added leaf
                        is_clone = False
                        for existing in filtered_leaves:
                            if (existing["rank"] == leaf_i["rank"] and
                                abs(existing["ccw_orientation"] - leaf_i["ccw_orientation"]) < 1e-6):
                                is_clone = True
                                break
                        
                        if not is_clone:
                            # Single leaf without pair, keep it
                            filtered_leaves.append(leaf_i)
                            used.add(i)
                            n_singles += 1
    
    # CULTIVAR-SPECIFIC: For lateral branch leaves (order=1), ensure both leaves in opposite pair exist
    # If only one leaf exists for a (rank, organ_index 0+1) pair, clone it for the missing organ_index
    if order == 1:
        # Group filtered leaves by rank
        leaves_by_rank_final = defaultdict(list)
        for leaf in filtered_leaves:
            leaves_by_rank_final[leaf["rank"]].append(leaf)
        
        cloned_leaves = []
        for rank, rank_leaves in leaves_by_rank_final.items():
            organ_indices = {leaf["organ_index"] for leaf in rank_leaves}
            
            # Check if we have both organ_index 0 and 1
            if 0 in organ_indices and 1 not in organ_indices:
                # Clone organ_index 0 → 1
                template = next(leaf for leaf in rank_leaves if leaf["organ_index"] == 0)
                cloned = template.copy()
                cloned["organ_index"] = 1
                cloned_leaves.append(cloned)
            elif 1 in organ_indices and 0 not in organ_indices:
                # Clone organ_index 1 → 0
                template = next(leaf for leaf in rank_leaves if leaf["organ_index"] == 1)
                cloned = template.copy()
                cloned["organ_index"] = 0
                cloned_leaves.append(cloned)
        
        filtered_leaves.extend(cloned_leaves)
        if cloned_leaves:
            print(f"[INFO] Cloned {len(cloned_leaves)} lateral leaves to complete opposite pairs")
    
    # Sort final list by rank, then organ_index
    filtered_leaves.sort(key=lambda x: (x["rank"], x["organ_index"]))
    
    order_str = "trunk" if order == 0 else f"lateral (order={order})"
    print(f"[INFO] Found {len(filtered_leaves)} {order_str} leaves for day {day} "
          f"({n_pairs} opposite pairs + {n_singles} singles)")
    
    return filtered_leaves


def lateral_branches_to_branch_config(lateral_branches: List[Dict], trunk_id: str = "trunk") -> List[Dict]:
    """
    Convert lateral branches to BRANCHES format.
    
    NOTE: Orientation logic (tilt=45°, symmetric rot) is specific to this cultivar.
          Other cultivars may require different branching angles.
    
    Uses average radius and height across all branches with same (rank, organ_index).
    Applies minimum radius clamping for PhysX stability.
    
    Args:
        lateral_branches: List of lateral branch dicts from load_lateral_branches()
        trunk_id: ID of trunk branch (default: "trunk")
    
    Returns:
        List of branch dicts in BRANCHES format
    """
    # Import tree_config directly to avoid pxr import in __init__
    import importlib.util
    config_path = Path(__file__).parent.parent.parent / "core" / "tree_config.py"
    spec = importlib.util.spec_from_file_location("tree_config", config_path)
    tree_config = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(tree_config)
    
    clamp_radius = tree_config.clamp_radius
    GLOBAL_SCALE = tree_config.GLOBAL_SCALE
    MIN_LINK_RADIUS_WORLD = tree_config.MIN_LINK_RADIUS_WORLD
    
    # Group by (rank, organ_index) to calculate averages
    from collections import defaultdict
    grouped = defaultdict(list)
    
    for branch in lateral_branches:
        key = (branch["rank"], branch["organ_index"])
        grouped[key].append(branch)
    
    # Convert each group to BRANCHES format
    branches = []
    
    for (rank, organ_index), group in sorted(grouped.items()):
        # Calculate averages
        avg_radius = sum(b["width_m"] / 2.0 for b in group) / len(group)
        avg_height = sum(b["length"] for b in group) / len(group)
        parent_rank = group[0]["parent_rank"]  # Same for all in group
        
        # Apply radius clamping
        radius_final, was_clamped = clamp_radius(avg_radius)
        
        if was_clamped:
            radius_world_original = avg_radius * GLOBAL_SCALE
            radius_world_clamped = radius_final * GLOBAL_SCALE
            print(f"[WARNING] Lateral branch rank={rank} organ_index={organ_index} radius clamped: "
                  f"{radius_world_original:.4f}m → {radius_world_clamped:.4f}m "
                  f"(min {MIN_LINK_RADIUS_WORLD}m at scale {GLOBAL_SCALE})")
        
        # Determine orientation
        # NOTE: This is cultivar-specific symmetric branching
        # organ_index 0 → rot=0°, organ_index 1 → rot=180° (opposite)
        # tilt = 45° (fixed angle from vertical)
        if organ_index == 0:
            rot_deg = 0.0
        elif organ_index == 1:
            rot_deg = 180.0
        else:
            # Fallback (shouldn't happen with our filtering)
            rot_deg = organ_index * 90.0
        
        tilt_deg = 45.0  # Fixed tilt for lateral branches
        
        # Create BRANCHES format dict
        branch = {
            "id": f"Branch_r{rank}_o{organ_index}",
            "parent": trunk_id,
            "attach_link": parent_rank + 1,  # 1-based indexing
            "n_links": len(group),
            "radius": radius_final,
            "height": avg_height,
            "tilt": tilt_deg,
            "rot": rot_deg,
        }
        
        branches.append(branch)
    
    return branches


def internodes_to_branch_config(internodes: List[Dict]) -> Dict:
    """
    Convert list of trunk internodes to BRANCHES format.
    
    Uses average radius and height across all internodes.
    Applies minimum radius clamping for PhysX stability.
    
    Args:
        internodes: List of trunk internode dicts from load_trunk_internodes()
    
    Returns:
        Branch dict in BRANCHES format (trunk configuration)
    """
    # Load tree_config directly to avoid pxr import in __init__
    import importlib.util
    config_path = Path(__file__).parent.parent / "tree_config.py"
    spec = importlib.util.spec_from_file_location("tree_config", config_path)
    tree_config = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(tree_config)
    
    clamp_radius = tree_config.clamp_radius
    GLOBAL_SCALE = tree_config.GLOBAL_SCALE
    MIN_LINK_RADIUS_WORLD = tree_config.MIN_LINK_RADIUS_WORLD
    
    # Calculate averages
    avg_radius = sum(i["width_m"] / 2.0 for i in internodes) / len(internodes)
    avg_height = sum(i["length"] for i in internodes) / len(internodes)
    
    # Apply radius clamping
    radius_final, was_clamped = clamp_radius(avg_radius)
    
    if was_clamped:
        radius_world_original = avg_radius * GLOBAL_SCALE
        radius_world_clamped = radius_final * GLOBAL_SCALE
        print(f"[WARNING] Trunk radius clamped: {radius_world_original:.4f}m → {radius_world_clamped:.4f}m "
              f"(min {MIN_LINK_RADIUS_WORLD}m at scale {GLOBAL_SCALE})")
    
    # Create BRANCHES format dict
    branch = {
        "id": "trunk",
        "parent": None,
        "attach_link": None,
        "n_links": len(internodes),
        "radius": radius_final,
        "height": avg_height,
        "tilt": 0.0,
        "rot": 0.0,
    }
    
    return branch


def save_branches_json(
    branches: List[Dict],
    day: int,
    output_dir: str,
    internodes: List[Dict],
    csv_filename: str
) -> str:
    """
    Save BRANCHES configuration to JSON with metadata.
    
    Args:
        branches: List of branch dicts in BRANCHES format
        day: Simulation day
        output_dir: Base output directory (e.g., "output")
        internodes: Original internode data for metadata
        csv_filename: Source CSV filename
    
    Returns:
        Absolute path to saved JSON file
    """
    # Import tree_config for metadata
    import importlib.util
    config_path = Path(__file__).parent.parent / "tree_config.py"
    spec = importlib.util.spec_from_file_location("tree_config", config_path)
    tree_config = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(tree_config)
    
    # Create output directory
    day_dir = Path(output_dir) / f"day_{day}"
    day_dir.mkdir(parents=True, exist_ok=True)
    
    output_path = day_dir / f"branches_v2_day_{day}.json"
    
    # Build JSON structure
    data = {
        "metadata": {
            "day": day,
            "plant_id": 1,  # Fixed to 1 as per requirements
            "generated_at": datetime.now().isoformat(),
            "source_csv": csv_filename,
            "n_branches": len(branches),
            "total_links": sum(b["n_links"] for b in branches),
            "global_scale": tree_config.GLOBAL_SCALE,
            "min_radius_world_m": tree_config.MIN_LINK_RADIUS_WORLD,
        },
        "branches": branches
    }
    
    # Save with pretty formatting
    with open(output_path, "w") as f:
        json.dump(data, f, indent=2)
    
    return str(output_path.absolute())


def parse_csv_to_branches(day: int, plant_id: int = 1) -> Tuple[List[Dict], str]:
    """
    Complete pipeline: CSV → internodes + leaves → BRANCHES → JSON.
    
    Args:
        day: Simulation day
        plant_id: Plant identifier (default: 1)
    
    Returns:
        Tuple (branches_list, json_path):
            branches_list: List of branch dicts in BRANCHES format (trunk + leaves)
            json_path: Absolute path to saved JSON file
    
    Raises:
        FileNotFoundError: If CSV file doesn't exist
        ValueError: If no trunk internodes found
    """
    # Import leaf_builder - handle both package and standalone execution
    try:
        from .leaf_builder import (
            leaf_to_petiole_rachis_branches,
            create_lateral_petiolules,
            create_terminal_petiolule,
        )
    except ImportError:
        # Standalone execution
        import sys
        import os
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        from leaf_builder import (
            leaf_to_petiole_rachis_branches,
            create_lateral_petiolules,
            create_terminal_petiolule,
        )
    
    # Build paths
    script_dir = Path(__file__).parent
    project_root = script_dir.parent.parent.parent
    csv_path = (project_root / "data" / "simulation_output" / "dynamic_output" / 
                "graphs" / f"graph_day_{day}.csv")
    output_dir = project_root / "output"
    
    # Load trunk internodes
    print(f"[INFO] Loading trunk internodes...")
    internodes = load_trunk_internodes(str(csv_path), day, plant_id)
    
    # Convert trunk to BRANCHES format
    trunk_branch = internodes_to_branch_config(internodes)
    all_branches = [trunk_branch]
    
    # Load lateral branches (order=1)
    print(f"[INFO] Loading lateral branches...")
    lateral_branches = load_lateral_branches(str(csv_path), day, plant_id)
    
    if lateral_branches:
        print(f"[INFO] Processing {len(lateral_branches)} lateral branches...")
        lateral_branch_configs = lateral_branches_to_branch_config(lateral_branches, trunk_branch["id"])
        all_branches.extend(lateral_branch_configs)
        
        # Create mapping: (rank, organ_index) → branch_id for lateral branches
        lateral_branch_map = {
            (int(b["id"].split("_r")[1].split("_")[0]), 
             int(b["id"].split("_o")[1])): b["id"]
            for b in lateral_branch_configs
        }
    else:
        lateral_branch_map = {}
    
    # Load trunk leaves (order=0)
    print(f"[INFO] Loading trunk leaves...")
    trunk_leaves = load_leaves(str(csv_path), day, plant_id, order=0)
    
    if trunk_leaves:
        print(f"[INFO] Processing {len(trunk_leaves)} trunk leaves...")
        for leaf in trunk_leaves:
            # Petiole + Rachis
            petiole_rachis = leaf_to_petiole_rachis_branches(leaf, trunk_branch["id"])
            all_branches.extend(petiole_rachis)
            
            # Check if rachis was created
            if len(petiole_rachis) > 1:
                rachis_branch = petiole_rachis[1]
                petiole_r = petiole_rachis[0]["radius"]
                
                # Lateral petiolules
                laterals = create_lateral_petiolules(leaf, rachis_branch["id"], petiole_r)
                all_branches.extend(laterals)
                
                # Terminal petiolule
                terminal = create_terminal_petiolule(
                    rachis_branch["id"], 
                    rachis_branch["n_links"], 
                    petiole_r, 
                    leaf["rank"],
                    leaf["organ_index"]
                )
                all_branches.append(terminal)
    
    # Load lateral branch leaves (order=1)
    if lateral_branch_map:
        print(f"[INFO] Loading lateral branch leaves...")
        lateral_leaves = load_leaves(str(csv_path), day, plant_id, order=1)
        
        if lateral_leaves:
            print(f"[INFO] Processing {len(lateral_leaves)} lateral branch leaves...")
            for leaf in lateral_leaves:
                # Find parent lateral branch: match by parent_rank AND organ_index
                # Leaf organ_index 0 → Branch organ_index 0
                # Leaf organ_index 1 → Branch organ_index 1
                parent_key = (leaf["parent_rank"], leaf["organ_index"])
                
                parent_branch_id = lateral_branch_map.get(parent_key)
                
                if not parent_branch_id:
                    print(f"[WARNING] Could not find parent lateral branch for leaf rank={leaf['rank']}, "
                          f"organ_index={leaf['organ_index']}, parent_rank={leaf['parent_rank']}")
                    continue
                
                # Petiole + Rachis (attach to lateral branch)
                petiole_rachis = leaf_to_petiole_rachis_branches(leaf, parent_branch_id)
                all_branches.extend(petiole_rachis)
                
                # Check if rachis was created
                if len(petiole_rachis) > 1:
                    rachis_branch = petiole_rachis[1]
                    petiole_r = petiole_rachis[0]["radius"]
                    
                    # Lateral petiolules
                    laterals = create_lateral_petiolules(leaf, rachis_branch["id"], petiole_r)
                    all_branches.extend(laterals)
                    
                    # Terminal petiolule
                    terminal = create_terminal_petiolule(
                        rachis_branch["id"], 
                        rachis_branch["n_links"], 
                        petiole_r, 
                        leaf["rank"],
                        leaf["organ_index"]
                    )
                    all_branches.append(terminal)
    
    # Calculate stats
    total_links = sum(b["n_links"] for b in all_branches)
    print(f"[INFO] Total branches: {len(all_branches)}, total links: {total_links}")
    
    # Import tree_config to check limit
    import importlib.util
    config_path = Path(__file__).parent.parent / "tree_config.py"
    spec = importlib.util.spec_from_file_location("tree_config", config_path)
    tree_config = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(tree_config)
    
    if total_links > tree_config.MAX_N_LINK:
        print(f"[WARNING] Total links ({total_links}) exceeds PhysX limit ({tree_config.MAX_N_LINK})")
        print(f"[WARNING] Consider reducing GLOBAL_SCALE or skipping some leaves")
    
    # Save to JSON
    json_path = save_branches_json(
        all_branches,
        day,
        str(output_dir),
        internodes,
        f"graph_day_{day}.csv"
    )
    
    return all_branches, json_path



# ==============================================================================
# STANDALONE TEST
# ==============================================================================

if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="CSV to BRANCHES Parser")
    parser.add_argument("--day", type=int, required=True, help="Simulation day")
    parser.add_argument("--plant-id", type=int, default=1, help="Plant ID (default: 1)")
    args = parser.parse_args()
    
    print("=" * 80)
    print("  CSV to BRANCHES Parser")
    print("=" * 80)
    
    branches, json_path = parse_csv_to_branches(args.day, args.plant_id)
    
    print(f"\nDay: {args.day}")
    print(f"Plant ID: {args.plant_id}")
    print(f"Trunk links: {branches[0]['n_links']}")
    print(f"Average radius: {branches[0]['radius']:.4f}m (pre-scale)")
    print(f"Average height: {branches[0]['height']:.4f}m (pre-scale)")
    print(f"JSON saved: {json_path}")
    print("=" * 80)
