import pandas as pd
from pathlib import Path
from typing import Union
from .models import OrganKey, OrganNode, InternodeNode, LeafNode, FruitsNode, RootNode, PlantSnapshot

def _parse_float_array(val_str: str) -> list[float]:
    s = str(val_str).strip()
    if s in ("0", "0.0", "", "nan", "None"):
        return []
    return [float(x) for x in s.split("_")]

def load_snapshot(csv_path: Union[str, Path], day: int, plant_id: int) -> PlantSnapshot:
    df = pd.read_csv(csv_path, skipinitialspace=True)  # manages spaces between headers
    df.columns = df.columns.str.strip()
    df = df[(df["day"] == day) & (df["plant_id"] == plant_id)].copy()
    
    # NOTE: GroIMP's exporter uses IdentityHashMap to prevent true graph-
    # traversal duplicates. We do NOT dedup here because twin branches
    # (different organ_index but identical biomass) are genuine distinct organs.

    # Strip trailing spaces on string colimns (after filtering, on less rows)
    str_cols = df.select_dtypes(include="object").columns
    df[str_cols] = df[str_cols].apply(lambda col: col.str.strip())
    

    if df.empty:
        raise ValueError(f"No data found for day {day} and plant_id {plant_id}")

    snapshot = PlantSnapshot(day=day, plant_id=plant_id)

    for _, row in df.iterrows():
        key = OrganKey(
            day=day,
            plant_id=plant_id, 
            order=int(row["order"]), 
            rank=int(row["rank"]), 
            organ_class = str(row["organ_class"]).strip(),
            organ_index=int(row["organ_index"]),
        )

        # Common fields to all organs
        base_kwargs = dict(
            key=key,
            parent_rank=int(row["parent_rank"]),
            parent_organ_class=row["parent_organ_class"],
            age_dd=float(row["age_dd"]),
            dry_biomass_mg=float(row["dry_biomass_mg"]),
            area_m2=float(row["area_m2"]),
            length=float(row["length"]),
            is_fruit=bool(row["is_fruit"]),
            is_root=bool(row["is_root"]),
        )

        organ_class = row["organ_class"]

        # Create the organ node based on the class
        if organ_class == "Internode":
            node = InternodeNode(
                **base_kwargs,
                width_m=float(row["internode_width_m"])
            )
        elif organ_class == "Leaf":
            node = LeafNode(
                **base_kwargs,
                length_petiole=float(row["leaf_length_petiole"]),
                diameter_petiole=float(row["leaf_diameter_petiole"]),
                angle_petiole=float(row["leaf_angle_petiole"]),
                ccw_orientation=float(row["leaf_ccw_orientation"]),
                curvature=float(row["leaf_curvature"]),
                blades_nr=int(row["leaf_blades_nr"]),
                area_blades_total=float(row["leaf_area_blades_total"]),
                rachis_length=float(row["leaf_rachis_length"]),
                leaf_segments_length=_parse_float_array(row["leaf_segments_length"]),
                leaf_area_m2blades=_parse_float_array(row["leaf_area_m2blades"]),
                leaf_inclination_segments=_parse_float_array(row["leaf_inclination_segments"]),
            )
            if node.area_blades_total <= 1e-9: # filter primordia
                continue
        elif organ_class == "Fruits":
            # Support both old CSV format (fruit_radius_0/1/2) and new
            # array format (fruit_radii, fruit_age_dd)
            if "fruit_radii" in row.index:
                radii = _parse_float_array(row["fruit_radii"])
                ages  = _parse_float_array(row["fruit_age_dd"])
            else:
                radii = [
                    float(row["fruit_radius_0"]),
                    float(row["fruit_radius_1"]),
                    float(row["fruit_radius_2"]),
                ]
                ages = [
                    float(row["fruit_age_dd_0"]),
                    float(row["fruit_age_dd_1"]),
                    float(row["fruit_age_dd_2"]),
                ]
            node = FruitsNode(
                **base_kwargs,
                fruit_nr=int(row["fruit_nr"]),
                fruit_radii=radii,
                fruit_age_dd=ages,
                ripening_dd=float(row["fruit_ripening_dd"]),
                truss_angle=float(row["fruit_truss_angle"]),
            )
        elif organ_class == "Root":
            node = RootNode(**base_kwargs)
        elif organ_class == "Truss":
            continue
        else:
            raise ValueError(f"Unknown organ class: {organ_class}")
        
        snapshot.organs.append(node)
        snapshot.by_key[key] = node

        # Structural index (order, rank) -> list of organs for that phyhtometer
        pos = (node.key.order, node.key.rank)
        snapshot.by_position.setdefault(pos, []).append(node)
        

    _link_hierarchy(snapshot)   # CREATES A HIERARCHICAL GRAPH of the plant!!

    return snapshot
        
            
def _link_hierarchy(snapshot: PlantSnapshot) -> None:
    for node in snapshot.organs:
        if node.parent_rank == -1:  # root node has no parent
            continue

        for search_order in ([node.key.order] if node.key.order == 0  # if main axis (order=0) look for parent in the main stem
                                              else [node.key.order, 0]):    # else search parent first in the same branch then main stem
            candidates = snapshot.by_position.get(
                (search_order, node.parent_rank), []
            )
            parent = next(                  # filter for the correct organ class and branch index
                (c for c in candidates
                 if c.key.organ_class == node.parent_organ_class and
                 (search_order != node.key.order or c.key.organ_index == node.key.organ_index)),
                None
            )
            if parent is not None:  # if parent is found, add the edge and stop searching
                node.parent = parent
                parent.children.append(node)
                break
