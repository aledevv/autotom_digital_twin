import json
from pathlib import Path
from plant_model.models import (
    PlantSnapshot, OrganNode, InternodeNode, RootNode, LeafNode, FruitsNode
)

def export_graph_json(snapshot: PlantSnapshot, output_path: str | Path) -> None:
    nodes = []
    for node in snapshot.organs:
        k = node.key
        entry = {
            "id": f"{k.organ_class}_r{k.rank}_o{k.order}_i{k.organ_index}",
            "organ_class": k.organ_class,
            "rank": k.rank,
            "order": k.order,
            "organ_index": k.organ_index,
            "parent_rank": node.parent_rank,
            "parent_organ_class": node.parent_organ_class,
            "parent_id": (
                f"{node.parent.key.organ_class}_r{node.parent.key.rank}"
                f"_o{node.parent.key.order}_i{node.parent.key.organ_index}"
                if node.parent else None
            ),
            "children_ids": [
                f"{c.key.organ_class}_r{c.key.rank}_o{c.key.order}_i{c.key.organ_index}"
                for c in node.children
            ],
            "age_dd": node.age_dd,
            "length": node.length,
        }

        # Campi specifici per tipo
        if isinstance(node, InternodeNode):
            entry["width_m"] = node.width_m
        elif isinstance(node, LeafNode):
            entry["length_petiole"] = node.length_petiole
            entry["angle_petiole"] = node.angle_petiole
            entry["ccw_orientation"] = node.ccw_orientation
            entry["blades_nr"] = node.blades_nr
            entry["area_blades_total"] = node.area_blades_total
        elif isinstance(node, FruitsNode):
            entry["fruit_nr"] = node.fruit_nr
            entry["fruit_radii"] = node.fruit_radii
            entry["truss_angle"] = node.truss_angle

        nodes.append(entry)

    output = {
        "day": snapshot.day,
        "plant_id": snapshot.plant_id,
        "organ_count": len(nodes),
        "organs": nodes,
    }

    Path(output_path).write_text(json.dumps(output, indent=2))
    print(f"[JSON] Saved → {output_path}")