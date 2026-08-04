import networkx as nx
from pyvis.network import Network
from pathlib import Path
from .models import PlantSnapshot, InternodeNode, LeafNode, FruitsNode

# Color for each organ type
_COLORS = {
    "Internode": "#4caf50",  # green
    "Leaf":      "#cddc39",  # lime
    "Fruits":    "#f44336",  # red
    "Root":      "#795548",  # brown
}

def visualize_snapshot(snapshot: PlantSnapshot, output_path: str | Path = "plant_debug.html") -> None:
    """
    Generates an interactive HTML graph of the PlantSnapshot.
    """
    G = nx.DiGraph()  # directed graph: parent → child edges

    # Add nodes
    for node in snapshot.organs:
        label = f"{node.key.organ_class}\nr{node.key.rank} o{node.key.order}"
        color = _COLORS.get(node.key.organ_class, "#999999")
        G.add_node(id(node), label=label, color=color, title=_node_tooltip(node))

    # Add edges
    for node in snapshot.organs:
        if node.parent is not None:
            G.add_edge(id(node.parent), id(node))

    # Root-to-base_internode dummy edge for visual layout
    roots = [n for n in snapshot.organs if n.key.organ_class == "Root"]
    base_internodes = [
        n for n in snapshot.organs
        if n.key.organ_class == "Internode" and n.key.rank == 0 and n.key.order == 0
    ]

    if roots and base_internodes:
        G.add_edge(id(roots[0]), id(base_internodes[0]))

    # Pyvis: converts networkx → interactive HTML
    net = Network(height="750px", width="100%", directed=True)
    net.from_nx(G)
    net.set_options("""
    {
      "layout": { "hierarchical": { "enabled": true, "direction": "UD", "sortMethod": "directed" } },
      "physics": { "hierarchicalRepulsion": { "nodeDistance": 120 } }
    }
    """)
    # Create parent directory if it doesn't exist
    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    net.write_html(str(path))
    print(f"Graph saved to: {path.resolve()}")


def _node_tooltip(node) -> str:
    """Tooltip text displayed on mouseover on the node."""
    lines = [
        f"Class: {node.key.organ_class}",
        f"Rank: {node.key.rank}  Order: {node.key.order}",
        f"Age (dd): {node.age_dd:.1f}",
        f"Biomass (mg): {node.dry_biomass_mg:.2f}",
        f"Length: {node.length:.4f} m",
    ]
    if isinstance(node, FruitsNode):
        lines.append(f"Fruits: {node.fruit_nr}")
    if isinstance(node, LeafNode):
        lines.append(f"Blades: {node.blades_nr}")
    return "\n".join(lines)