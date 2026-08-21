from GroPy import GroPy
from pathlib import Path

API_URL = "http://localhost:58081/api/"
PROJECT_ROOT = Path(__file__).resolve().parents[3]
PROJECT_PATH = str(PROJECT_ROOT / "model" / "project_bridge.gsz")

print("[1] Connecting...")
link = GroPy.GroLink(API_URL)

print("[2] Opening project...")
request = link.openWB(path=PROJECT_PATH).run()

if request.result.status_code != 200:
    print(request.result.text)
    raise SystemExit("PROJECT OPEN FAILED")

wb = request.read()

print("[3] Project opened")

# -------------------------------------------------------
# BEFORE
# -------------------------------------------------------

print("[4] Reading graph BEFORE step...")

graph_before = wb.getProjectGraph().run().read()

from collections import Counter

def count_types(graph):
    return Counter(
        node.get("type", "UNKNOWN")
        for node in graph.get("projectgraphNodes", [])
    )
types_before = count_types(graph_before)

nodes_before = graph_before.get("projectgraphNodes", [])
edges_before = graph_before.get("projectgraphEdges", [])

print(f"BEFORE: nodes={len(nodes_before)}, edges={len(edges_before)}")

# -------------------------------------------------------
# ONE GROIMP STEP
# -------------------------------------------------------

print("[5] Running Dynamic_Model ONCE...")

run_request = wb.runRGGFunction("Dynamic_Model").run()

print("\n=== RGG RESPONSE ===")
print("Status:", run_request.result.status_code)
print("Body:")
print(run_request.result.text)
print("====================\n")

if run_request.result.status_code != 200:
    wb.close().run()
    raise SystemExit("RGG STEP FAILED")

# -------------------------------------------------------
# AFTER
# -------------------------------------------------------

print("[6] Reading graph AFTER step...")

graph_after = wb.getProjectGraph().run().read()

types_after = count_types(graph_after)

nodes_after = graph_after.get("projectgraphNodes", [])
edges_after = graph_after.get("projectgraphEdges", [])

print(f"AFTER: nodes={len(nodes_after)}, edges={len(edges_after)}")

print()
print("=== DIFFERENCE ===")
print(f"Nodes: {len(nodes_before)} -> {len(nodes_after)} "
      f"({len(nodes_after) - len(nodes_before):+d})")

print(f"Edges: {len(edges_before)} -> {len(edges_after)} "
      f"({len(edges_after) - len(edges_before):+d})")

print("\n=== NODE TYPES ===")

all_types = sorted(set(types_before) | set(types_after))

for node_type in all_types:
    before = types_before.get(node_type, 0)
    after = types_after.get(node_type, 0)
    diff = after - before

    print(
        f"{node_type:60} "
        f"{before:4} -> {after:4} "
        f"({diff:+d})"
    )

# -------------------------------------------------------
# CLOSE WITHOUT SAVE
# -------------------------------------------------------

print("[7] Closing workbench WITHOUT saving...")
wb.close().run()

print()
print("=== TEST 02B PASSED ===")