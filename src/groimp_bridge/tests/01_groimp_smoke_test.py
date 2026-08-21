from pathlib import Path
from GroPy import GroPy

API_URL = "http://localhost:58081/api/"
PROJECT_ROOT = Path(__file__).resolve().parents[3]
PROJECT_PATH = str(PROJECT_ROOT / "model" / "project_bridge.gsz")

print("[1] Connecting...")
link = GroPy.GroLink(API_URL)

print("[2] Opening GSZ by server-side path...")
print("Project:", PROJECT_PATH)

request = link.openWB(path=PROJECT_PATH).run()

print("\n=== GROIMP RESPONSE ===")
print("Status:", request.result.status_code)
print("Body:")
print(request.result.text)
print("=======================\n")

if request.result.status_code != 200:
    raise SystemExit("PROJECT OPEN FAILED")

wb = request.read()

print("[3] Workbench opened successfully")

graph = wb.getProjectGraph().run().read()

print("[4] Graph retrieved")
print("Nodes:", len(graph.get("projectgraphNodes", [])))
print("Edges:", len(graph.get("projectgraphEdges", [])))
print("Root:", graph.get("projectgraphRoot"))

print("\n=== PROJECT SMOKE TEST PASSED ===")