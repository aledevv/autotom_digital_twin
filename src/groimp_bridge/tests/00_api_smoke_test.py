from GroPy import GroPy

API_URL = "http://localhost:58081/api/"

print("[1] Connecting...")
link = GroPy.GroLink(API_URL)

print("[2] Creating empty workbench...")

request = link.createWB("newRGG").run()

print("HTTP status:", request.result.status_code)
print("Response:", request.result.text)

if request.result.status_code != 200:
    raise RuntimeError(
        f"GroIMP returned HTTP {request.result.status_code}: "
        f"{request.result.text}"
    )

data = request.result.json()

if "id" not in data:
    raise RuntimeError(
        f"GroIMP did not return a workbench id: {data}"
    )

wb = request.read()

print("[3] Workbench created")
print("Workbench response:", data)

print("[4] Reading graph...")

graph = wb.getProjectGraph().run().read()

print("Nodes:", len(graph.get("projectgraphNodes", [])))
print("Edges:", len(graph.get("projectgraphEdges", [])))
print("Root:", graph.get("projectgraphRoot"))

print("\n=== API SMOKE TEST PASSED ===")