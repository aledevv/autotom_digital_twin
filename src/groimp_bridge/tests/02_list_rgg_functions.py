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

print("[3] Project opened successfully")

print("[4] Listing RGG functions...")

result = wb.listRGGFunctions().run().read()

print("\n=== AVAILABLE RGG FUNCTIONS ===")
print(result)
print("===============================\n")

print("=== TEST 02A PASSED ===")