from pathlib import Path

from GroPy import GroPy


API_URL = "http://localhost:58081/api/"

PROJECT_ROOT = Path(__file__).resolve().parents[3]
PROJECT_PATH = str(PROJECT_ROOT / "model" / "project_bridge.gsz")


def query_numeric_attributes(wb, node_type, fields):
    """
    Query numeric attributes from all GroIMP nodes of a given type.

    Returns:
        {
            node_id: {
                "field1": value,
                "field2": value,
                ...
            }
        }
    """

    # Example generated expression:
    #
    # n.getId() + ";" + n.length
    #
    print_expression = ' + ";" + '.join(
        ["n.getId()"] + [f"n.{field}" for field in fields]
    )

    query = f"""
    for ({node_type} n : (* {node_type} *))
    {{
        println({print_expression});
    }}
    """

    request = wb.runXLQuery(query).run()

    if request.result.status_code != 200:
        print(f"\n[ERROR] XL query failed for {node_type}")
        print(request.result.text)
        return {}

    response = request.read()

    values = {}

    for line in response.get("console", []):
        parts = str(line).strip().split(";")

        # node id + requested attributes
        if len(parts) != len(fields) + 1:
            continue

        try:
            node_id = int(float(parts[0]))

            values[node_id] = {
                field: float(value)
                for field, value in zip(fields, parts[1:])
            }

        except ValueError:
            # Ignore unrelated console output
            continue

    return values


def print_attributes(title, data):
    print(f"\n=== {title} ===")

    if not data:
        print("No nodes found.")
        return

    for node_id, attributes in sorted(data.items()):
        formatted = ", ".join(
            f"{name}={value:.6f}"
            for name, value in attributes.items()
        )

        print(f"id={node_id:<5} {formatted}")

    print(f"Total: {len(data)}")


# ---------------------------------------------------------------------
# CONNECT
# ---------------------------------------------------------------------

print("[1] Connecting to GroIMP...")
link = GroPy.GroLink(API_URL)

print("[2] Opening project...")
request = link.openWB(path=PROJECT_PATH).run()

if request.result.status_code != 200:
    print(request.result.text)
    raise SystemExit("PROJECT OPEN FAILED")

wb = request.read()

print("[3] Project opened successfully")


try:

    # -----------------------------------------------------------------
    # GENERATE FIRST PLANT STATE
    # -----------------------------------------------------------------

    print("[4] Running Dynamic_Model ONCE...")

    run_request = wb.runRGGFunction("Dynamic_Model").run()

    if run_request.result.status_code != 200:
        print(run_request.result.text)
        raise RuntimeError("Dynamic_Model failed")

    run_response = run_request.read()

    print("Dynamic_Model completed.")

    # Optional useful GroIMP output
    for line in run_response.get("console", []):
        print(f"  GroIMP: {line}")

    # -----------------------------------------------------------------
    # INTERNODES
    # -----------------------------------------------------------------

    print("\n[5] Querying Internode.length...")

    internodes = query_numeric_attributes(
        wb,
        "organs.Internode",
        ["length"],
    )

    # -----------------------------------------------------------------
    # LEAVES
    # -----------------------------------------------------------------

    print("[6] Querying Leaf.length...")

    leaves = query_numeric_attributes(
        wb,
        "organs.Leaf",
        ["length"],
    )

    # -----------------------------------------------------------------
    # RESULTS
    # -----------------------------------------------------------------

    print_attributes(
        "INTERNODES",
        internodes,
    )

    print_attributes(
        "LEAVES",
        leaves,
    )

    print("\n=== SUMMARY ===")
    print(f"Internodes extracted : {len(internodes)}")
    print(f"Leaves extracted     : {len(leaves)}")

    print("\n=== TEST 03A PASSED ===")


finally:

    # -----------------------------------------------------------------
    # ALWAYS CLOSE WITHOUT SAVING
    # -----------------------------------------------------------------

    print("\n[7] Closing workbench WITHOUT saving...")

    try:
        wb.close().run()
    except Exception as exc:
        print(f"Warning while closing workbench: {exc}")