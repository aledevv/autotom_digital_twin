#!/bin/bash
# Test 2B-D — Explicit PhysX transform sync
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "========================================"
echo "  Test 2B-D — Explicit PhysX sync"
echo "========================================"
echo ""
echo "Same workload as C1:"
echo "  6 rigid links / 6 bones / 5 D6"
echo "  37 visual rings / 14 radial"
echo "  1 capsule per link = 6 capsules"
echo ""
echo "Only new operation:"
echo "  world.step"
echo "    -> PhysX update_transformations"
echo "    -> sync_skin"
echo "    -> app.update"
echo ""

"$HOME/isaacsim/python.sh" "$SCRIPT_DIR/run_explicit_physx_sync.py"
