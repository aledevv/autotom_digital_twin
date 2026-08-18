#!/bin/bash
# Test 2D-B — exporterV2 physics with flexible main stem
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "========================================"
echo "  Test 2D-B — V2 physics"
echo "========================================"
echo ""
echo "  main stem FLEXIBLE"
echo "  E = 70 MPa"
echo "  damping ratio = 0.8"
echo "  density = 1000 kg/m^3"
echo "  bend limit = +/-30 deg"
echo "  physics = 480 Hz"
echo "  solver = TGS 32 / 4"
echo "  gravity = 9.81 from frame 0"
echo "  NO gravity ramp"
echo ""
echo "Interaction:"
echo "  SHIFT + LEFT CLICK + drag"
echo ""

"$HOME/isaacsim/python.sh" "$SCRIPT_DIR/run_manual_mouse_grab_v2physics.py"
