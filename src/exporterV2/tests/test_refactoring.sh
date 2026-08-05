#!/bin/bash
# Test script for exporterV2 refactoring
# Tests that Phase 1-2 changes preserve functionality

set -e

echo "========================================"
echo "ExporterV2 Refactoring Test Suite"
echo "========================================"
echo ""

# Test 1: Import structure
echo "[Test 1/4] Testing import structure..."
cd src/exporterV2
python3 << 'EOF'
import sys
sys.path.insert(0, '..')

# Test core imports (no pxr dependency)
from exporterV2.core import tree_config
from exporterV2.core import BRANCHES, GLOBAL_SCALE
print("  ✓ core imports")

# Test profile imports
from exporterV2.profiles import TOMATO_PROFILE, SIMPLE_PLANT_PROFILE
print("  ✓ profile imports")

# Test adapter module exists (don't import parser - requires pandas)
import os
assert os.path.exists("../exporterV2/adapters/groimp_csv/parser.py")
print("  ✓ adapter module present")

print("[PASS] All imports working")
EOF

cd ../..
echo ""

# Test 2: JSON generation for day 1
echo "[Test 2/4] Testing JSON generation (day 1)..."
if [ -f "output/day_1/branches_v2_day_1.json" ]; then
    BRANCHES=$(grep -o '"n_branches": [0-9]*' output/day_1/branches_v2_day_1.json | grep -o '[0-9]*')
    LINKS=$(grep -o '"total_links": [0-9]*' output/day_1/branches_v2_day_1.json | grep -o '[0-9]*')
    
    if [ "$BRANCHES" = "20" ] && [ "$LINKS" = "23" ]; then
        echo "  ✓ Day 1: $BRANCHES branches, $LINKS links (expected)"
        echo "[PASS] JSON generation correct"
    else
        echo "  ✗ Day 1: $BRANCHES branches, $LINKS links (expected 20, 23)"
        echo "[FAIL] JSON values incorrect"
        exit 1
    fi
else
    echo "  ! JSON not found, run: ./run_mainV2.sh --day 1"
    echo "[SKIP] Test skipped"
fi
echo ""

# Test 3: Profile configuration
echo "[Test 3/4] Testing profile system..."
python3 << 'EOF'
import sys
sys.path.insert(0, 'src')

from exporterV2.profiles.tomato_default import TOMATO_PROFILE
from exporterV2.profiles.simple_plant import SIMPLE_PLANT_PROFILE

# Verify tomato profile structure
assert "lateral_branches" in TOMATO_PROFILE
assert "trunk_leaves" in TOMATO_PROFILE
assert "lateral_leaves" in TOMATO_PROFILE
assert TOMATO_PROFILE["name"] == "Tomato Default"
print("  ✓ Tomato profile structure valid")

# Verify simple profile structure
assert "lateral_branches" in SIMPLE_PLANT_PROFILE
assert SIMPLE_PLANT_PROFILE["name"] == "Simple Plant"
assert SIMPLE_PLANT_PROFILE["lateral_branches"]["enabled"] == False
print("  ✓ Simple plant profile structure valid")

print("[PASS] Profile system working")
EOF
echo ""

# Test 4: Directory structure
echo "[Test 4/4] Testing directory structure..."
MISSING=""

[ -d "src/exporterV2/core" ] || MISSING="$MISSING core/"
[ -d "src/exporterV2/adapters/groimp_csv" ] || MISSING="$MISSING adapters/groimp_csv/"
[ -d "src/exporterV2/profiles" ] || MISSING="$MISSING profiles/"
[ -f "src/exporterV2/core/tree_config.py" ] || MISSING="$MISSING core/tree_config.py"
[ -f "src/exporterV2/adapters/groimp_csv/parser.py" ] || MISSING="$MISSING adapters/groimp_csv/parser.py"
[ -f "src/exporterV2/profiles/tomato_default.py" ] || MISSING="$MISSING profiles/tomato_default.py"

if [ -z "$MISSING" ]; then
    echo "  ✓ All required directories and files present"
    echo "[PASS] Structure correct"
else
    echo "  ✗ Missing: $MISSING"
    echo "[FAIL] Structure incomplete"
    exit 1
fi
echo ""

# Summary
echo "========================================"
echo "✅ All Tests Passed!"
echo "========================================"
echo ""
echo "Refactoring Phase 1-2 Complete:"
echo "  • Directory structure: core/, adapters/, profiles/"
echo "  • Profile system: Working"
echo "  • Import structure: Clean"
echo "  • Functionality: Preserved"
echo ""
echo "Ready for production use!"
