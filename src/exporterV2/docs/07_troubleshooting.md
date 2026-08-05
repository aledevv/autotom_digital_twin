# Troubleshooting

Common issues and solutions.

---

## Import Errors

### "ModuleNotFoundError: No module named 'exporterV2'"

**Cause:** Running from wrong directory or PYTHONPATH not set.

**Solution:**
```bash
# Run from project root
cd /home/alessandro/isaacsim/autotom_digital_twin
./run_mainV2.sh --day 100
```

Or set PYTHONPATH:
```bash
export PYTHONPATH=/home/alessandro/isaacsim/autotom_digital_twin:$PYTHONPATH
```

---

### "ModuleNotFoundError: No module named 'pxr'"

**Cause:** Running outside Isaac Sim environment.

**Solution:**
```bash
# Use Isaac Sim Python
~/isaac-sim-4.2.0/python.sh your_script.py

# Or activate Isaac Sim environment
source ~/isaac-sim-4.2.0/setup_python_env.sh
```

---

## Collision Errors

### Simulation Explodes (PhysX Error)

**Symptoms:**
- Joints detach
- Links fly apart
- Console errors: "PxArticulation: solver failed"

**Cause:** Lateral branches intersecting → invalid initial state.

**Diagnosis:**
```bash
python3 src/exporterV2/tests/test_collision_geometry.py output/day_100/branches_v2_day_100.json
```

**Solutions:**

1. **Increase min separation:**
```python
# profiles/tomato_default.py
"min_angle_separation_deg": 90.0  # Was 60.0
```

2. **Reduce jitter:**
```python
"rot_jitter_deg": 30.0  # Was 45.0
```

3. **Check test output:**
Look for separations < 60° in test results.

---

### "Could not find collision-free angle"

**Symptoms:**
Warning in logs, missing lateral branches in output.

**Cause:** Too many existing branches → no space for new one.

**Solution:**
```python
# Increase max attempts
max_attempts = 200  # Was 100

# Or reduce branch count
"organ_indices": [0]  # Only first branch
```

---

## CSV Parsing Errors

### "FileNotFoundError: graph_day_X.csv"

**Cause:** CSV file missing or wrong path.

**Solution:**
```bash
# Check file exists
ls data/graph_day_100.csv

# Or specify custom path
python main.py --csv-path /path/to/custom.csv
```

---

### "KeyError: 'ccw_orientation'"

**Cause:** CSV missing required column.

**Solution:**
Profile should handle missing data:
```python
# In adapter
orientation = row.get("ccw_orientation")
if orientation is None:
    # Use phyllotaxis fallback
    orientation = (rank * 137.5) % 360
```

---

## USD Generation Errors

### "Prim already exists"

**Cause:** Duplicate branch IDs in BRANCHES config.

**Solution:**
Ensure unique IDs:
```python
branch_id = f"Branch_r{rank}_o{organ_index}"  # Unique per rank+organ
```

---

### "Invalid joint configuration"

**Cause:** Parent prim doesn't exist.

**Solution:**
Verify parent ordering in BRANCHES:
```python
# Trunk must come before its children
[
  {"id": "trunk", "parent": None},       # ✅ First
  {"id": "Branch_r1", "parent": "trunk"} # ✅ Parent exists
]
```

---

## Testing Errors

### Test Script Not Executable

**Symptoms:**
```
bash: ./test_refactoring.sh: Permission denied
```

**Solution:**
```bash
chmod +x src/exporterV2/tests/test_refactoring.sh
```

---

### Test Fails: "JSON file not found"

**Cause:** Model not generated yet.

**Solution:**
```bash
# Generate model first
./run_mainV2.sh --day 100

# Then run test
python3 src/exporterV2/tests/test_collision_geometry.py output/day_100/branches_v2_day_100.json
```

---

## Performance Issues

### "Generation takes too long"

**Typical times:**
- Day 1: ~10s
- Day 50: ~12s
- Day 100: ~15s

**If much slower:**

1. **Check CSV size:**
```bash
wc -l data/graph_day_100.csv
```

2. **Profile code:**
```bash
python -m cProfile main.py --day 100
```

3. **Reduce complexity:**
```python
# Fewer lateral branches
"organ_indices": [0]  # Was [0, 1]
```

---

## Configuration Errors

### Profile Not Found

**Symptoms:**
```
KeyError: 'lateral_branches'
```

**Solution:**
Ensure profile has all required keys:
```python
REQUIRED_KEYS = [
    "lateral_branches",
    "trunk_leaves",
    "lateral_leaves",
    "physics"
]
```

---

### Invalid Profile Values

**Symptoms:**
```
ValueError: min_angle_separation_deg must be > 0
```

**Solution:**
Validate profile:
```python
assert 0 < profile["lateral_branches"]["min_angle_separation_deg"] <= 180
assert profile["lateral_branches"]["rot_jitter_deg"] >= 0
```

---

## Getting Help

1. **Check logs:**
```bash
tail -f output/export.log
```

2. **Run tests:**
```bash
./src/exporterV2/tests/test_refactoring.sh
python3 src/exporterV2/tests/test_collision_geometry.py output/day_100/branches_v2_day_100.json
```

3. **Visual inspection:**
Open USD in Isaac Sim and check for:
- Missing branches
- Overlapping geometry
- Incorrect orientations

---

**See also:**
- [05_testing.md](05_testing.md) - How to run tests
- [06_implementation_notes.md](06_implementation_notes.md) - Common pitfalls
- [04_collision_checks.md](04_collision_checks.md) - Collision system details
