# ExporterV2 Usage Guide

## Quick Start

### 1. Run with default configuration

```bash
./run_mainV2.sh
```

This will:
- Generate tree USD from `src/exporterV2/tree_config.py` BRANCHES configuration
- Apply PhysX settings
- Load in Isaac Sim for interactive simulation

### 2. Customize tree configuration

Edit `src/exporterV2/tree_config.py` to modify:

```python
# Global settings
GLOBAL_SCALE = 2.0      # Scale factor for all dimensions
BEND_LIMIT_DEG = 30.0   # Joint angle limits

# Branch configuration
BRANCHES = [
    {
        "id": "trunk",
        "parent": None,
        "attach_link": None,
        "n_links": 5,
        "radius": 0.10,
        "height": 0.20,
        "tilt": 0.0,
        "rot": 0.0,
    },
    # Add more branches...
]
```

Then run `./run_mainV2.sh` to see your changes in Isaac Sim.

### 3. Generate USD only (no Isaac Sim)

```bash
uv run python src/exporterV2/generate_tree.py
```

Output: `data/usd_models/tree_v2.usda`

### 4. Verify physics parameters

```bash
cd src/exporterV2
python tree_config.py
```

This prints spring constants, damping coefficients, and natural periods for all branches.

## Configuration Reference

See `src/exporterV2/README.md` for detailed BRANCHES configuration format.

## Troubleshooting

**Problem:** Script fails with "module not found"  
**Solution:** Make sure you're running from project root and using the correct Python environment

**Problem:** Physics looks wrong  
**Solution:** Check GLOBAL_SCALE (bigger = more stable) and physics parameters in tree_config.py

**Problem:** Tree doesn't match visualization  
**Solution:** Run `python tree_config.py` to verify configuration before launching Isaac Sim
