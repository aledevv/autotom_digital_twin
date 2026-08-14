#!/usr/bin/env python3
"""
Test with 10× scale to verify if jitter is due to small numbers.

Simply generates baseline config USD with 10× scale instead of 2×.
"""
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).parent.resolve()

# Just re-use test_scalability infrastructure
sys.path.insert(0, str(SCRIPT_DIR))
from test_scalability import generate_baseline_tomato, test_config_geometry


def main():
    print("="*80)
    print("SCALE TEST: 10× instead of 2×")
    print("="*80)
    print()
    print("Current scale (2×):")
    print("  Stem radius: 4mm → 8mm world (0.008m)")
    print("  Petiolule radius: 1.5mm → 3mm world (0.003m)")
    print()
    print("New scale (10×):")
    print("  Stem radius: 4mm → 40mm world (0.04m)")
    print("  Petiolule radius: 1.5mm → 15mm world (0.015m)")
    print()
    print("PhysX works best in 0.01-10m range → 10× should reduce jitter")
    print()
    
    # Generate baseline
    branches = generate_baseline_tomato()
    
    # Test and save USD with 10× scale
    print("Generating USD with 10× scale...")
    
    # Monkey-patch the scale factor temporarily
    import generate_recursive_tree_usda as gen_module
    original_scale = gen_module.GLOBAL_SCALE
    gen_module.GLOBAL_SCALE = 10.0
    
    try:
        passed, max_error, details = test_config_geometry(
            "baseline_x10_scale", 
            branches, 
            "TEST", 
            save_usd=True
        )
        
        print()
        if passed:
            print("✅ USD generated successfully")
            output_file = SCRIPT_DIR / "scalability_usds" / "baseline_x10_scale.usda"
            print(f"📁 File: {output_file}")
        else:
            print("❌ USD generation failed")
            return
    
    finally:
        # Restore original scale
        gen_module.GLOBAL_SCALE = original_scale
    
    print()
    print("="*80)
    print("TEST INSTRUCTIONS")
    print("="*80)
    print()
    print("1. Run manual test:")
    print("   python3 src/experiments/recursive_tree/tests/test_manual_cli.py")
    print()
    print("2. When prompted, test 'baseline_x10_scale'")
    print()
    print("3. Compare with 'baseline_tomato_realistic' (2× scale)")
    print()
    print("Expected:")
    print("  ✅ If 10× is stable → problem was small numbers")
    print("  ❌ If 10× still jitters → need solver tuning")
    print()


if __name__ == "__main__":
    main()
    
    print()
    print("="*80)
    print("TEST INSTRUCTIONS")
    print("="*80)
    print()
    print("1. Run manual test:")
    print("   python3 src/experiments/recursive_tree/tests/test_manual_cli.py")
    print()
    print("2. When prompted, test 'baseline_x10_scale'")
    print()
    print("3. Compare with 'baseline_tomato_realistic' (2× scale)")
    print()
    print("Expected:")
    print("  ✅ If 10× is stable → problem was small numbers")
    print("  ❌ If 10× still jitters → need solver tuning")
    print()


if __name__ == "__main__":
    main()
