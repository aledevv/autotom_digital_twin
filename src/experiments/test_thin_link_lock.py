"""
test_thin_link_lock.py - Test sintetico per Thin Link Lock logic
"""
import sys
import os

# Aggiungi il root del progetto al path
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "../../"))
sys.path.insert(0, PROJECT_ROOT)

from src.exporterV2.core.tree_config import MIN_LINK_RADIUS_WORLD, GLOBAL_SCALE
from src.exporterV2.core.optimizations.techniques.thin_link_lock import ThinLinkLockTechnique

def main():
    print(f"=== Test Thin Link Lock Logic ===")
    print(f"Soglia minima (MIN_LINK_RADIUS_WORLD): {MIN_LINK_RADIUS_WORLD}m")
    print(f"Scala globale (GLOBAL_SCALE): {GLOBAL_SCALE}")
    
    # 2mm threshold
    # Branch 1 (Trunk): 10mm -> 0.01m (Safe)
    # Branch 2 (Lateral): 3mm -> 0.003m (Safe)
    # Branch 3 (Petiolule): 1mm -> 0.001m (Under threshold!)
    
    branches = [
        {
            "id": "trunk",
            "parent": None,
            "n_links": 3,
            "radius": 0.010,  # 10mm
            "joint_type": "d6"
        },
        {
            "id": "lateral",
            "parent": "trunk",
            "n_links": 2,
            "radius": 0.003,  # 3mm
            "joint_type": "d6"
        },
        {
            "id": "petiolule_thin",
            "parent": "lateral",
            "n_links": 1,
            "radius": 0.001,  # 1mm
            "joint_type": "d6"
        }
    ]
    
    tech = ThinLinkLockTechnique()
    
    print("\nPrima dell'ottimizzazione:")
    for b in branches:
        r_world = b['radius'] * GLOBAL_SCALE
        print(f" - {b['id']}: raggio={r_world:.4f}m, joint_type={b['joint_type']}")
        
    print(f"\nApplicabile? {tech.can_apply(branches)}")
    print(f"Stima riduzione (n_links): {tech.estimate_reduction(branches)}")
    
    mod_branches, report = tech.apply(branches)
    
    print("\nDopo l'ottimizzazione:")
    for b in mod_branches:
        r_world = b['radius'] * GLOBAL_SCALE
        status = "(SOTTO SOGLIA!)" if r_world < MIN_LINK_RADIUS_WORLD else "(OK)"
        print(f" - {b['id']}: raggio={r_world:.4f}m {status}, joint_type={b['joint_type']}")
        
    print(f"\nReport:")
    print(report)
    
    # Check validazione
    validation = tech.validate(branches, mod_branches)
    print(f"\nValidazione OK? {validation.valid}")
    if not validation.valid:
        print("Errori:", validation.errors)

if __name__ == "__main__":
    main()
