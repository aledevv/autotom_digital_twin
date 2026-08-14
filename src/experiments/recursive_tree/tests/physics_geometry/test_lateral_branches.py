#!/usr/bin/env python3
"""
Test Lateral Branches - Antenna "Restrelliera" Style

Questo script genera TRE configurazioni di test con rametti laterali:

TEST 1: SINGLE BRANCH ANTENNA
- Trunk verticale (tronco principale)
- 1 Branch principale inclinato che si stacca dal trunk
- 8 Subbranches laterali distribuiti ai lati del branch (stile antenna)
- Rametti inclinati 35°
- Total: 27 links

TEST 2: MULTI-BRANCH HORIZONTAL
- Trunk verticale (7 links)
- 3 Main branches ORIZZONTALI (paralleli al terreno, tilt=90°)
- Ogni branch ha 6 rametti laterali ORIZZONTALI intersecati
- Pattern intersecato: lati alternati (90°/270°)
- Total: 64 links (esattamente al limite PhysX!)

TEST 3: COMPLEX MULTI-BRANCH HEAVY
- Trunk verticale più alto (10 links)
- 4 Main branches ORIZZONTALI (distribuzione radiale completa)
- Ogni branch ha 7 rametti laterali ORIZZONTALI intersecati
- Rami più lunghi (8 links) e più robusti (radius 12mm)
- Total: 98 links (heavy test, oltre il limite standard!)

Il Test 3 usa skip_limit_check=True per superare il limite di 64 links.

Usage:
    cd ~/isaacsim/autotom_digital_twin
    uv run src/experiments/recursive_tree/tests/test_lateral_branches.py

Dopo aver generato i USD, testali con:
    python3 src/experiments/recursive_tree/tests/test_manual_cli.py
"""
import os
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).parent.resolve()
sys.path.insert(0, str(SCRIPT_DIR.parent))

from tree_config import validate_branches, scaled, calculate_physics_params, compute_mass
from generate_recursive_tree_usda import build_stage
from pxr import Usd


# ==============================================================================
# CONFIGURAZIONE ANTENNA A RESTRELLIERA
# ==============================================================================

def generate_lateral_branches_config():
    """
    Genera configurazione con rami laterali stile antenna a restrelliera.
    
    Struttura:
    - Trunk: 5 links verticali (tronco principale)
    - Main branch: 6 links inclinati che si staccano dal trunk
    - Lateral subbranches: 8 rametti che si staccano dai lati del main branch
      (4 per lato, distribuiti lungo il main branch)
    
    Pattern laterale:
        rot=0°, 180° (destra/sinistra)
        Alternati lungo il main branch ai link 2, 3, 4, 5
    
    Total: 5 + 6 + (8 × 2) = 27 links
    """
    branches = []
    
    # ========== TRUNK (tronco principale) ==========
    branches.append({
        "id": "trunk",
        "parent": None,
        "attach_link": None,
        "n_links": 5,
        "radius": 0.010,   # 10mm → 100mm world
        "height": 0.025,   # 25mm → 250mm world
        "tilt": 0.0,
        "rot": 0.0,
    })
    
    # ========== MAIN BRANCH (ramo principale) ==========
    # Si attacca al trunk al link 3, inclinato 50°
    branches.append({
        "id": "main_branch",
        "parent": "trunk",
        "attach_link": 3,
        "n_links": 6,
        "radius": 0.004,   # 4mm → 40mm world
        "height": 0.020,   # 20mm → 200mm world
        "tilt": 50.0,
        "rot": 45.0,
    })
    
    # ========== LATERAL SUBBRANCHES (rametti laterali) ==========
    # Pattern antenna: rametti ai lati del main_branch
    # Distribuzione:
    #   Link 2: 2 rametti (rot 0°, 180°)
    #   Link 3: 2 rametti (rot 0°, 180°)
    #   Link 4: 2 rametti (rot 0°, 180°)
    #   Link 5: 2 rametti (rot 0°, 180°)
    
    lateral_config = [
        # (attach_link, rot_angle, sub_id)
        (2, 0.0, "sub_L2_right"),
        (2, 180.0, "sub_L2_left"),
        (3, 0.0, "sub_L3_right"),
        (3, 180.0, "sub_L3_left"),
        (4, 0.0, "sub_L4_right"),
        (4, 180.0, "sub_L4_left"),
        (5, 0.0, "sub_L5_right"),
        (5, 180.0, "sub_L5_left"),
    ]
    
    for attach_link, rot, sub_id in lateral_config:
        branches.append({
            "id": sub_id,
            "parent": "main_branch",
            "attach_link": attach_link,
            "n_links": 2,
            "radius": 0.002,   # 2mm → 20mm world
            "height": 0.015,   # 15mm → 150mm world
            "tilt": 35.0,      # Inclinazione rispetto al main_branch
            "rot": rot,        # 0° (destra) o 180° (sinistra)
        })
    
    return branches


def generate_multi_branch_horizontal_config():
    """
    Genera configurazione con 4 rami laterali orizzontali (paralleli al terreno).
    
    Struttura:
    - Trunk: 10 links verticali
    - 4 Main branches: rami orizzontali (tilt=90°) distribuiti lungo il trunk
    - Per ogni main branch: 6 rametti laterali ORIZZONTALI intersecati
    
    Pattern laterale INTERSECATO:
        I rametti si espandono orizzontalmente ruotati di 90° attorno all'asse del ramo
        tilt=0° (seguono direzione del parent branch)
        rot=90° e rot=270° ALTERNATI (intersecati tra un link e l'altro)
        
        Link 2: side1 (90°)
        Link 3: side2 (270°) ← INTERSECATO (lato opposto)
        Link 4: side1 (90°)  ← INTERSECATO
        Link 5: side2 (270°) ← INTERSECATO
        Link 6: side1 (90°)  ← INTERSECATO
        Link 7: side2 (270°) ← INTERSECATO
    
    Dimensioni CORRETTE:
    - Trunk: 12mm radius (120mm world)
    - Branches: 10mm radius (1cm → 100mm world) ✓
    - Branches n_links: 7 (per ospitare 6 rametti uniformemente)
    - Subbranches: 1mm radius (0.001m → 10mm world) ✓ corretto per GLOBAL_SCALE=10
    - Subbranches height: 12mm → 120mm world
    
    Total: 10 + (4 × 7) + (4 × 6 × 2) = 10 + 28 + 48 = 86 links
    NOTA: Supera limite 64! Dobbiamo ridurre.
    
    Soluzione: 3 main branches invece di 4
    Total: 10 + (3 × 7) + (3 × 6 × 2) = 10 + 21 + 36 = 67 links
    Ancora sopra!
    
    Soluzione 2: branches con 7 links, ma solo 6 rametti (invece di 7)
    Total: 10 + (3 × 7) + (3 × 6 × 2) = 10 + 21 + 36 = 67 links
    Ancora sopra!
    
    Soluzione 3: Trunk con 8 links + 3 branches × 7 links + 6 rametti × 2 links
    Total: 8 + 21 + 36 = 65 links - ANCORA SOPRA!
    
    Soluzione 4: Trunk 8 links + 3 branches × 6 links + 5 rametti × 2 links
    Total: 8 + 18 + 30 = 56 links ✓
    
    Ma vogliamo 4 rami e 6 rametti ciascuno!
    
    Soluzione FINALE: Trunk 8 links + 4 branches × 4 links + 6 rametti × 2 links
    NO - vogliamo 7 links per branch per distribuire 6 rametti uniformemente
    
    OK: Trunk 8 links + 3 branches × 7 links + 6 rametti × 2 links
    Total: 8 + 21 + 36 = 65 links (1 sopra limite)
    
    FINALE: Trunk 7 links + 3 branches × 7 links + 6 rametti × 2 links  
    Total: 7 + 21 + 36 = 64 links ✓✓✓ ESATTO!
    """
    branches = []
    
    # ========== TRUNK (tronco principale) ==========
    branches.append({
        "id": "trunk",
        "parent": None,
        "attach_link": None,
        "n_links": 7,
        "radius": 0.012,   # 12mm → 120mm world
        "height": 0.020,   # 20mm → 200mm world
        "tilt": 0.0,
        "rot": 0.0,
    })
    
    # ========== 3 MAIN BRANCHES (rami principali orizzontali) ==========
    # Si attaccano al trunk ai link 3, 5, 7
    # tilt=90° per essere orizzontali (paralleli al terreno)
    # rot distribuito per pattern simmetrico: 0°, 120°, 240°
    
    main_branches_config = [
        # (branch_id, attach_link, rot_angle)
        ("branch_1", 3, 0.0),
        ("branch_2", 5, 120.0),
        ("branch_3", 7, 240.0),
    ]
    
    for branch_id, attach_link, rot in main_branches_config:
        # Main branch orizzontale - 7 links per ospitare 7 rametti
        branches.append({
            "id": branch_id,
            "parent": "trunk",
            "attach_link": attach_link,
            "n_links": 7,
            "radius": 0.002,   # 10mm → 100mm world (1cm!)
            "height": 0.020,   # 20mm → 200mm world
            "tilt": 90.0,      # ORIZZONTALE (parallelo al terreno)
            "rot": rot,
        })
        
        # ========== LATERAL SUBBRANCHES per questo branch ==========
        # 7 rametti distribuiti uniformemente sui link 2-8 (non sul link 1 = attacco trunk)
        # PATTERN INTERSECATO: alternare lati tra un link e l'altro
        #   Link 2: side1 (90°)
        #   Link 3: side2 (270°) - lato opposto
        #   Link 4: side1 (90°)
        #   Link 5: side2 (270°)
        #   Link 6: side1 (90°)
        #   Link 7: side2 (270°)
        # 
        # CHIAVE: tilt=0° (seguono parent) + rot=90°/270° (ruotati attorno asse parent)
        # Questo fa sì che i rametti si espandano perpendicolarmente al ramo principale,
        # mantenendosi paralleli al terreno, intersecati tra i segmenti
        
        lateral_config = [
            # (attach_link, rot_angle, sub_id_suffix)
            (2, 90.0, "L2_side1"),     # lato 1
            (3, 270.0, "L3_side2"),    # lato 2 (opposto) - INTERSECATO
            (4, 90.0, "L4_side1"),     # lato 1 - INTERSECATO
            (5, 270.0, "L5_side2"),    # lato 2 (opposto) - INTERSECATO
            (6, 90.0, "L6_side1"),     # lato 1 - INTERSECATO
            (7, 270.0, "L7_side2"),    # lato 2 (opposto) - INTERSECATO
        ]
        
        for attach, sub_rot, suffix in lateral_config:
            branches.append({
                "id": f"{branch_id}_sub_{suffix}",
                "parent": branch_id,
                "attach_link": attach,
                "n_links": 2,
                "radius": 0.001,   # 1mm → 10mm world (corretto!)
                "height": 0.012,   # 12mm → 120mm world
                "tilt": 20.0,       # Segue direzione parent (orizzontale)
                "rot": sub_rot,    # Ruotato 90°/270° attorno asse parent
            })
    
    return branches


def generate_complex_multi_branch_config():
    """
    Genera configurazione COMPLESSA con molti rami e sottorametti (~100 links).
    
    Struttura:
    - Trunk: 12 links verticali (molto alto)
    - 5 Main branches: rami orizzontali (tilt=90°) distribuiti lungo il trunk
    - Per ogni main branch: 8-10 rametti laterali ORIZZONTALI intersecati
    - PLUS: 2 branches più lunghi (10 links) con ancora più rametti
    
    Pattern laterale INTERSECATO:
        rot=90° e rot=270° ALTERNATI lungo il ramo
    
    Dimensioni:
    - Trunk: 15mm radius (150mm world) - più robusto
    - Main branches: 12mm radius (120mm world) - robusti
    - Main branches corti: 8 links
    - Main branches lunghi: 10 links
    - Subbranches: 1mm radius (10mm world)
    - Subbranches height: 12mm → 120mm world
    
    Calcolo links:
    - Trunk: 12 links
    - 3 Branches corti: 3 × 8 = 24 links
    - 2 Branches lunghi: 2 × 10 = 20 links
    - Rametti branches corti (7 ciascuno): 3 × 7 × 2 = 42 links
    - Rametti branches lunghi (9 ciascuno): 2 × 9 × 2 = 36 links
    
    Total: 12 + 24 + 20 + 42 + 36 = 134 links
    TROPPO! Dobbiamo usare skip_limit_check=True
    
    Alternativa per restare sotto 100:
    - Trunk: 10 links
    - 4 Branches: 4 × 8 = 32 links
    - Rametti (7 per branch): 4 × 7 × 2 = 56 links
    Total: 10 + 32 + 56 = 98 links ✓
    """
    branches = []
    
    # ========== TRUNK (tronco principale ALTO) ==========
    branches.append({
        "id": "trunk",
        "parent": None,
        "attach_link": None,
        "n_links": 10,
        "radius": 0.010,   # 10mm → 100mm world (più robusto!)
        "height": 0.025,   # 25mm → 250mm world (più alto)
        "tilt": 0.0,
        "rot": 0.0,
    })
    
    # ========== 4 MAIN BRANCHES (rami principali orizzontali) ==========
    # Distribuiti lungo il trunk ai link 3, 5, 7, 9
    # Orientamento radiale: 0°, 90°, 180°, 270°
    
    main_branches_config = [
        # (branch_id, attach_link, rot_angle)
        ("branch_1", 3, 0.0),
        ("branch_2", 5, 90.0),
        ("branch_3", 7, 180.0),
        ("branch_4", 9, 270.0),
    ]
    
    for branch_id, attach_link, rot in main_branches_config:
        # Main branch orizzontale - 8 links per ospitare 7 rametti
        branches.append({
            "id": branch_id,
            "parent": "trunk",
            "attach_link": attach_link,
            "n_links": 8,
            "radius": 0.004,   # 6mm → 60mm world (più robusto)
            "height": 0.022,   # 22mm → 220mm world (più lungo)
            "tilt": 90.0,      # ORIZZONTALE
            "rot": rot,
        })
        
        # ========== LATERAL SUBBRANCHES per questo branch ==========
        # 7 rametti distribuiti uniformemente sui link 2-8
        # Pattern INTERSECATO: lati alternati (90°/270°)
        
        lateral_config = [
            # (attach_link, rot_angle, sub_id_suffix)
            (2, 90.0, "L2_side1"),
            (3, 270.0, "L3_side2"),    # INTERSECATO
            (4, 90.0, "L4_side1"),
            (5, 270.0, "L5_side2"),    # INTERSECATO
            (6, 90.0, "L6_side1"),
            (7, 270.0, "L7_side2"),    # INTERSECATO
            (8, 90.0, "L8_side1"),
        ]
        
        for attach, sub_rot, suffix in lateral_config:
            branches.append({
                "id": f"{branch_id}_sub_{suffix}",
                "parent": branch_id,
                "attach_link": attach,
                "n_links": 2,
                "radius": 0.001,   # 1mm → 10mm world
                "height": 0.012,   # 12mm → 120mm world
                "tilt": 0.0,       # Segue parent (orizzontale)
                "rot": sub_rot,    # Intersecato 90°/270°
            })
    
    # ========== TRUSS VERTICALE (stile grappolo pomodoro) ==========
    # UNA SOLA truss verticale molto sottile che parte dall'ULTIMO link del trunk,
    # con rametti laterali piccolissimi ai lati (stile rastrelliera)
    #
    # Struttura:
    #   Trunk (10 links)
    #     |
    #     └─ Link 10 (TOP) → truss_main (verticale, 6 links, radius 1mm)
    #                           ├─ Link 2: ─|─ micro_sub (sinistra e destra, radius 0.5mm)
    #                           ├─ Link 3: ─|─ micro_sub
    #                           ├─ Link 4: ─|─ micro_sub
    #                           └─ Link 5: ─|─ micro_sub
    #
    # Total truss: 6 + (8 × 2) = 22 links
    # Grand total: 98 + 22 = 120 links!
    
    # Truss main (rametto verticale sottile CHE PARTE DALL'ALTO DEL TRUNK)
    branches.append({
        "id": "truss_main",
        "parent": "trunk",
        "attach_link": 10,     # Si attacca all'ULTIMO link del trunk (TOP!)
        "n_links": 6,
        "radius": 0.001,       # 1mm → 10mm world (sottile!)
        "height": 0.015,       # 15mm → 150mm world
        "tilt": 90.0,           # VERTICALE (segue trunk, sale in alto)
        "rot": 0.0,
        "roll": 90.0
    })
    
    # Micro rametti laterali sul truss (stile grappolo)
    # Pattern: alternato sinistra/destra
    truss_lateral_config = [
        # (attach_link, rot_angle, sub_id_suffix)
        (2, 0.0, "micro_L2_right"),
        (2, 180.0, "micro_L2_left"),
        (3, 0.0, "micro_L3_right"),
        (3, 180.0, "micro_L3_left"),
        (4, 0.0, "micro_L4_right"),
        (4, 180.0, "micro_L4_left"),
        (5, 0.0, "micro_L5_right"),
        (5, 180.0, "micro_L5_left"),
    ]
    
    for attach, sub_rot, suffix in truss_lateral_config:
        branches.append({
            "id": f"truss_{suffix}",
            "parent": "truss_main",
            "attach_link": attach,
            "n_links": 2,
            "radius": 0.0005,  # 0.5mm → 5mm world (MOLTO SOTTILE!)
            "height": 0.008,   # 8mm → 80mm world (corti)
            "tilt": 90.0,      # Perpendolare al truss (si espande orizzontalmente)
            "rot": sub_rot,    # 0° (destra) / 180° (sinistra)
        })
    
    return branches


# ==============================================================================
# TEST FUNCTIONS
# ==============================================================================

def validate_config(branches, skip_limit_check=False):
    """Valida la configurazione e stampa info."""
    print("="*80)
    print(" " * 25 + "VALIDAZIONE CONFIGURAZIONE")
    print("="*80)
    print()
    
    try:
        validate_branches(branches, skip_limit_check=skip_limit_check)
        print("✅ Configurazione valida!")
    except ValueError as e:
        print(f"❌ Errore di validazione: {e}")
        return False
    
    # Statistiche
    total_links = sum(b["n_links"] for b in branches)
    trunk = [b for b in branches if b.get("parent") is None][0]
    main_branches = [b for b in branches if b.get("parent") == "trunk"]
    subbranches = [b for b in branches if b.get("parent") == "main_branch"]
    
    print()
    print(f"Struttura:")
    print(f"  - Trunk: {trunk['n_links']} links")
    print(f"  - Main branches: {len(main_branches)} ({sum(b['n_links'] for b in main_branches)} links)")
    print(f"  - Lateral subbranches: {len(subbranches)} ({sum(b['n_links'] for b in subbranches)} links)")
    print(f"  - Total links: {total_links}")
    print()
    
    # Dettagli lateral subbranches
    print("Distribuzione lateral subbranches (antenna):")
    by_attach = {}
    for b in subbranches:
        attach = b["attach_link"]
        if attach not in by_attach:
            by_attach[attach] = []
        by_attach[attach].append(b["id"])
    
    for attach_link in sorted(by_attach.keys()):
        subs = by_attach[attach_link]
        print(f"  Link {attach_link}: {len(subs)} rametti ({', '.join(subs)})")
    print()
    
    # Check physics
    print("Parametri fisici:")
    print(f"  {'Branch':<20} {'Radius(mm)':<12} {'Height(mm)':<12} {'Mass(kg)':<10} {'K(N*m/r)':<12}")
    print("-"*80)
    
    for b in branches[:3]:  # Mostra solo primi 3 per brevità
        r_w = scaled(b["radius"])
        h_w = scaled(b["height"])
        m = compute_mass(r_w, h_w)
        K, D = calculate_physics_params(r_w, h_w, m)
        print(f"  {b['id']:<20} {r_w*1000:<12.2f} {h_w*1000:<12.2f} {m:<10.4f} {K:<12.2f}")
    
    if len(branches) > 3:
        print(f"  ... (+ {len(branches)-3} altri rami)")
    print()
    
    return True


def generate_usd(branches, config_name="lateral_branches", skip_limit_check=False):
    """Genera il file USD."""
    print("="*80)
    print(" " * 27 + "GENERAZIONE USD")
    print("="*80)
    print()
    
    # Output path
    usd_dir = SCRIPT_DIR / "scalability_usds"
    usd_dir.mkdir(exist_ok=True)
    usd_path = usd_dir / f"{config_name}.usda"
    
    print(f"Generazione USD in corso...")
    print(f"Path: {usd_path}")
    if skip_limit_check:
        print("⚠️  skip_limit_check=True (oltre il limite PhysX di 64 links!)")
    print()
    
    try:
        stage, stem_path = build_stage(str(usd_path), branches, skip_limit_check=skip_limit_check)
        stage.GetRootLayer().Save()
        
        # Verifica rigid bodies
        link_count = sum(
            1 for prim in stage.Traverse()
            if prim.HasAPI(Usd.CollectionAPI)  # Check any API to count prims
        )
        
        print("✅ USD generato con successo!")
        print()
        print(f"File: {usd_path.name}")
        print(f"Size: {usd_path.stat().st_size / 1024:.2f} KB")
        print()
        
        return True, usd_path
        
    except Exception as e:
        print(f"❌ Errore durante generazione USD: {e}")
        import traceback
        traceback.print_exc()
        return False, None


def print_test_instructions(usd_path):
    """Stampa istruzioni per testare in Isaac Sim."""
    print("="*80)
    print(" " * 25 + "PROSSIMI PASSI - ISAAC SIM TEST")
    print("="*80)
    print()
    print("Il file USD è stato creato con successo!")
    print(f"Posizione: {usd_path}")
    print()
    print("Per testarlo in Isaac Sim, usa il test interattivo:")
    print()
    print("  cd ~/isaacsim/autotom_digital_twin")
    print("  python3 src/experiments/recursive_tree/tests/test_manual_cli.py")
    print()
    print("Il test:")
    print("  1. Caricherà automaticamente il nuovo USD in Isaac Sim")
    print("  2. Premi PLAY per osservare il comportamento")
    print("  3. Chiudi Isaac Sim quando hai finito")
    print("  4. Classifica la stabilità (STABLE/MARGINAL/UNSTABLE)")
    print()
    print("Cosa osservare:")
    print("  - I rametti laterali si stabilizzano rapidamente?")
    print("  - Ci sono oscillazioni o jitter?")
    print("  - La struttura mantiene la forma 'antenna'?")
    print("  - Collisioni tra rametti adiacenti?")
    print()
    print("="*80)


# ==============================================================================
# MAIN
# ==============================================================================

def run_test(test_num, test_name, config_func, usd_name, skip_limit_check=False):
    """Esegue un singolo test di generazione configurazione."""
    print()
    print("="*80)
    print(f" " * 30 + f"TEST {test_num}: {test_name}")
    print("="*80)
    print()
    
    # Step 1: Genera configurazione
    print(f"Step 1: Generazione configurazione '{test_name}'...")
    branches = config_func()
    total_links = sum(b["n_links"] for b in branches)
    print(f"✓ Configurazione generata: {len(branches)} rami, {total_links} links totali")
    if skip_limit_check:
        print(f"⚠️  ATTENZIONE: {total_links} links supera il limite PhysX di 64!")
    print()
    
    # Step 2: Valida
    print("Step 2: Validazione configurazione...")
    if not validate_config(branches, skip_limit_check=skip_limit_check):
        print("❌ Test fallito (validazione)")
        return False, None
    
    # Step 3: Genera USD
    print("Step 3: Generazione USD...")
    success, usd_path = generate_usd(branches, usd_name, skip_limit_check=skip_limit_check)
    if not success:
        print("❌ Test fallito (generazione USD)")
        return False, None
    
    print(f"✅ Test {test_num} completato con successo!")
    return True, usd_path


def main():
    print()
    print("="*80)
    print(" " * 20 + "TEST LATERAL BRANCHES - ANTENNA RESTRELLIERA")
    print("="*80)
    print()
    print("Questo script genera TRE configurazioni di test:")
    print("  1. Single branch con rametti laterali inclinati (27 links)")
    print("  2. Multi-branch con rametti orizzontali intersecati (64 links)")
    print("  3. Complex multi-branch - HEAVY TEST (~98 links)")
    print()
    
    results = []
    
    # ========== TEST 1: Single Branch Antenna ==========
    print("\n" + "█"*80)
    print("█" + " "*78 + "█")
    print("█" + " "*25 + "TEST 1: SINGLE BRANCH ANTENNA" + " "*25 + "█")
    print("█" + " "*78 + "█")
    print("█"*80)
    
    success1, usd_path1 = run_test(
        test_num=1,
        test_name="Single Branch Antenna",
        config_func=generate_lateral_branches_config,
        usd_name="lateral_branches_antenna"
    )
    results.append(("lateral_branches_antenna", success1, usd_path1))
    
    # ========== TEST 2: Multi-Branch Horizontal ==========
    print("\n" + "█"*80)
    print("█" + " "*78 + "█")
    print("█" + " "*22 + "TEST 2: MULTI-BRANCH HORIZONTAL" + " "*24 + "█")
    print("█" + " "*78 + "█")
    print("█"*80)
    
    success2, usd_path2 = run_test(
        test_num=2,
        test_name="Multi-Branch Horizontal",
        config_func=generate_multi_branch_horizontal_config,
        usd_name="multi_branch_horizontal"
    )
    results.append(("multi_branch_horizontal", success2, usd_path2))
    
    # ========== TEST 3: Complex Multi-Branch ==========
    print("\n" + "█"*80)
    print("█" + " "*78 + "█")
    print("█" + " "*20 + "TEST 3: COMPLEX MULTI-BRANCH (HEAVY)" + " "*21 + "█")
    print("█" + " "*78 + "█")
    print("█"*80)
    
    success3, usd_path3 = run_test(
        test_num=3,
        test_name="Complex Multi-Branch Heavy",
        config_func=generate_complex_multi_branch_config,
        usd_name="complex_multi_branch_heavy",
        skip_limit_check=True  # Supera il limite di 64 links!
    )
    results.append(("complex_multi_branch_heavy", success3, usd_path3))
    
    # ========== RIEPILOGO FINALE ==========
    print()
    print("="*80)
    print(" " * 30 + "RIEPILOGO FINALE")
    print("="*80)
    print()
    
    all_success = all(success for _, success, _ in results)
    
    for config_name, success, usd_path in results:
        status = "✅ SUCCESS" if success else "❌ FAILED"
        print(f"  {config_name:<35} {status}")
        if success and usd_path:
            print(f"    → {usd_path}")
    
    print()
    print("="*80)
    
    if all_success:
        print(" " * 25 + "✅ TUTTI I TEST COMPLETATI! ✅")
        print("="*80)
        print()
        print("File USD generati con successo!")
        print()
        print("PROSSIMI PASSI:")
        print()
        print("  1. Testa in Isaac Sim:")
        print("     cd ~/isaacsim/autotom_digital_twin")
        print("     python3 src/experiments/recursive_tree/tests/test_manual_cli.py")
        print()
        print("  2. Osserva entrambe le configurazioni:")
        print()
        print("     Test 1 - lateral_branches_antenna:")
        print("       • 1 ramo principale con 8 rametti laterali")
        print("       • Rametti inclinati 35°")
        print("       • Total: 27 links")
        print()
        print("     Test 2 - multi_branch_horizontal:")
        print("       • 3 rami principali ORIZZONTALI (tilt=90°, radiale 0°/120°/240°)")
        print("       • 6 rametti INTERSECATI per ramo (lati alternati 90°/270°)")
        print("       • Rami: radius 1cm (100mm world)")
        print("       • Rametti: radius 1mm (10mm world)")
        print("       • Total: 64 links ✓ (esattamente al limite!)")
        print()
        print("     Test 3 - complex_multi_branch_heavy:")
        print("       • 4 rami principali ORIZZONTALI (tilt=90°, radiale completo)")
        print("       • 7 rametti INTERSECATI per ramo (distribuzione uniforme)")
        print("       • Trunk più alto (10 links) e rami più lunghi (8 links)")
        print("       • Rami: radius 12mm (120mm world, più robusti)")
        print("       • Rametti: radius 1mm (10mm world)")
        print("       • Total: 98 links ✓ (heavy test!)")
        print()
        print("  3. Classifica la stabilità per ciascuno:")
        print("     • 1 = ✅ STABLE")
        print("     • 2 = ⚠️  MARGINAL")
        print("     • 3 = ❌ UNSTABLE")
        print()
        print("="*80)
    else:
        print(" " * 25 + "❌ ALCUNI TEST FALLITI ❌")
        print("="*80)
        failed = [name for name, success, _ in results if not success]
        print(f"\nTest falliti: {', '.join(failed)}")
        print()
        sys.exit(1)
    
    print()


if __name__ == "__main__":
    main()
