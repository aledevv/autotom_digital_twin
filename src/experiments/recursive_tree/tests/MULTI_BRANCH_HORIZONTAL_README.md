# Test Multi-Branch Horizontal - Rami Paralleli al Terreno

## Descrizione

Test con **5 rami principali orizzontali** (ridotti a **3** per rispettare il limite di 64 links) che si espandono dal trunk in direzioni radiali. Ogni ramo ha **6 rametti laterali** che si estendono anch'essi orizzontalmente, paralleli al terreno.

Questo test è una **variante ridotta e orizzontale** del test lateral_branches_antenna.

## Struttura

```
Vista dall'alto (orizzontale):

                    Trunk
                   (vert.)
                      |
         Branch_3 ----+---- Branch_1 (rot 0°)
                 \    |    /
                  \   |   /
                   \  |  /
                    \ | /
                   Branch_2
                  (rot 120°)


Vista laterale di un singolo branch:

    Trunk ──→ Branch (orizzontale, tilt=90°)
               ├── L2: ↑ sub_up    (rot 0°)
               │       ↓ sub_down  (rot 180°)
               │
               ├── L3: ↑ sub_up
               │       ↓ sub_down
               │
               └── L4: ↑ sub_up
                       ↓ sub_down
```

### Componenti

1. **Trunk (tronco principale)**
   - 8 links verticali (più alto per ospitare 3 rami)
   - Radius: 8mm (80mm world scale) - **più sottile**
   - Height: 20mm (200mm world scale)

2. **3 Main Branches (rami principali orizzontali)**
   - 4 links ciascuno
   - **tilt=90°** → ORIZZONTALI (paralleli al terreno)
   - Si attaccano al trunk ai link 3, 5, 7
   - Rotazioni azimutali: 0°, 120°, 240° (distribuzione radiale)
   - Radius: 3mm (30mm world scale) - **più sottili**
   - Height: 15mm (150mm world scale) - **più corti**

3. **18 Lateral Subbranches (6 per branch)**
   - 2 links ciascuno
   - **tilt=90°** → ORIZZONTALI
   - Distribuiti ai link 2, 3, 4 di ogni branch
   - rot 0° (su) e 180° (giù) rispetto al parent branch
   - Radius: 1.5mm (15mm world scale) - **più sottili**
   - Height: 10mm (100mm world scale) - **più corti**

## Caratteristiche

### 1. Orientamento Orizzontale
- **Rami principali**: tilt=90° → si espandono parallelamente al terreno
- **Rametti laterali**: tilt=90° → anch'essi orizzontali
- Testa la stabilità con gravità che agisce perpendicolarmente ai rami

### 2. Pattern Radiale
- 3 rami distribuiti radialmente: 0°, 120°, 240°
- Copertura uniforme dello spazio orizzontale
- Simula struttura di pianta con crescita radiale

### 3. Dimensioni Ridotte
- **25% più sottili** rispetto a lateral_branches_antenna
- **25-33% più corti** rispetto a lateral_branches_antenna
- Testa la stabilità con elementi più flessibili

### 4. Complessità Controllata
- **Total links: 56**
  - Trunk: 8 links
  - Main branches: 12 links (3 × 4)
  - Lateral subbranches: 36 links (18 × 2)
- Sotto il limite PhysX di 64 links (margine: 8 links)

## Parametri Fisici

| Component | Radius (mm) | Height (mm) | Mass (kg) | K (N·m/rad) | L/D Ratio |
|-----------|-------------|-------------|-----------|-------------|-----------|
| Trunk | 80.0 | 200.0 | 4.021 | 1403.68 | 1.25 |
| Main branches | 30.0 | 150.0 | 0.424 | 37.01 | 5.0 |
| Subbranches | 15.0 | 100.0 | 0.071 | 3.47 | 6.67 |

### Confronto con lateral_branches_antenna

| Parametro | Antenna | Horizontal | Variazione |
|-----------|---------|------------|------------|
| **Trunk radius** | 100mm | 80mm | -20% |
| **Trunk links** | 5 | 8 | +60% |
| **Branch radius** | 40mm | 30mm | -25% |
| **Branch height** | 200mm | 150mm | -25% |
| **Sub radius** | 20mm | 15mm | -25% |
| **Sub height** | 150mm | 100mm | -33% |
| **Branches** | 1 | 3 | +200% |
| **Total links** | 27 | 56 | +107% |

## Aspettative di Stabilità

### Scenari Attesi

**✅ STABLE (ideale)**
- I rami orizzontali si stabilizzano rapidamente (< 3 secondi)
- Nessun droop eccessivo dovuto alla gravità
- I rametti laterali mantengono l'orientamento orizzontale
- Pattern radiale ben bilanciato

**⚠️ MARGINAL (accettabile)**
- Droop moderato dei rami orizzontali (normale per gravità)
- Oscillazioni verticali che si smorzano in 5-10 secondi
- Rametti più flessibili mostrano leggero movimento
- Struttura raggiunge equilibrio stabile entro 10 secondi

**❌ UNSTABLE (problematico)**
- Droop eccessivo (rami toccano il terreno)
- Oscillazioni verticali persistenti
- Collisioni tra rami/rametti
- Jitter continuo o instabilità numerica

### Sfide Specifiche

1. **Gravità Perpendicolare**
   - Rami orizzontali subiscono massimo stress da gravità
   - Droop prevedibile: ~20-50mm per rami (dimensioni ridotte)
   - Rametti più sottili: droop maggiore

2. **Dimensioni Ridotte**
   - K (stiffness) ridotta → più flessibilità
   - L/D ratio più alto nei subbranches (6.67 vs 7.5 antenna)
   - Potrebbe richiedere solver iterations più alte

3. **Complessità**
   - 56 links: vicino al limite (64)
   - 18 attachment joints simultanei
   - Pattern radiale: possibili interazioni tra rami adiacenti

## Test in Isaac Sim

```bash
cd ~/isaacsim/autotom_digital_twin
python3 src/experiments/recursive_tree/tests/test_manual_cli.py
```

### Cosa Osservare

**Rami Principali**
- [ ] Droop verticale accettabile? (< 100mm)
- [ ] Si stabilizzano in posizione orizzontale?
- [ ] Oscillazioni verticali si smorzano?
- [ ] Pattern radiale mantiene forma?

**Rametti Laterali**
- [ ] Mantengono orientamento orizzontale?
- [ ] Droop accettabile? (< 50mm)
- [ ] Oscillazioni si smorzano rapidamente?
- [ ] Nessuna collisione tra rametti adiacenti?

**Convergenza Generale**
- [ ] Tempo di settling: < 10 secondi?
- [ ] Raggiunge equilibrio statico?
- [ ] Nessun drift continuo?
- [ ] Nessun jitter o instabilità numerica?

**Performance**
- [ ] FPS accettabile (> 30)?
- [ ] Nessun warning PhysX?
- [ ] 56 links gestibili dal solver?

## Variazioni Possibili

### Se mostra UNSTABLE

1. **Aumentare stiffness attachment**
   ```python
   K_attach = K * 10.0  # invece di 5.0
   ```

2. **Ridurre numero rami**
   ```python
   # Solo 2 branches invece di 3
   main_branches_config = [
       ("branch_1", 3, 0.0),
       ("branch_2", 6, 180.0),
   ]
   # Total: 44 links
   ```

3. **Ridurre rametti laterali**
   ```python
   # Solo 4 rametti per branch invece di 6
   lateral_config = [
       (2, 0.0, "L2_up"),
       (2, 180.0, "L2_down"),
       (4, 0.0, "L4_up"),
       (4, 180.0, "L4_down"),
   ]
   # Total: 44 links
   ```

4. **Aumentare dimensioni (meno flessibile)**
   ```python
   "radius": 0.004,   # 4mm invece di 3mm
   "height": 0.018,   # 18mm invece di 15mm
   ```

### Se mostra STABLE

1. **Aggiungere più rami**
   ```python
   # 4 branches: 0°, 90°, 180°, 270°
   # Attenzione al limite 64 links!
   ```

2. **Ridurre ulteriormente dimensioni**
   ```python
   "radius": 0.0012,  # 1.2mm invece di 1.5mm
   "height": 0.008,   # 8mm invece di 10mm
   ```

3. **Test con angoli diversi**
   ```python
   "tilt": 70.0,  # Semi-orizzontale invece di 90°
   ```

## Confronto con Altri Test

| Test | Links | Orientation | Branches | L/D Max | Expected |
|------|-------|-------------|----------|---------|----------|
| lateral_antenna | 27 | Inclinati | 1 | 7.5 | ? |
| **multi_horizontal** | **56** | **Orizzontali** | **3** | **6.67** | **?** |
| baseline_tomato | 41 | Mixed | 4+12 | 5.87 | SAFE |
| six_petioles | 50 | Mixed | 6+18 | 5.0 | MARGINAL |

**Vantaggi:**
- Testa specificamente orientamento orizzontale (gravità perpendicolare)
- Pattern radiale realistico
- Dimensioni ridotte → test limiti flessibilità

**Sfide:**
- Più links (56 vs 27) → più stress solver
- Dimensioni ridotte → più flessibile
- Gravità perpendicolare → droop massimo

## File Generati

- **Script**: `test_lateral_branches.py` (genera entrambi i test)
- **USD**: `scalability_usds/multi_branch_horizontal.usda` (~154 KB)
- **Test runner**: `test_manual_cli.py`
- **Risultati**: `convergence_results.json` (dopo test)

## Note Tecniche

- **18 sibling collision filters** applicati automaticamente
- **Attachment joints**: stiffness 5× per gestire orientamento orizzontale
- **Center of mass**: esplicitamente impostato per evitare torque spurio
- **Collision filtering**: include parent + next sibling in chain

---

**Status**: ⏳ In attesa di test Isaac Sim  
**Created**: 2026-08-04  
**Priority**: Medium-High (test complessità + orientamento)  
**Related**: `lateral_branches_antenna` (versione più semplice)
