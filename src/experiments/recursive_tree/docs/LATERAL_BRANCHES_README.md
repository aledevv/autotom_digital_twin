# Test Lateral Branches - Antenna "Restrelliera"

## Descrizione

Questo test crea una struttura ad albero con rami laterali distribuiti in stile antenna a restrelliera. È progettato per testare la stabilità e la convergenza di configurazioni con molteplici attaccamenti laterali allo stesso ramo parent.

## Struttura

```
                    Trunk (verticale)
                       |
                       |
                   Link 3
                      /
                     /
              main_branch (inclinato 50°)
                   /
                  /
     Link 2:  ←-●-→     (2 rametti laterali)
                /
     Link 3:  ←-●-→     (2 rametti laterali)
                /
     Link 4:  ←-●-→     (2 rametti laterali)
                /
     Link 5:  ←-●-→     (2 rametti laterali)
                /
              Link 6
```

### Componenti

1. **Trunk (tronco principale)**
   - 5 links verticali
   - Radius: 10mm (100mm world scale)
   - Height: 25mm (250mm world scale)

2. **Main Branch (ramo principale)**
   - 6 links inclinati a 50°
   - Si attacca al trunk al link 3
   - Radius: 4mm (40mm world scale)
   - Height: 20mm (200mm world scale)
   - Rotazione azimutale: 45°

3. **Lateral Subbranches (rametti laterali)**
   - 8 rametti totali (2 links ciascuno)
   - Distribuiti ai lati del main_branch (rot 0° e 180°)
   - Attaccati ai links 2, 3, 4, 5 del main_branch
   - Radius: 2mm (20mm world scale)
   - Height: 15mm (150mm world scale)
   - Inclinazione: 35° rispetto al main_branch

## Caratteristiche Testate

### 1. Attaccamenti Multipli
- Ogni link del main_branch (2-5) ha 2 rametti che si attaccano ai lati opposti
- Testa il sistema di collision filtering tra siblings
- Verifica che rametti attaccati allo stesso punto non collidano tra loro

### 2. Pattern Simmetrico
- Rametti distribuiti simmetricamente (rot 0° e 180°)
- Testa il bilanciamento della struttura
- Verifica la stabilità con carichi laterali simmetrici

### 3. Complessità Moderata
- **Total links: 27**
  - Trunk: 5 links
  - Main branch: 6 links
  - Lateral subbranches: 16 links (8 × 2)
- Ben sotto il limite PhysX di 64 links
- Complessità sufficiente per testare la stabilità senza sovraccarico

## Parametri Fisici

| Component | Radius (mm) | Height (mm) | Mass (kg) | K (N·m/rad) | L/D Ratio |
|-----------|-------------|-------------|-----------|-------------|-----------|
| Trunk | 100.0 | 250.0 | 7.854 | 2741.56 | 1.25 |
| Main branch | 40.0 | 200.0 | 1.005 | 87.73 | 3.0 |
| Subbranches | 20.0 | 150.0 | 0.189 | 7.31 | 7.5 |

## Aspettative di Stabilità

### Scenari Attesi

**✅ STABLE (ideale)**
- I rametti laterali si stabilizzano rapidamente dopo il PLAY
- Nessuna oscillazione visibile
- La struttura mantiene la forma "antenna"
- Tempo di settling: < 2 secondi

**⚠️ MARGINAL (accettabile)**
- Lievi oscillazioni iniziali che si smorzano
- I rametti potrebbero avere piccoli movimenti pendolari
- Stabilizzazione entro 5 secondi
- Potrebbe richiedere aggiustamenti solver

**❌ UNSTABLE (problematico)**
- Oscillazioni continue o crescenti
- Jitter persistente
- Collisioni tra rametti adiacenti
- Non si stabilizza mai completamente

### Fattori Critici

1. **Collision Filtering**
   - Il sistema deve filtrare correttamente collisioni tra:
     - Rametti siblings (attaccati allo stesso link)
     - Rametti e il parent link successivo

2. **Solver Settings**
   - Position iterations: 64-128 raccomandati
   - Velocity iterations: 8-16 raccomandati
   - Damping: critico per rametti sottili (L/D = 7.5)

3. **Joint Stiffness**
   - Attachment joints: 5× stiffness normale
   - Internal joints: stiffness standard
   - Balance tra rigidità e stabilità numerica

## Generazione del Test

```bash
cd ~/isaacsim/autotom_digital_twin
uv run src/experiments/recursive_tree/tests/test_lateral_branches.py
```

Output:
- `scalability_usds/lateral_branches_antenna.usda` (~74 KB)

## Test in Isaac Sim

```bash
cd ~/isaacsim/autotom_digital_twin
python3 src/experiments/recursive_tree/tests/test_manual_cli.py
```

### Workflow

1. Il test caricherà automaticamente `lateral_branches_antenna.usda`
2. Isaac Sim si aprirà con la scena caricata
3. Premi **PLAY** per avviare la simulazione
4. Osserva il comportamento per 5-10 secondi
5. **CHIUDI Isaac Sim** quando hai finito
6. Classifica nel terminal:
   - `1` = ✅ STABLE
   - `2` = ⚠️ MARGINAL
   - `3` = ❌ UNSTABLE
   - `0` = ⏭️ SKIP (testa dopo)

### Cosa Osservare

**Comportamento Generale**
- [ ] I rametti si stabilizzano rapidamente?
- [ ] La forma "antenna" viene mantenuta?
- [ ] Ci sono oscillazioni pendolari?

**Interazioni**
- [ ] Rametti adiacenti collidono tra loro?
- [ ] Rametti collidono con il main_branch?
- [ ] Il main_branch si piega sotto il peso?

**Convergenza**
- [ ] Quanto tempo per stabilizzarsi?
- [ ] Ci sono drift o jitter persistenti?
- [ ] La struttura raggiunge un equilibrio statico?

## Confronto con Altri Test

| Test | Total Links | Max L/D | Complexity | Expected |
|------|-------------|---------|------------|----------|
| baseline_tomato | 41 | 5.87 | Medium | SAFE |
| lateral_branches | **27** | **7.5** | **Medium** | **?** |
| petiolule_ld_10 | 41 | 10.0 | Medium | RISKY |
| six_petioles_50 | 50 | 5.0 | High | MARGINAL |

**Vantaggi:**
- Meno links totali (27 vs 41) → più performante
- Pattern simmetrico → potenzialmente più stabile
- Testa specificamente attaccamenti laterali multipli

**Sfide:**
- L/D ratio più alto nei subbranches (7.5) → più flessibili
- 8 attachment points simultanei → stress sul parent
- Pattern "antenna" non comune in natura → comportamento incerto

## Possibili Variazioni

Se il test mostra instabilità, considerare:

1. **Ridurre L/D dei subbranches**
   ```python
   "height": 0.012,  # 12mm invece di 15mm → L/D = 6.0
   ```

2. **Aumentare stiffness degli attachment**
   ```python
   K_attach = K * 10.0  # invece di 5.0
   ```

3. **Ridurre numero di rametti**
   ```python
   # Solo links 2, 4 invece di 2, 3, 4, 5
   lateral_config = [
       (2, 0.0, "sub_L2_right"),
       (2, 180.0, "sub_L2_left"),
       (4, 0.0, "sub_L4_right"),
       (4, 180.0, "sub_L4_left"),
   ]
   ```

4. **Pattern asimmetrico**
   ```python
   # Alternare lato invece di simmetrico
   lateral_config = [
       (2, 0.0, "sub_L2_right"),
       (3, 180.0, "sub_L3_left"),
       (4, 0.0, "sub_L4_right"),
       (5, 180.0, "sub_L5_left"),
   ]
   ```

## File Correlati

- **Script generatore**: `test_lateral_branches.py`
- **USD output**: `scalability_usds/lateral_branches_antenna.usda`
- **Test runner**: `test_manual_cli.py`
- **Risultati**: `convergence_results.json` (dopo il test)

## Note

- Il filtro collisioni tra siblings è stato aggiunto automaticamente
- Il sistema usa attachment joints con stiffness 5× per gestire i laterali
- La configurazione è biologicamente plausibile per piante con pattern ramificato
- Il nome "restrelliera" si riferisce alle antenne TV tradizionali con elementi laterali

---

**Status**: ⏳ In attesa di test Isaac Sim  
**Created**: 2026-08-04  
**Priority**: Medium (test esplorativo)
