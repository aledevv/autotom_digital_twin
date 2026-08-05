# Joint-Budget Optimization System - Documentation Index

> **Sistema di ottimizzazione incrementale per ridurre joints in piante USD Isaac Sim**

## 📚 Documentazione Disponibile

### 1. [OPTIMIZATION_IMPLEMENTATION_PLAN.md](./OPTIMIZATION_IMPLEMENTATION_PLAN.md)
**Cosa contiene**: Checklist dettagliata delle 12 task di implementazione con status tracking.

**Quando usarlo**:
- Iniziare nuova task
- Verificare dipendenze tra task
- Tracciare progresso implementazione
- Stimare effort rimanente

**Aggiornamenti**: Marca task come ✅ DONE man mano che completi

---

### 2. [OPTIMIZATION_DESIGN.md](./OPTIMIZATION_DESIGN.md)
**Cosa contiene**: Architettura tecnica, specifiche componenti, algoritmi dettagliati, decisioni di design.

**Quando usarlo**:
- Capire come funziona un componente
- Implementare una tecnica di ottimizzazione
- Estendere il sistema (nuova tecnica, nuovo collision stage)
- Risolvere dubbi architetturali

**Sezioni chiave**:
- Component Specifications
- Optimization Techniques (5 tecniche dettagliate)
- Collision Detection System (Sphere + AABB)
- Geometry Remapping (algoritmi)
- Configuration Schema (YAML completo)
- Design Decisions (rationale)

---

### 3. [OPTIMIZATION_QUICK_START.md](./OPTIMIZATION_QUICK_START.md)
**Cosa contiene**: Guida rapida per iniziare, esempi d'uso, troubleshooting.

**Quando usarlo**:
- Primo approccio al sistema
- Quick reference durante implementazione
- Debugging problemi comuni
- Esempi di utilizzo API

**Sezioni chiave**:
- Ordine implementazione consigliato
- Esempi codice per usare il sistema
- File structure reference
- Common issues & solutions

---

### 4. Questo File (README)
**Cosa contiene**: Overview generale e index della documentazione.

---

## 🎯 Quick Navigation

| Voglio... | Vai a... |
|-----------|----------|
| Iniziare implementazione | [Implementation Plan](./OPTIMIZATION_IMPLEMENTATION_PLAN.md) → Task 1 |
| Capire architettura | [Design Doc](./OPTIMIZATION_DESIGN.md) → Architecture |
| Implementare una tecnica | [Design Doc](./OPTIMIZATION_DESIGN.md) → Optimization Techniques |
| Capire collision detection | [Design Doc](./OPTIMIZATION_DESIGN.md) → Collision Detection System |
| Esempi codice | [Quick Start](./OPTIMIZATION_QUICK_START.md) → Uso Base |
| Troubleshooting | [Quick Start](./OPTIMIZATION_QUICK_START.md) → Common Issues |
| Configurare budget | [Quick Start](./OPTIMIZATION_QUICK_START.md) → Configuration |
| Testare | [Quick Start](./OPTIMIZATION_QUICK_START.md) → Testing Reference |

---

## 📋 Executive Summary

### Problema
Isaac Sim/PhysX ha un limite hardware-imposed di ~250 joints per articolazioni. Piante di pomodoro al day 160 con truss e frutti superano questo limite, causando instabilità o crash.

### Soluzione
Sistema di ottimizzazione incrementale che applica 5 tecniche LOD-based in ordine di impatto visivo minimo, riducendo joints fino a rientrare nel budget mantenendo integrità strutturale.

### Tecniche (Priority Order)
1. **Petiole Lock** (Priority 1): D6 → Fixed joint (no geometry change)
2. **Lateral Reduce** (Priority 2): Riduci segments lateral branches
3. **Stem Collapse** (Priority 3): Collassa trunk + remap attachments
4. **Truss Static** (Priority 4): Pre-bent static geometry
5. **Leaf Branch Reduce** (Priority 5): Merge petiole+rachis

### Key Features
- ✅ **Incremental**: Applica tecniche progressivamente, stop quando budget raggiunto
- ✅ **Safe**: Validazione geometrica + collision check dopo ogni step
- ✅ **Transparent**: Report dettagliato con breakdown per tecnica
- ✅ **Configurable**: YAML esterno per budget, limiti, parametri
- ✅ **Extensible**: Plugin architecture per nuove tecniche

### Validation
- **Research-backed**: Approccio validato da letteratura LOD/MOR (vedi `Research_Joint-Budget Optimization for USD Tomato Plant Exporter Approach Validation and SOTA Review.md`)
- **Industry standard**: Collision detection broad-phase (Sphere + AABB) usato in game engines

---

## 🚀 Getting Started

### Per Implementatori

1. **Leggi** [Implementation Plan](./OPTIMIZATION_IMPLEMENTATION_PLAN.md) per overview delle 12 task
2. **Inizia** con Task 1 (Setup Infrastructure)
3. **Consulta** [Design Doc](./OPTIMIZATION_DESIGN.md) durante implementazione
4. **Usa** [Quick Start](./OPTIMIZATION_QUICK_START.md) per riferimenti rapidi
5. **Aggiorna** Implementation Plan man mano che completi task

### Per Utilizzatori (Dopo Implementazione)

```python
# Esempio base
from exporterV2.core.optimizations import BudgetOptimizer

optimizer = BudgetOptimizer()
optimized_branches, report = optimizer.optimize(branches)
print(report)
```

```bash
# Da CLI
./run_mainV2.sh --day 50 --optimize
```

Vedi [Quick Start](./OPTIMIZATION_QUICK_START.md) per esempi completi.

---

## 📊 Implementation Status

**Ultima Verifica**: YYYY-MM-DD

| Phase | Tasks | Status |
|-------|-------|--------|
| Phase 1: Infrastructure | 1-3 | 🔴 Not Started |
| Phase 2: Techniques | 4-8 | 🔴 Not Started |
| Phase 3: Integration | 9-12 | 🔴 Not Started |

**Next Step**: Task 1 - Setup Infrastructure

Vedi [Implementation Plan](./OPTIMIZATION_IMPLEMENTATION_PLAN.md) per dettagli.

---

## 🏗️ Architecture Overview

```
Optimizer (Orchestrator)
    ↓
Apply Techniques by Priority
    ↓
┌─────────────┬──────────────┬────────────┐
│  Technique  │  Collision   │  Geometry  │
│  Plugins    │  Detection   │  Remapping │
└─────────────┴──────────────┴────────────┘
    ↓
Optimized Branches Config
    ↓
build_stage() → USD Export
```

Vedi [Design Doc](./OPTIMIZATION_DESIGN.md) → Architecture per dettagli.

---

## 📝 Notes

### Design Principles
- **Minimal Visual Impact**: Priorità tecniche che preservano realismo
- **Structural Integrity**: Mai scendere sotto lower bound
- **Fail Safe**: Errore chiaro se ottimizzazione insufficiente
- **Transparent**: Report traccia ogni passo

### Research Foundation
Approccio validato da 3 domini:
- **Skeletal Animation**: Bone-count LOD reduction
- **Vegetation Rendering**: Tree branch LOD simplification
- **Multibody Dynamics**: Model order reduction (MOR)

Vedi `Research_Joint-Budget Optimization for USD Tomato Plant Exporter Approach Validation and SOTA Review.md`

---

## 🔗 Related Documentation

- **Research Background**: `Research_Joint-Budget Optimization for USD Tomato Plant Exporter Approach Validation and SOTA Review.md`
- **Collision Recommendation**: `collision_check_recommendation.md`
- **Tree Config**: `../core/tree_config.py`
- **USD Stage Builder**: `../core/usd/stage.py`

---

## 📞 Support

Per domande, problemi o suggerimenti:
- Controlla [Quick Start - Common Issues](./OPTIMIZATION_QUICK_START.md#common-issues--solutions)
- Leggi Design Decisions in [Design Doc](./OPTIMIZATION_DESIGN.md#design-decisions)
- Apri issue su GitHub
- Contatta il team

---

**Documenti creati**: 2024-01-XX  
**Versione**: 1.0  
**Autore**: Alessandro (Planning Agent)
