# Joint-Budget Optimization - Design Document

> **Documento di Design Tecnico**: Architettura, decisioni di design, e specifiche implementative per il sistema di ottimizzazione joints.

## Table of Contents

1. [Overview](#overview)
2. [Architecture](#architecture)
3. [Component Specifications](#component-specifications)
4. [Optimization Techniques](#optimization-techniques)
5. [Collision Detection System](#collision-detection-system)
6. [Geometry Remapping](#geometry-remapping)
7. [Configuration Schema](#configuration-schema)
8. [Data Flow](#data-flow)
9. [Design Decisions](#design-decisions)
10. [Extension Points](#extension-points)

---

## Overview

### Purpose

Il sistema di ottimizzazione joints riduce progressivamente il numero di articolazioni in una pianta USD per rispettare un budget hardware-imposed (~250 joints per Isaac Sim/PhysX).

### Design Goals

1. **Incremental**: Applica tecniche progressivamente fino a raggiungere il budget
2. **Transparent**: Report dettagliato di ogni passo con joints risparmiati
3. **Safe**: Validazione geometrica e strutturale dopo ogni tecnica
4. **Extensible**: Facile aggiungere nuove tecniche di ottimizzazione
5. **Configurable**: Parametri esterni (YAML) senza modificare codice

### Design Principles

- **Minimal Visual Impact**: Priorità alle tecniche che riducono joints preservando realismo
- **Structural Integrity**: Mai scendere sotto il lower bound strutturale
- **Collision-Free**: Validazione geometrica dopo remapping attachment
- **Fail-Safe**: Errore chiaro se ottimizzazione insufficiente

---

## Architecture

### High-Level Components

```
┌──────────────────────────────────────────────────────────────┐
│                      Optimizer (Orchestrator)                 │
│  - Load config YAML                                           │
│  - Calculate lower bound                                      │
│  - Apply techniques by priority                               │
│  - Generate report                                            │
└──────────────────────────┬───────────────────────────────────┘
                           │
        ┌──────────────────┼──────────────────┐
        │                  │                  │
        ▼                  ▼                  ▼
┌───────────────┐  ┌──────────────┐  ┌──────────────┐
│  Techniques   │  │  Collision   │  │  Geometry    │
│  - Base class │  │  - Sphere    │  │  - Remapping │
│  - 5 concrete │  │  - AABB      │  │  - Bounds    │
│    techniques │  │  - Broad ph. │  │              │
└───────────────┘  └──────────────┘  └──────────────┘
```

### Module Structure

```
exporterV2/core/optimizations/
├── __init__.py                 # Public API exports
├── optimizer.py                # Main orchestrator
├── budget_config.yaml          # Configuration
├── techniques/
│   ├── __init__.py
│   ├── base.py                 # Abstract base class
│   ├── stem_collapse.py        # Technique 1
│   ├── petiole_lock.py         # Technique 2
│   ├── lateral_reduce.py       # Technique 3
│   ├── truss_static.py         # Technique 4
│   └── leaf_branch_reduce.py   # Technique 5
├── collision/
│   ├── __init__.py
│   ├── sphere.py               # Sphere overlap
│   ├── aabb.py                 # AABB overlap
│   └── broad_phase.py          # Two-stage check
├── geometry/
│   ├── __init__.py
│   ├── remapping.py            # Attachment remapping
│   └── bounds.py               # Bounding volumes
└── tests/
    ├── test_*.py               # Unit + integration tests
    └── visual_validation/      # Isaac Sim validation
```


---

## Component Specifications

### 1. Optimizer (Orchestrator)

**File**: `optimizer.py`

**Responsibilities**:
- Load and parse `budget_config.yaml`
- Calculate total joints in current branches config
- Calculate structural lower bound
- Select and apply techniques by priority
- Validate after each technique
- Generate optimization report

**Key Methods**:

```python
class BudgetOptimizer:
    def __init__(self, config_path: str = "budget_config.yaml"):
        """Load configuration from YAML."""
        
    def calculate_total_joints(self, branches: List[Dict]) -> int:
        """Count total joints in current configuration."""
        
    def calculate_lower_bound(self, branches: List[Dict]) -> int:
        """Calculate minimum joints needed for structural integrity."""
        
    def optimize(self, branches: List[Dict]) -> Tuple[List[Dict], OptimizationReport]:
        """Apply techniques until budget met or exhausted."""
        
    def _select_next_technique(self, branches: List[Dict], 
                               current_joints: int) -> OptimizationTechnique:
        """Select highest-priority applicable technique."""
```

**Optimization Loop**:

1. Calculate total joints
2. If within budget → return
3. Calculate lower bound → if violated, raise BuildError
4. Select next technique by priority + `can_apply()`
5. Apply technique
6. Validate result
7. If valid → update branches, loop to step 1
8. If invalid → revert, try alternative or next technique


### 2. OptimizationTechnique (Base Class)

**File**: `techniques/base.py`

**Purpose**: Abstract interface per tutte le tecniche di ottimizzazione.

**Interface**:

```python
from abc import ABC, abstractmethod
from typing import List, Dict, Tuple
from dataclasses import dataclass

@dataclass
class OptimizationReport:
    """Report for a single technique application."""
    technique_name: str
    joints_before: int
    joints_after: int
    joints_saved: int
    details: Dict[str, any]  # Technique-specific info

@dataclass
class ValidationResult:
    """Result of geometry/collision validation."""
    valid: bool
    errors: List[str]
    warnings: List[str]

class OptimizationTechnique(ABC):
    """Base class for all optimization techniques."""
    
    @property
    @abstractmethod
    def name(self) -> str:
        """Technique name for reporting."""
        pass
    
    @property
    @abstractmethod
    def priority(self) -> int:
        """Priority (lower = applied first)."""
        pass
    
    @abstractmethod
    def can_apply(self, branches: List[Dict], 
                  current_joints: int, budget: int) -> bool:
        """Check if technique can be applied to current configuration."""
        pass
    
    @abstractmethod
    def estimate_reduction(self, branches: List[Dict]) -> int:
        """Estimate how many joints this would save."""
        pass
    
    @abstractmethod
    def apply(self, branches: List[Dict]) -> Tuple[List[Dict], OptimizationReport]:
        """Apply the technique and return modified config + report."""
        pass
    
    @abstractmethod
    def validate(self, branches: List[Dict]) -> ValidationResult:
        """Validate result (geometry, collisions, structural)."""
        pass
```


---

## Optimization Techniques

### Technique 1: Petiole Lock (D6 → Fixed Joint)

**File**: `techniques/petiole_lock.py`  
**Priority**: 1 (highest)  
**Impact**: Minimal visual, high DOF reduction

**Strategy**: Converte petiolule joints da D6 (6 DOF) a FixedJoint (0 DOF) senza cambiare geometria.

**Implementation Details**:
- Identifica tutti i branches con ID matching `"Petiolule_*"`
- Aggiungi metadata `{"joint_type": "fixed"}` a questi branches
- Modifica `build_chain()` per rispettare `joint_type` override

**Joint Reduction**:
- Ogni petiolule convertito: ~5-6 DOF ridotti (equivalente a ~0.8 joint PhysX)
- Tipica pianta day 100: ~40 petiolules → ~30 joints risparmiati

**Validation**:
- Topologia parent-child preservata
- Geometria identica (solo tipo joint cambia)
- Attachment points invariati

---

### Technique 2: Lateral Branch Reduction

**File**: `techniques/lateral_reduce.py`  
**Priority**: 2  
**Impact**: Medio visual (branch più rigidi), alto joint reduction

**Strategy**: Riduce incrementalmente n_links nei lateral branches fino al minimo configurato (default: 1).

**Implementation Details**:
- Per ogni lateral branch con `n_links > min_segments`:
  - Riduci `n_links` di 1
  - Ricalcola `height` = lunghezza_totale / nuovo_n_links
  - Preserva `radius`, `tilt`, `rot`
- Applica riduzione un link alla volta (iterativo)

**Joint Reduction**:
- 1 joint per link ridotto per branch
- Tipica pianta day 100: 20 lateral branches con 3→1 links = 40 joints risparmiati

**Validation**:
- `n_links >= min_segments` (default: 1)
- Lunghezza totale branch preservata (tolerance 1%)
- Child branches (foglie) attachment ancora validi


### Technique 3: Stem Collapse con Remapping

**File**: `techniques/stem_collapse.py`  
**Priority**: 3  
**Impact**: Medio-alto visual (trunk più rigido), alto joint reduction

**Strategy**: Collassa main stem (trunk) riducendo n_links, rimappando attachment points dei child branches, validando collisioni.

**Implementation Details**:
1. Riduci trunk `n_links` di 1
2. Per ogni child branch:
   - Calcola nuova altezza attachment con `remap_attachment_height()`
   - Aggiorna `attach_link` (nuovo indice link)
   - Valida collisioni con siblings usando `check_attachment_collision()`
3. Se collision irrisolvibile → fallback: prova offset alternativo o reverte

**Joint Reduction**:
- 1 joint per link trunk ridotto
- Tipica trunk: 10→5 links = 5 joints, 10→1 links = 9 joints

**Validation**:
- Tutti child attachment within trunk geometry
- No overlap tra siblings (broad-phase check)
- Preservazione topologia
- Altezze attachment preservate (tolerance 2%)

**Collision Resolution**:
- Se overlap detectato:
  1. Tenta micro-offset radiale (±5% radius)
  2. Se persiste, tenta attachment al link adiacente (±1)
  3. Se fallisce, reverte stem collapse step

---

### Technique 4: Truss Static Pre-bent

**File**: `techniques/truss_static.py`  
**Priority**: 4  
**Impact**: Medio visual (truss statici), medio joint reduction

**Strategy**: Converte truss multi-segment in single link con geometria mesh pre-piegata (no physics).

**Implementation Details**:
- Identifica truss branches
- Riduci a `n_links = 1`
- Genera mesh geometry pre-bent (invece di cilindro):
  - Applica `prebend_angle` per ogni segmento originale
  - Mesh simula curvatura gravitazionale
- Aggiungi metadata `{"prebent": true, "prebend_angle": X}`

**Joint Reduction**:
- (n_links_originali - 1) joints per truss
- Tipica pianta: 5 truss × 4 links = 15 joints risparmiati

**Validation**:
- Mesh geometry valida (no degenerate triangles)
- Attachment point al base preservato
- Fruit attachment points remappati lungo mesh

**Note**: Implementazione completa dipende da truss non ancora nel codebase. Per ora, implementa logica placeholder testabile con generic branches.


### Technique 5: Leaf Branch Reduction (Petiole+Rachis Merge)

**File**: `techniques/leaf_branch_reduce.py`  
**Priority**: 5 (lowest)  
**Impact**: Alto visual (foglie più rigide), medio-alto joint reduction

**Strategy**: Merge petiole + rachis in singolo segmento, opzionalmente pre-bent.

**Implementation Details**:
1. Identifica coppie (petiole, rachis) nella topologia foglie
2. Merge in single branch:
   - `length = petiole.length + rachis.length`
   - `radius = avg(petiole.radius, rachis.radius)`
   - `n_links = 1`
3. Se `prebend: true`:
   - Calcola `angle = petiole.tilt + rachis_tilt_avg`
   - Genera geometria pre-bent
4. Remap petiolules attachment al nuovo single branch

**Joint Reduction**:
- (petiole.n_links + rachis.n_links - 1) per foglia
- Tipica pianta: 30 foglie × 2 links = 60 joints risparmiati

**Validation**:
- Lunghezza totale preservata
- Petiolules attachment validi (no overlap)
- Angolo prebend realistico (< 90°)

---

## Collision Detection System

### Overview

Sistema two-stage broad-phase per validare attachment remapping senza overlap geometrico.

### Stage 1: Sphere Overlap (Fast Pre-check)

**File**: `collision/sphere.py`

**Purpose**: Esclusione rapida di attachment chiaramente validi o invalidi.

**Algorithm**:

```python
def calculate_bounding_sphere(link: CylinderGeometry) -> Tuple[Vec3, float]:
    """
    Calculate bounding sphere for cylindrical link.
    
    Args:
        link: Cylinder with base position, height, radius, orientation
    
    Returns:
        (center, radius) where center is midpoint, radius covers entire cylinder
    """
    center = link.base + 0.5 * link.height * link.axis
    radius = sqrt((link.height / 2)**2 + link.radius**2)
    return (center, radius)

def check_sphere_overlap(sphere1: Tuple[Vec3, float],
                         sphere2: Tuple[Vec3, float],
                         margin: float = 0.01) -> bool:
    """
    Check if two bounding spheres overlap.
    
    Returns:
        True if distance < r1 + r2 + margin (overlap detected)
    """
    c1, r1 = sphere1
    c2, r2 = sphere2
    distance = (c2 - c1).length()
    return distance < (r1 + r2 + margin)
```

**Usage**: Controlla attachment contro tutti siblings + parent. Se nessun overlap → safe. Se overlap → passa a Stage 2.


### Stage 2: AABB Overlap (Precision Check)

**File**: `collision/aabb.py`

**Purpose**: Verifica precisa overlap per casi dubbi da Stage 1.

**Algorithm**:

```python
def calculate_aabb(link: CylinderGeometry) -> Tuple[Vec3, Vec3]:
    """
    Calculate Axis-Aligned Bounding Box for oriented cylinder.
    
    Returns:
        (min_point, max_point) in world coordinates
    """
    # Sample 8 corner points of cylinder bounding box
    corners = []
    for z in [0, link.height]:
        for angle in [0, 90, 180, 270]:
            offset = link.radius * Vec3(cos(angle), sin(angle), 0)
            # Rotate offset by link orientation
            rotated_offset = link.orientation.rotate(offset)
            corner = link.base + z * link.axis + rotated_offset
            corners.append(corner)
    
    # AABB is min/max of all corners
    min_point = Vec3(min(c.x for c in corners),
                     min(c.y for c in corners),
                     min(c.z for c in corners))
    max_point = Vec3(max(c.x for c in corners),
                     max(c.y for c in corners),
                     max(c.z for c in corners))
    
    return (min_point, max_point)

def check_aabb_overlap(aabb1: Tuple[Vec3, Vec3],
                       aabb2: Tuple[Vec3, Vec3]) -> bool:
    """
    Check if two AABBs overlap.
    
    Returns:
        True if overlap detected on all 3 axes
    """
    min1, max1 = aabb1
    min2, max2 = aabb2
    
    # Overlap if intervals overlap on all axes
    return (min1.x <= max2.x and max1.x >= min2.x and
            min1.y <= max2.y and max1.y >= min2.y and
            min1.z <= max2.z and max1.z >= min2.z)
```

### Broad-Phase Orchestration

**File**: `collision/broad_phase.py`

```python
@dataclass
class CollisionResult:
    collision_detected: bool
    colliding_with: List[str]  # IDs of colliding branches
    stage_detected: str         # "sphere", "aabb", or "none"

def check_attachment_collision(
    new_link: CylinderGeometry,
    siblings: List[Tuple[str, CylinderGeometry]],  # (id, geometry)
    parent: CylinderGeometry,
    margin: float = 0.01
) -> CollisionResult:
    """
    Two-stage broad-phase collision check.
    
    Stage 1: Sphere overlap against all siblings + parent
    Stage 2: AABB overlap for candidates from Stage 1
    
    Returns:
        CollisionResult with detection details
    """
    # Stage 1: Sphere pre-check
    new_sphere = calculate_bounding_sphere(new_link)
    
    sphere_candidates = []
    for sibling_id, sibling_geom in siblings:
        sibling_sphere = calculate_bounding_sphere(sibling_geom)
        if check_sphere_overlap(new_sphere, sibling_sphere, margin):
            sphere_candidates.append((sibling_id, sibling_geom))
    
    # Also check parent
    parent_sphere = calculate_bounding_sphere(parent)
    if check_sphere_overlap(new_sphere, parent_sphere, margin):
        sphere_candidates.append(("parent", parent))
    
    if not sphere_candidates:
        return CollisionResult(False, [], "none")
    
    # Stage 2: AABB precision check
    new_aabb = calculate_aabb(new_link)
    
    collisions = []
    for candidate_id, candidate_geom in sphere_candidates:
        candidate_aabb = calculate_aabb(candidate_geom)
        if check_aabb_overlap(new_aabb, candidate_aabb):
            collisions.append(candidate_id)
    
    if collisions:
        return CollisionResult(True, collisions, "aabb")
    else:
        return CollisionResult(False, [], "sphere_only")
```


---

## Geometry Remapping

### Attachment Height Remapping

**File**: `geometry/remapping.py`

**Problem**: Quando il trunk collassa da N links a M links (M < N), gli attachment points dei child branches devono essere rimappati preservando l'altezza geometrica assoluta.

**Algorithm**:

```python
def remap_attachment_height(
    original_link_idx: int,      # 0-based index in original trunk
    original_n_links: int,
    new_n_links: int,
    segment_heights: List[float]  # Heights of original segments
) -> Tuple[int, float]:
    """
    Remap attachment point from original to collapsed trunk.
    
    Returns:
        (new_link_idx, offset_z): New 0-based link index and Z offset within link
    
    Example:
        Trunk 5 links (h=0.2 each) → 3 links (h=0.333 each)
        Original attachment: link 3 (z=0.6)
        New attachment: link 1, offset 0.267 (total z=0.6)
    """
    # Calculate absolute height of original attachment
    absolute_height = sum(segment_heights[:original_link_idx])
    
    # Calculate new segment height (assume uniform for simplicity)
    total_height = sum(segment_heights)
    new_segment_height = total_height / new_n_links
    
    # Find which new link contains this height
    new_link_idx = int(absolute_height / new_segment_height)
    
    # Clamp to valid range
    new_link_idx = min(new_link_idx, new_n_links - 1)
    
    # Calculate offset within new link
    new_link_base_height = new_link_idx * new_segment_height
    offset_z = absolute_height - new_link_base_height
    
    # Clamp offset to link height
    offset_z = min(offset_z, new_segment_height)
    
    return (new_link_idx, offset_z)
```

**Validation**: 
- Absolute height before = absolute height after (tolerance 1%)
- New link index within valid range [0, new_n_links - 1]
- Offset within link height [0, new_segment_height]

### Bounding Volume Calculation

**File**: `geometry/bounds.py`

```python
@dataclass
class CylinderGeometry:
    """Geometric representation of a branch link."""
    base: Vec3           # Base position (world)
    axis: Vec3           # Unit vector along link
    height: float        # Link height
    radius: float        # Link radius
    orientation: Quat    # Link orientation quaternion

def link_to_cylinder_geometry(
    branch: Dict,
    link_idx: int,
    branch_registry: Dict[str, Tuple[List, List, Vec3, Quat]]
) -> CylinderGeometry:
    """
    Convert branch config + link index to explicit cylinder geometry.
    
    Uses branch_registry from build_stage to get actual world positions.
    """
    branch_id = branch["id"]
    link_paths, link_bases, axis, orientation = branch_registry[branch_id]
    
    base = link_bases[link_idx]
    height = scaled(branch["height"])
    radius = scaled(branch["radius"])
    
    return CylinderGeometry(base, axis, height, radius, orientation)
```


---

## Configuration Schema

### budget_config.yaml

```yaml
# Budget and thresholds
budget:
  max_joints: 250                 # Hard limit (hardware-imposed)
  warning_threshold: 230          # Warn if approaching limit

# Structural minimum for each component type
structural_limits:
  trunk:
    min_links: 1
    description: "Main stem cannot be reduced below 1 link"
  
  lateral_branch:
    min_links: 1
    description: "Each lateral branch needs at least 1 link"
  
  petiole:
    min_links: 1
    description: "Petiole needs at least 1 link for attachment"
  
  rachis:
    min_links: 0
    description: "Rachis can be merged with petiole"
  
  petiolule:
    min_links: 0
    description: "Petiolules can be converted to fixed joints"
  
  truss:
    min_links: 1
    description: "Truss needs at least 1 link for attachment"

# Technique configurations (priority order)
techniques:
  - id: "petiole_lock"
    priority: 1
    description: "Convert petiolule D6 joints to Fixed joints"
    enabled: true
    params:
      convert_all_petiolules: true
  
  - id: "lateral_reduce"
    priority: 2
    description: "Reduce lateral branch segments"
    enabled: true
    params:
      min_segments: 1
      reduction_step: 1  # Reduce by N segments per iteration
  
  - id: "stem_collapse"
    priority: 3
    description: "Collapse main stem with attachment remapping"
    enabled: true
    params:
      min_segments: 1
      reduction_step: 1
      collision_check:
        broad_phase: "sphere"
        narrow_phase: "aabb"
        safety_margin: 0.01  # meters
        max_remap_attempts: 3
  
  - id: "truss_static"
    priority: 4
    description: "Convert truss to pre-bent static geometry"
    enabled: true
    params:
      min_segments: 1
      prebend_angle: 15  # degrees per original segment
  
  - id: "leaf_branch_reduce"
    priority: 5
    description: "Merge petiole+rachis to single pre-bent segment"
    enabled: true
    params:
      merge_petiole_rachis: true
      prebend: true
      max_prebend_angle: 90  # degrees

# Logging configuration
logging:
  level: "INFO"  # DEBUG, INFO, WARNING, ERROR
  show_technique_details: true
  show_collision_details: false  # Verbose collision logging
```

### Loading Configuration

```python
import yaml
from pathlib import Path

class BudgetConfig:
    """Configuration loader and validator."""
    
    def __init__(self, config_path: str = "budget_config.yaml"):
        self.config_path = Path(config_path)
        self.config = self._load_config()
        self._validate_config()
    
    def _load_config(self) -> Dict:
        """Load YAML configuration."""
        with open(self.config_path) as f:
            return yaml.safe_load(f)
    
    def _validate_config(self):
        """Validate required fields and value ranges."""
        assert "budget" in self.config
        assert "structural_limits" in self.config
        assert "techniques" in self.config
        assert self.config["budget"]["max_joints"] > 0
        # ... more validation
    
    @property
    def max_joints(self) -> int:
        return self.config["budget"]["max_joints"]
    
    @property
    def structural_limits(self) -> Dict:
        return self.config["structural_limits"]
    
    @property
    def techniques(self) -> List[Dict]:
        """Return techniques sorted by priority."""
        techs = self.config["techniques"]
        return sorted(techs, key=lambda t: t["priority"])
```


---

## Data Flow

### Complete Optimization Pipeline

```
CSV Input (groIMP export)
    ↓
parse_csv_to_branches()
    ↓
[BRANCHES config: List[Dict]]
    ↓
BudgetOptimizer.optimize() ────────────────┐
    │                                       │
    ├─ Load config YAML                     │
    ├─ Calculate total joints               │
    ├─ Calculate lower bound                │
    │                                       │
    └─ LOOP (until budget met):             │
        │                                   │
        ├─ Select next technique            │
        │   (by priority + can_apply())     │
        │                                   │
        ├─ Technique.apply()                │
        │   ├─ Modify branches config       │
        │   ├─ Remap attachments (if needed)│
        │   └─ Generate report chunk        │
        │                                   │
        ├─ Technique.validate()             │
        │   ├─ Check geometry               │
        │   ├─ Check collisions             │
        │   └─ Check structural limits      │
        │                                   │
        ├─ If valid:                        │
        │   └─ Update branches, continue    │
        ├─ If invalid:                      │
        │   └─ Revert, try alternative      │
        │                                   │
        └─ [Loop back]                      │
                                            │
[Optimized BRANCHES config] ←───────────────┘
    ↓
build_stage(branches)
    ↓
[USD file with reduced joints]
```

### Technique Application Flow (Example: Stem Collapse)

```
StemCollapseTechnique.apply(branches)
    ↓
1. Identify trunk branch
    ↓
2. Reduce n_links by 1
    ↓
3. FOR EACH child branch attached to trunk:
    │
    ├─ remap_attachment_height()
    │   └─ Calculate new (link_idx, offset_z)
    │
    ├─ Update child.attach_link = new_link_idx
    │
    ├─ Get sibling branches (same parent_rank ±1)
    │
    ├─ check_attachment_collision()
    │   ├─ Stage 1: Sphere overlap
    │   └─ Stage 2: AABB overlap (if needed)
    │
    ├─ IF collision:
    │   ├─ Try micro-offset (±5% radius)
    │   ├─ Try adjacent link (±1)
    │   └─ IF still collision: REVERT & ERROR
    │
    └─ Update branches config
    ↓
4. Generate OptimizationReport
    ↓
5. Return (modified_branches, report)
```


---

## Design Decisions

### 1. Why YAML Configuration Instead of Python Config?

**Decision**: Use external YAML file for budget, limits, and technique parameters.

**Rationale**:
- **Separation of Concerns**: Configuration is data, not code
- **Easier for Non-Programmers**: Ricercatori possono modificare budget senza toccare Python
- **Version Control**: Config changes sono più leggibili nei diffs
- **Runtime Flexibility**: Possibile caricare config diverse senza recompilare

**Alternative Considered**: 
- Python config in `tree_config.py`: Rejected perché richiede modifica codice per ogni cambio
- JSON: Rejected perché YAML è più human-readable (commenti, multiline)

---

### 2. Why Incremental Technique Application?

**Decision**: Applica tecniche una alla volta, validando dopo ogni step.

**Rationale**:
- **Early Exit**: Se budget è raggiunto dopo 2 tecniche, non applica le altre 3
- **Transparency**: Report mostra esattamente quale tecnica ha contribuito quanto
- **Safety**: Validazione dopo ogni step previene cascading errors
- **Debugging**: Facile identificare quale tecnica causa problemi

**Alternative Considered**:
- Batch application: Rejected perché impossibile tracciare contributi individuali
- Greedy optimization (applica tutto subito): Rejected perché over-optimizes e perde realismo

---

### 3. Why Two-Stage Collision Check (Sphere + AABB)?

**Decision**: Usa sphere overlap come pre-check, poi AABB per precisione.

**Rationale**:
- **Performance**: Sphere check è O(1) per pair, AABB è più costoso
- **False Positives**: Sphere è conservativo → molti falsi positivi per cilindri lunghi
- **Precision**: AABB cattura orientamento, riduce falsi positivi del 70-80%
- **Industry Standard**: Approccio documentato in game engines (Unity, Unreal)

**Alternative Considered**:
- Solo AABB: Rejected perché troppo lento per controllare tutti i sibling pairs
- Oriented Bounding Box (OBB): Rejected perché complessità implementativa >> beneficio
- PhysX query diretta: Rejected perché richiede USD stage già costruito (chicken-egg problem)

---

### 4. Why Plugin Architecture for Techniques?

**Decision**: Abstract base class + concrete implementations.

**Rationale**:
- **Extensibility**: Facile aggiungere nuove tecniche senza modificare orchestrator
- **Testability**: Ogni tecnica è isolata e testabile indipendentemente
- **Maintainability**: Cambiamenti a una tecnica non impattano le altre
- **Reusability**: Tecniche possono essere abilitate/disabilitate via config

**Alternative Considered**:
- Monolithic optimizer con if/else: Rejected perché non scalabile
- Function-based approach: Rejected perché manca structure per validation e reporting

---

### 5. Why Calculate Lower Bound Before Optimization?

**Decision**: Calcola lower bound upfront e fallisce fast se impossibile.

**Rationale**:
- **Fail Fast**: Se budget è impossibile da rispettare, fallisce prima di sprecare compute
- **Clear Error Messages**: User sa subito che la pianta non è esportabile
- **Avoid Partial Results**: Non genera USD parziali/corrotti
- **Matches LOD Literature**: Standard practice in mesh simplification (topology-preserving constraint)

**Alternative Considered**:
- Calculate on-demand: Rejected perché spreca tempo se lower bound > budget
- Soft warning invece di error: Rejected perché genera USD instabili che crashano Isaac Sim

---

### 6. Why Priority-Based Technique Selection?

**Decision**: Tecniche hanno priorità fissa (1-5), applicate in ordine.

**Rationale**:
- **Predictability**: Stesso input → stesso output (deterministic)
- **Visual Quality**: Ordine basato su impatto visivo minimo (research-backed)
- **Simplicity**: No need for complex heuristics o machine learning
- **Debuggability**: Facile capire quale tecnica viene applicata quando

**Alternative Considered**:
- Greedy by estimated reduction: Rejected perché tecnica con max reduction può avere max visual impact
- Cost-benefit optimization: Rejected perché complesso quantificare "visual quality"
- User-specified order: Supported via config, ma default è priority-based


---

## Extension Points

### Adding a New Optimization Technique

**Steps**:

1. **Create new file** in `techniques/`:
   ```python
   # techniques/my_new_technique.py
   from .base import OptimizationTechnique, OptimizationReport, ValidationResult
   
   class MyNewTechnique(OptimizationTechnique):
       @property
       def name(self) -> str:
           return "My New Technique"
       
       @property
       def priority(self) -> int:
           return 6  # Lower than existing
       
       def can_apply(self, branches, current_joints, budget):
           # Check if applicable
           pass
       
       def estimate_reduction(self, branches):
           # Estimate joints saved
           pass
       
       def apply(self, branches):
           # Modify branches
           # Return (modified_branches, report)
           pass
       
       def validate(self, branches):
           # Validate result
           # Return ValidationResult
           pass
   ```

2. **Add to config** in `budget_config.yaml`:
   ```yaml
   techniques:
     - id: "my_new_technique"
       priority: 6
       description: "Description of what it does"
       enabled: true
       params:
         param1: value1
         param2: value2
   ```

3. **Register in optimizer**:
   ```python
   # optimizer.py
   from .techniques.my_new_technique import MyNewTechnique
   
   # In BudgetOptimizer.__init__():
   self.technique_registry["my_new_technique"] = MyNewTechnique(params)
   ```

4. **Write tests**:
   ```python
   # tests/test_my_new_technique.py
   def test_can_apply():
       # Test logic
       pass
   
   def test_estimate_reduction():
       pass
   
   def test_apply():
       pass
   
   def test_validate():
       pass
   ```

---

### Adding a New Collision Detection Stage

**Example**: Add oriented bounding box (OBB) as Stage 3.

1. **Create new file** `collision/obb.py`:
   ```python
   def calculate_obb(link: CylinderGeometry) -> OBB:
       """Calculate oriented bounding box."""
       pass
   
   def check_obb_overlap(obb1: OBB, obb2: OBB) -> bool:
       """Separating axis theorem."""
       pass
   ```

2. **Extend broad_phase.py**:
   ```python
   def check_attachment_collision(..., use_obb: bool = False):
       # ... existing sphere + AABB code ...
       
       if use_obb and aabb_candidates:
           # Stage 3: OBB precision check
           for candidate in aabb_candidates:
               if check_obb_overlap(new_obb, candidate_obb):
                   collisions.append(candidate)
       
       # ...
   ```

3. **Add to config**:
   ```yaml
   collision_check:
     broad_phase: "sphere"
     narrow_phase: "aabb"
     precision_phase: "obb"  # NEW
   ```

---

### Customizing Lower Bound Calculation

**Use Case**: Different plant types have different structural minimums.

**Approach**: Make lower bound calculation pluggable:

```python
# optimizer.py
class BudgetOptimizer:
    def __init__(self, config_path, lower_bound_calculator=None):
        self.lower_bound_calc = lower_bound_calculator or DefaultLowerBoundCalculator()
    
    def calculate_lower_bound(self, branches):
        return self.lower_bound_calc.calculate(branches, self.config.structural_limits)

# Custom calculator for different plant type
class CucumberLowerBoundCalculator:
    def calculate(self, branches, limits):
        # Different logic for cucumbers
        pass
```

---

### Adding New Geometry Remapping Strategies

**Example**: Non-uniform segment heights in trunk.

**Current**: Assumes uniform segment heights for simplicity.

**Extension**:

```python
# geometry/remapping.py
def remap_attachment_height_nonuniform(
    original_link_idx: int,
    segment_heights_original: List[float],  # Actual heights per segment
    segment_heights_new: List[float]        # New heights after collapse
) -> Tuple[int, float]:
    """
    Remapping with non-uniform segment heights.
    """
    # Calculate absolute height using actual segment heights
    absolute_height = sum(segment_heights_original[:original_link_idx])
    
    # Find new link by iterating through new segment heights
    cumulative = 0.0
    for new_idx, new_height in enumerate(segment_heights_new):
        if cumulative + new_height >= absolute_height:
            offset = absolute_height - cumulative
            return (new_idx, offset)
        cumulative += new_height
    
    # Fallback: attach to last link
    return (len(segment_heights_new) - 1, segment_heights_new[-1])
```

---

## Summary

Questo design document fornisce:

1. **Architettura completa** del sistema di ottimizzazione
2. **Specifiche dettagliate** per ogni componente
3. **Algoritmi concreti** per collision detection e remapping
4. **Rationale** per decisioni di design principali
5. **Extension points** per future modifiche

Il sistema è progettato per essere:
- **Modulare**: Componenti isolati e testabili
- **Configurabile**: YAML per parametri esterni
- **Estendibile**: Facile aggiungere nuove tecniche
- **Robusto**: Validazione dopo ogni step
- **Trasparente**: Report dettagliato dell'ottimizzazione

Per l'implementazione, segui l'**Implementation Plan** (`OPTIMIZATION_IMPLEMENTATION_PLAN.md`) che suddivide il lavoro in 12 task incrementali.
