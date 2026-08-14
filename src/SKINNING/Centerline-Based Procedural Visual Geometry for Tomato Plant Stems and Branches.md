# Centerline-Based Procedural Visual Geometry for Tomato Plant Stems and Branches

## 1. Obiettivo

Lo scopo di questa proposta è sostituire l'attuale rappresentazione visiva schematica di stelo, rami, piccioli, rachis, truss e pedicelli con una **mesh continua e morfologicamente più simile a quella di una pianta di pomodoro reale**, senza modificare il modello fisico già esistente.

L'obiettivo non è ricostruire nuovi organi né aumentare la complessità botanica della pianta. Si mantengono esattamente gli organi già descritti dal modello GroIMP/exporter.

La modifica riguarda esclusivamente la loro **forma visiva**.

Il principio fondamentale è:

> La simulazione fisica rimane basata sugli attuali rigid link, D6 joint, FixedJoint e collision proxy; una seconda geometria continua viene generata attorno alle centerline degli organi e deformata tramite skinning seguendo le trasformazioni dei rigid link.

---

# 2. Situazione attuale

Exporter V2 genera la pianta a partire dalla struttura GroIMP, trasformandola in trunk, lateral branches, petiole, rachis, petiolules, truss, pedicels e terminal bodies. Successivamente `build_stage()` costruisce la scena USD e la relativa articolazione fisica. 

Attualmente `create_rigid_segment()` crea ogni link come `Xform` rigid body contenente un `UsdGeom.Cylinder`, sul quale viene applicata anche la collisione. 

Concettualmente:

```text
/PhysicalLink_01
└── Cylinder

/PhysicalLink_02
└── Cylinder

/PhysicalLink_03
└── Cylinder
```

Questo è ottimo per la simulazione ma produce una geometria fortemente riconoscibile come assemblaggio di primitive.

La proposta introduce invece la separazione:

```text
PHYSICS
├── rigid links
├── D6 joints
├── collision cylinders
└── masses / COM

VISUAL
└── continuous skinned mesh
```

La visual mesh non deve contribuire a:

- massa;
- collisioni;
- centro di massa;
- joint;
- stiffness;
- damping.

---

# 3. Architettura proposta

Per ogni organo fisicamente articolato:

```text
Physical skeleton

●────────●────────●────────●
Link 1     Link 2     Link 3
     D6         D6
```

viene costruita una superficie continua:

```text
Visual envelope

     ╭────────────────────────╮
   ╭─╯                        ╰─╮
●────────●────────●────────●
   ╰─╮                        ╭─╯
     ╰────────────────────────╯
```

I link fisici diventano concettualmente i **bone** del sistema visuale.

La visual mesh viene creata una volta nella rest pose e successivamente deformata tramite skinning.

---

# 4. Centerline

## 4.1 Definizione

Per ogni ramo si definisce una centerline:

\[
C(s)
\]

dove \(s\) rappresenta la distanza lungo l'asse longitudinale dell'organo.

I punti iniziali possono essere ottenuti direttamente dalla struttura fisica:

```text
C0 ●────────● C1────────● C2────────● C3
```

Idealmente i punti corrispondono alle posizioni dei joint o alle estremità dei rigid link.

Poiché la geometria fisica e quella visuale derivano dalla stessa centerline, il loro allineamento è noto per costruzione.

---

# 5. Centerline fisica e centerline visuale

La centerline fisica può essere una polilinea:

```text
──────●
       \
        \
         ●──────
```

Utilizzarla direttamente produrrebbe una geometria con cambi di direzione visibili.

La centerline visuale può quindi utilizzare una interpolazione locale smooth:

```text
──────╮
       ╲
        ╰──────
```

Possibili interpolatori:

- cubic Hermite;
- Catmull-Rom;
- cubic B-spline;
- interpolazione specifica vincolata ai joint.

La curva non deve alterare significativamente la posizione fisica dell'organo.

L'obiettivo è solamente eliminare i kink visivi tra segmenti.

---

# 6. Campionamento longitudinale

La centerline viene campionata in una sequenza:

\[
C(s_0), C(s_1), \dots, C(s_m)
\]

Esempio:

```text
●--●--●--●--●--●--●--●
```

Ogni campione genererà una sezione trasversale.

Non è necessario utilizzare un numero elevato di campioni.

Una prima configurazione ragionevole può essere:

```text
2-4 intervalli visuali per rigid link
```

con eventuali campioni extra vicino agli attachment.

---

# 7. Frame locale lungo la centerline

In ogni punto campionato si costruisce un riferimento locale:

\[
T(s), N(s), B(s)
\]

dove:

- \(T\) = tangent;
- \(N\) = primo asse trasversale;
- \(B\) = secondo asse trasversale.

```text
             N
             ↑
             │
             ●────→ B
            /
           /
          T
```

È consigliato utilizzare un **parallel transport frame** piuttosto che il Frenet frame puro.

Questo riduce il rischio di rotazioni improvvise della sezione nei tratti quasi rettilinei.

---

# 8. Generazione della sezione

La forma più semplice sarebbe un cerchio:

\[
P(s,\theta)=
C(s)
+r(s)
[
N(s)\cos\theta+B(s)\sin\theta
]
\]

con:

\[
0\le\theta<2\pi
\]

e ad esempio 8-12 vertici per ring.

```text
       • •
    •       •
   •    +    •
    •       •
       • •

+ = centerline
```

Il risultato base è uno swept generalized cylinder.

---

# 9. Sezione organica

Gli steli reali non sono matematicamente circolari.

La superficie proposta utilizza quindi:

\[
r(s,\theta)
\]

anziché un semplice \(r(s)\).

Una formulazione possibile è:

\[
r(s,\theta)
=
r_0(s)
[
1+
A_2(s)\cos(2\theta+\phi_2(s))
+
A_3(s)\cos(3\theta+\phi_3(s))
]
\]

dove:

- \(A_2\) introduce una leggera ellitticità;
- \(A_3\) rompe ulteriormente la simmetria;
- le ampiezze devono rimanere piccole.

Esempio concettuale:

```text
Cerchio perfetto        Sezione organica

       ○                      ╭─╮
                             │   ╲
                              ╲  │
                               ╰─╯
```

L'effetto deve essere sottile.

Non si vuole ottenere una superficie rugosa o nodosa, ma semplicemente evitare il comportamento visivo di un cilindro CAD perfetto.

---

# 10. Taper longitudinale

Ogni organo deve poter diminuire progressivamente di diametro.

Una formulazione semplice:

\[
r_\text{taper}(s)
=
r_\text{base}
+
(r_\text{tip}-r_\text{base})h(s)
\]

dove \(h(s)\) è una funzione smooth da 0 a 1.

Per esempio:

```text
CURRENT

████████████████████████████


PROPOSED

██████████████████████████
 ███████████████████████
  █████████████████████
   ███████████████████
```

I valori iniziali possono essere espressi relativamente al raggio nominale fornito dal modello.

Possibile punto di partenza sperimentale:

```text
base radius multiplier ≈ 1.05 – 1.15
tip radius multiplier  ≈ 0.85 – 0.95
```

Questi valori devono essere considerati **parametri visuali iniziali da calibrare**, non valori botanici definitivi.

---

# 11. Attachment flare del ramo figlio

Uno degli elementi visivi più importanti è la zona di connessione parent-child.

La geometria attuale assomiglia concettualmente a:

```text
                child
                 ││
                 ││
─────────────────┼────────────
parent
```

La geometria desiderata è:

```text
                    ╭────── child
                  ╭─╯
                 ╱
────────────────╯
parent
```

Il ramo figlio parte quindi con un raggio maggiore e converge progressivamente verso il proprio raggio nominale.

Si può utilizzare:

\[
r_\text{child}(s)
=
r_\text{nominal}(s)
[
1+
A_f
e^{-(s/\sigma_f)^2}
]
\]

dove:

- \(A_f\) controlla l'intensità del flare;
- \(\sigma_f\) controlla la lunghezza della zona di transizione.

---

# 12. Parametrizzazione relativa al diametro

È preferibile evitare di esprimere la lunghezza del flare in metri.

Meglio utilizzare:

\[
L_\text{flare}=k_fD
\]

dove:

- \(D\) è il diametro locale del ramo;
- \(k_f\) è un parametro adimensionale.

Per esempio:

```text
flare length = 1.5-3 × local diameter
```

Questo permette allo stesso algoritmo di funzionare automaticamente su:

- trunk;
- lateral branch;
- petiole;
- rachis;
- truss;
- pedicel.

---

# 13. Node swelling sul parent

Anche il parent può essere leggermente più largo vicino all'attachment.

Si introduce:

\[
r'_\text{parent}(s)
=
r_\text{parent}(s)
\left[
1+
A_n
e^{-
((s-s_a)/\sigma_n)^2
}
\right]
\]

dove \(s_a\) è la posizione dell'attachment.

Visivamente:

```text
WITHOUT

──────────────┬────────
              ╲
               ╲──────


WITH NODE SHAPING

────────────╮
            ╰╮
             ╰────────
```

Questo riduce ulteriormente l'aspetto di due cilindri intersecanti.

---

# 14. Prima strategia per le biforcazioni

Per il primo prototipo non è necessario generare una mesh topologicamente unica tra parent e child.

È sufficiente creare:

```text
ParentVisualMesh
ChildVisualMesh
```

con un leggero overlap.

Il child viene fatto iniziare leggermente all'interno del volume del parent.

Grazie a:

- parent node swelling;
- child root flare;
- smooth normals;
- materiale comune;

la giunzione dovrebbe diventare poco visibile.

Questa strategia è significativamente più semplice della generazione di una vera junction mesh.

---

# 15. Possibile evoluzione: junction mesh reale

Solo se necessario, una fase successiva potrebbe generare una superficie realmente connessa:

```text
             child rings
                 ◯
                /
               ◯
              /
parent  ◯───◯───◯
```

Le alternative includono:

- explicit triangulated patch;
- local remeshing;
- implicit/SDF smooth union;
- voxelization + surface extraction.

Questa non è necessaria per il primo esperimento.

---

# 16. Randomness deterministica

La variazione casuale deve essere:

- piccola;
- smooth;
- riproducibile.

Non deve essere applicato rumore indipendente a ogni vertice.

Si propone invece un seed derivato dall'ID dell'organo:

```python
seed = stable_hash(branch_id)
```

e la generazione di parametri leggermente diversi.

Esempio:

```text
Branch A
ellipticity = 0.026
flare       = 0.18
taper       = 0.91

Branch B
ellipticity = 0.037
flare       = 0.15
taper       = 0.94

Branch C
ellipticity = 0.021
flare       = 0.20
taper       = 0.89
```

Questo elimina la ripetitività senza rendere la geometria casuale ad ogni export.

---

# 17. Variazione longitudinale smooth

È possibile aggiungere piccole variazioni al raggio:

\[
r(s)
=
r_\text{baseModel}(s)
[
1+\epsilon n(s)
]
\]

dove \(n(s)\) è low-frequency noise.

Una soluzione ancora più controllabile consiste nel generare pochi control point casuali:

```text
s0      s1      s2      s3      s4
●-------●-------●-------●-------●
r0      r1      r2      r3      r4
```

e interpolarli mediante cubic spline.

Questo produce:

```text
r
│       ╭──╮
│____╭──╯  ╰────╮____
│               ╰──
└──────────────────── s
```

anziché rumore ad alta frequenza.

---

# 18. Rotazione lenta della sezione

Se la sezione è leggermente ellittica, mantenere sempre la stessa orientazione può produrre un pattern troppo regolare.

È possibile utilizzare:

\[
\phi(s)=\phi_0+\omega s
\]

con una \(\omega\) molto piccola.

Il ramo presenta quindi una lentissima variazione della sezione.

```text
ring 1    ring 2    ring 3    ring 4

  ◯         ◯         ◯         ◯
  0°        5°        11°       16°
```

La torsione deve essere praticamente impercettibile geometricamente ma sufficiente a variare highlights e silhouette.

---

# 19. Mesh generation

Dati due ring consecutivi:

```text
Ring i                 Ring i+1

A0 -------------------- B0
| \                     |
|  \                    |
A1 -------------------- B1
```

si generano triangoli:

```text
A0, B0, B1
A0, B1, A1
```

ripetuti per tutti i vertici radiali.

Con:

```text
radial_segments = 10
```

ogni intervallo longitudinale produce circa 20 triangoli.

La mesh rimane quindi relativamente leggera.

---

# 20. Smooth normals

Le normali della visual mesh devono essere smooth.

Questo consente a una mesh con un numero relativamente ridotto di radial segments di apparire molto più tonda.

Non è necessario utilizzare 32-64 vertici per ring.

Una prima sperimentazione dovrebbe confrontare:

```text
8 segments
10 segments
12 segments
```

considerando qualità e costo.

---

# 21. Skinning: principio

La visual mesh deve essere **continua attraverso i confini dei rigid link**.

Esempio:

```text
PHYSICS

[Link 1] -- [Link 2] -- [Link 3]


VISUAL

═════════════════════════════════
```

Ogni rigid link viene associato a un bone visuale.

I vertici della mesh vengono influenzati dai bone tramite `jointIndices` e `jointWeights`.

OpenUSD definisce espressamente questi primvar per controllare le influenze per punto e supporta skinning LBS e Dual Quaternion.

---

# 22. Perché non si vedranno separazioni tra i link

Consideriamo un D6 joint tra Bone A e Bone B.

Se un vertice è lontano dal joint:

```text
Bone A influence = 1.0
Bone B influence = 0.0
```

Vicino al joint:

```text
0.8 / 0.2
0.6 / 0.4
0.5 / 0.5
0.4 / 0.6
0.2 / 0.8
```

poi:

```text
Bone A influence = 0.0
Bone B influence = 1.0
```

Questa zona crea una deformazione continua.

Concettualmente:

```text
PHYSICS

========|========
        ^
       D6


SKIN WEIGHTS

Bone A      1 ─────────╲
                       ╲
                        ╲──────── 0

Bone B      0 ─────────╱
                       ╱
                      ╱────────── 1
```

Quando i due rigid link ruotano, la mesh cambia forma gradualmente invece di separarsi.

---

# 23. Blend zone

La larghezza della zona di skinning deve essere parametrica.

Possibile definizione:

\[
L_\text{blend}=k_bD
\]

oppure come frazione della lunghezza del rigid link.

Per esempio:

```text
0.2 – 0.5 × link length
```

come range iniziale da testare.

Una zona troppo piccola produce:

```text
══════╮
      ╰══════
```

una piega molto netta.

Una zona adeguata produce:

```text
══════╮
      ╰╮
       ╰══════
```

Una zona eccessiva può invece rendere il ramo troppo "gommoso" visivamente.

Il parametro va quindi tarato rispetto alla discretizzazione fisica.

---

# 24. LBS vs Dual Quaternion

La prima implementazione può utilizzare Linear Blend Skinning.

OpenUSD supporta anche Dual Quaternion Skinning.

DQS è interessante da confrontare perché può preservare meglio il volume in presenza di rotazioni significative.

La proposta è:

```text
baseline: LBS
experiment: DQS
```

e confrontare:

- conservazione del diametro;
- artefatti nella zona dei joint;
- torsione;
- costo;
- comportamento in Isaac Sim.

---

# 25. Collegamento tra PhysX e visual skeleton

La fisica rimane source of truth.

Pipeline runtime:

```text
PhysX rigid link poses
        │
        ▼
read link transforms
        │
        ▼
convert to skeleton transforms
        │
        ▼
UsdSkel animation/bone transforms
        │
        ▼
skinned visual mesh
```

Non si modifica il funzionamento di:

- D6 joints;
- articolazione;
- stiffness;
- damping;
- collisioni.

Il visual skeleton segue soltanto le pose ottenute dalla simulazione.

---

# 26. Struttura USD concettuale

Una possibile struttura è:

```text
/World
├── Physics
│   └── Stem
│       ├── Link_01
│       ├── Link_02
│       ├── Link_03
│       └── ...
│
└── Visual
    └── Plant
        ├── Skeleton
        └── StemMesh
```

Oppure si può mantenere la gerarchia fisica attuale e aggiungere la sezione visuale sotto un prim dedicato.

L'importante è che la mesh visuale **non possieda CollisionAPI né MassAPI**.

---

# 27. Organ profiles

Lo stesso generatore può essere utilizzato per tutti gli organi.

## Trunk

Caratteristiche:

- taper moderato;
- sezione leggermente organica;
- node swelling visibile;
- flare degli attachment evidente;
- variazione radiale leggera.

## Lateral branches

Caratteristiche:

- taper leggermente maggiore;
- root flare evidente;
- sezione quasi circolare;
- piccola variazione deterministica.

## Petiole

Caratteristiche:

- taper moderato;
- flare ridotto;
- sezione relativamente regolare;
- node swelling leggero.

## Rachis

Caratteristiche:

- diametro progressivamente decrescente;
- flare basso;
- sezione quasi circolare.

## Truss

Caratteristiche:

- taper significativo;
- sezione sottile;
- flare locale moderato.

## Pedicel

Caratteristiche:

- taper;
- flare limitato;
- geometria molto leggera.

---

# 28. Configurazione proposta

Esempio concettuale:

```python
@dataclass
class VisualBranchProfile:
    radial_segments: int = 10
    samples_per_link: int = 3

    taper_base_multiplier: float = 1.08
    taper_tip_multiplier: float = 0.92

    ellipticity: float = 0.025
    third_order_lobing: float = 0.01

    attachment_flare_strength: float = 0.18
    attachment_flare_length_diameters: float = 2.0

    parent_node_swelling: float = 0.08
    parent_node_length_diameters: float = 1.5

    longitudinal_variation: float = 0.015

    section_twist_rate: float = 0.0

    skin_blend_fraction: float = 0.30
```

I valori sono starting point visuali da calibrare.

---

# 29. Randomization ranges

Potrebbe essere utile definire:

```python
@dataclass
class VisualVariationConfig:
    taper_jitter: float = 0.03
    ellipticity_jitter: float = 0.01
    flare_jitter: float = 0.03
    node_swelling_jitter: float = 0.02
    section_phase_randomization: bool = True
```

Per ogni organo:

```python
rng = Random(stable_hash(branch_id))
```

e i parametri vengono generati una sola volta in maniera deterministica.

---

# 30. Moduli software suggeriti

Possibile struttura:

```text
src/exporterV2/core/visual/

centerline.py
frames.py
radius_profile.py
sweep_mesh.py
junctions.py
skinning.py
profiles.py
randomization.py
```

Responsabilità:

### `centerline.py`

```python
extract_branch_centerline(...)
smooth_centerline(...)
sample_centerline(...)
```

### `frames.py`

```python
compute_parallel_transport_frames(...)
```

### `radius_profile.py`

```python
compute_taper(...)
compute_cross_section(...)
compute_child_flare(...)
compute_parent_swelling(...)
compute_longitudinal_variation(...)
```

### `sweep_mesh.py`

```python
generate_ring(...)
connect_rings(...)
generate_branch_mesh(...)
```

### `skinning.py`

```python
compute_joint_weights(...)
build_usd_skeleton(...)
bind_mesh_to_skeleton(...)
update_skeleton_from_physics(...)
```

### `profiles.py`

```python
TRUNK_VISUAL_PROFILE
LATERAL_VISUAL_PROFILE
PETIOLE_VISUAL_PROFILE
RACHIS_VISUAL_PROFILE
TRUSS_VISUAL_PROFILE
PEDICEL_VISUAL_PROFILE
```

---

# 31. Primo prototipo consigliato

Non implementare subito la pianta intera.

Utilizzare un singolo ramo con almeno 3 rigid link.

## Phase A — geometry only

Implementare:

```text
physical centerline
→ sampled centerline
→ parallel transport frames
→ circular sweep
```

Risultato atteso:

```text
rigid segmented cylinders
        ↓
continuous visual tube
```

Nessun taper e nessun randomness.

---

# 32. Phase B — taper

Aggiungere solamente il profilo longitudinale:

```text
Sweep
+
Taper
```

Validare:

- continuità;
- diametro;
- rapporto base/tip;
- orientamento.

---

# 33. Phase C — organic cross-section

Aggiungere:

```text
ellipticity
+
small third-order variation
+
slow section rotation
```

Confrontare visivamente:

```text
perfect cylinder
vs
organic generalized cylinder
```

---

# 34. Phase D — parent-child attachment

Costruire un test:

```text
       child
      /
parent
```

e implementare progressivamente:

```text
A. plain overlap
B. child root flare
C. parent node swelling
D. B + C
```

Il confronto A/B/C/D permetterà di identificare quanto ogni componente contribuisce al realismo.

---

# 35. Phase E — skinning

Creare:

```text
one continuous mesh
+
one bone per physical rigid link
```

Assegnare inizialmente due influenze massime per vertice:

```text
bone i
bone i+1
```

La posizione longitudinale \(s\) permette di calcolare i pesi automaticamente.

---

# 36. Test fondamentale dello skinning

Configurazione:

```text
3 rigid links
2 D6 joints
1 continuous visual mesh
```

Applicare progressivamente:

```text
0°
5°
10°
20°
30°
```

a ogni joint.

Osservare:

- presenza di crease;
- perdita di volume;
- stretching;
- twist;
- separazioni;
- intersezione con collision proxy.

Confrontare LBS e DQS.

---

# 37. Phase F — full plant

Solo dopo la validazione del ramo singolo:

```text
trunk
→ lateral branches
→ petioles
→ rachis
→ truss
→ pedicels
```

La stessa pipeline deve lavorare su ogni organo senza modifiche strutturali.

---

# 38. Ablation study consigliato

Per documentazione tecnica o tesi è utile salvare immagini nelle seguenti condizioni:

```text
A0 current cylinders

A1 continuous circular sweep

A2 + taper

A3 + organic cross-section

A4 + child root flare

A5 + parent node swelling

A6 + deterministic variation

A7 + skinning under physical deformation
```

Questo permette di mostrare precisamente quale modifica produce ogni miglioramento visivo.

---

# 39. Metriche possibili

Oltre alla valutazione qualitativa:

## Geometry complexity

```text
vertex count
triangle count
USD size
```

## Runtime

```text
physics FPS
render FPS
skinning update cost
```

## Physical invariance

Confrontare prima/dopo:

```text
link mass
center of mass
joint parameters
collision geometry
cantilever response
```

Poiché la visual mesh non partecipa alla fisica, queste quantità dovrebbero rimanere invariate.

## Visual continuity

Misurare eventualmente:

- distanza tra ring adiacenti;
- variazione di normale presso joint;
- diametro sotto bending;
- curvature discontinuity.

---

# 40. Non-obiettivi

Questa fase **non comprende**:

- nuove foglie;
- micro-piccioli;
- peli;
- trichomes;
- texture synthesis;
- tomato reconstruction;
- calyx;
- NeRF;
- Gaussian Splatting;
- diffusion models.

Foglie e pomodori devono essere trattati separatamente.

Questa fase riguarda esclusivamente la geometria visiva di:

```text
stem
branches
petioles
rachis
truss
pedicels
```

---

# 41. Criterio di successo

Il prototipo è considerato riuscito se:

1. la topologia botanica rimane invariata;
2. la simulazione fisica rimane invariata;
3. non sono visibili gap tra rigid link dello stesso organo;
4. gli organi appaiono come superfici continue;
5. il diametro diminuisce progressivamente;
6. gli attachment parent-child hanno transizioni smooth;
7. sezioni e profili presentano variazioni leggere ma non artificiali;
8. organi differenti non sembrano copie geometriche perfette;
9. il costo geometrico rimane contenuto;
10. la mesh segue correttamente le deformazioni prodotte dai D6 joint.

---

# 42. Target finale

La trasformazione desiderata può essere riassunta come:

```text
CURRENT PHYSICS + VISUAL

[CYL]--[CYL]--[CYL]
          |
         [CYL]


TARGET

Physics:
[CYL]--[CYL]--[CYL]
          |
         [CYL]

Visual:
        ╭────────────────────
───────╯
         ╲
          ╰────────────
```

La fisica continua a vedere una struttura articolata composta da rigid body.

Il renderer vede invece **un unico oggetto organico, continuo e deformabile**.

Questa separazione tra struttura fisica discreta e superficie visuale continua costituisce il principio centrale dell'intera proposta.