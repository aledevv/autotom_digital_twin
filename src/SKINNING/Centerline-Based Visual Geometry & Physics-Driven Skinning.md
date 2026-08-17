# Piano aggiornato — Centerline-Based Visual Geometry & Physics-Driven Skinning

## 1. Obiettivo

L'obiettivo è migliorare il realismo geometrico di:

- stem principale;
- lateral branches;
- petiole;
- rachis;
- truss;
- pedicel;

senza modificare il modello botanico né la simulazione fisica già esistente.

In questa fase **non** vengono affrontati:

- foglie;
- pomodori;
- texture;
- peli/trichomes;
- microrametti;
- nuovi organi;
- NeRF;
- Gaussian Splatting;
- diffusion models.

L'idea centrale è costruire una **mesh visuale continua** attorno alle centerline degli organi già presenti e deformarla attraverso uno skeleton visuale che segue le pose dei rigid link PhysX.

---

# 2. Principio architetturale

La simulazione fisica rimane la source of truth.

```text
PHYSICS

[Link 0] --D6-- [Link 1] --D6-- [Link 2]

               ↓ pose


VISUAL

        Bone0 ---- Bone1 ---- Bone2
                    ↓
          continuous skin mesh
```

La parte fisica continua a utilizzare:

- rigid body;
- D6 joints;
- FixedJoint;
- collision proxy;
- masse;
- center of mass;
- stiffness;
- damping.

La parte visuale utilizza:

- centerline;
- mesh procedurale;
- skeleton;
- skinning;
- smooth normals.

La visual mesh non deve possedere:

```text
CollisionAPI
MassAPI
RigidBodyAPI
```

e quindi non deve alterare la dinamica della pianta.

---

# 3. Risultato desiderato

Situazione fisica:

```text
[link 1] -- [link 2] -- [link 3]
```

Visuale attuale:

```text
████████  ████████  ████████
```

Visuale desiderato:

```text
╭──────────────────────────────╮
╰──────────────────────────────╯
```

Quando i D6 si piegano:

```text
PHYSICS

             [3]
            /
         [2]
        /
[1]
```

la visual mesh deve produrre:

```text
                ╭══════
             ╭══╯
          ╭══╯
══════════╯
```

e **non**:

```text
══════   ══════   ══════
```

La separazione tra rigid segment deve quindi rimanere completamente invisibile.

---

# 4. Perché procedere con uno spike isolato

Prima di modificare la pianta reale dell'`exporterV2`, bisogna verificare il punto tecnologicamente più rischioso:

> È possibile pilotare in maniera affidabile uno skeleton skinnato durante il runtime di Isaac Sim usando le pose prodotte da PhysX?

La parte geometrica:

```text
centerline
→ frames
→ rings
→ triangoli
→ taper
→ flare
```

è relativamente deterministica.

Il bridge:

```text
PhysX → UsdSkel → skinning runtime
```

deve invece essere verificato sperimentalmente prima di investire tempo nell'integrazione completa.

Questa raccomandazione deriva anche dalla validazione tecnica esterna del piano.

---

# 5. Nuovo ordine di sviluppo

Il piano aggiornato diventa:

```text
SPIKE 0
UsdSkel + PhysX feasibility
        │
        ▼
PHASE 1
Centerline + basic sweep
        │
        ▼
PHASE 2
Taper
        │
        ▼
PHASE 3
Organic cross-section
+ controlled randomness
        │
        ▼
PHASE 4
Parent-child attachment
flare + swelling
        │
        ▼
PHASE 5
Full skinning integration
        │
        ▼
PHASE 6
Integration in exporterV2
        │
        ▼
PHASE 7
Full plant visual validation
```

La differenza fondamentale rispetto al piano precedente è che **UsdSkel viene testato immediatamente**.

---

# 6. SPIKE 0 — Esperimento isolato

Lo spike non deve utilizzare la pianta.

Deve vivere in una cartella completamente separata, ad esempio:

```text
src/experiments/
└── usdskel_physx_bridge/
    ├── README.md
    ├── test_00_static_skin.py
    ├── test_01_runtime_skin.py
    ├── test_02_physx_bridge.py
    └── output/
```

Non devono essere modificati inizialmente:

```text
src/exporterV2/core/usd/stage.py
src/exporterV2/core/usd/geometry.py
src/exporterV2/core/tree_config.py
```

---

# 7. Test 0A — UsdSkel statico

## Obiettivo

Dimostrare che Isaac Sim visualizza correttamente una singola mesh continua controllata da tre bones.

La scena contiene:

```text
SkelRoot
├── Skeleton
│   ├── Bone0
│   ├── Bone1
│   └── Bone2
│
├── SkelAnimation
│
└── TubeMesh
```

La mesh può essere inizialmente un tubo estremamente semplice.

```text
══════════════════════════════
```

Nessuna fisica.

Nessun D6.

Nessun algoritmo centerline avanzato.

I bone vengono messi manualmente in una configurazione tipo:

```text
Bone0 = 0°
Bone1 = 25°
Bone2 = 45°
```

Risultato atteso:

```text
                ╭══════
            ╭═══╯
════════════╯
```

## Success criterion

La mesh:

- viene correttamente skinnata;
- non presenta gap;
- non presenta trasformazioni iniziali inattese;
- mantiene un binding stabile.

---

# 8. Test 0B — UsdSkel runtime

## Obiettivo

Verificare che le pose dello skeleton possano essere modificate durante l'esecuzione.

Per esempio:

\[
\theta(t)=30^\circ \sin(\omega t)
\]

La mesh deve quindi oscillare continuamente.

```text
t0

══════════════════


t1

════════════╮
            ╰══════


t2

══════════╮
          ╰╮
           ╰══════
```

Questo test serve in particolare a verificare che:

- gli attributi dello skeleton non vengano sovrascritti;
- l'evaluator di animazione non entri in conflitto con gli aggiornamenti;
- Hydra aggiorni correttamente la mesh;
- il comportamento sia stabile in Isaac Sim 4.5.

---

# 9. Test 0C — PhysX → Skeleton

Solo dopo il successo di 0A e 0B viene introdotta la fisica.

Configurazione:

```text
World
 │
 ●──────●──────●
 Link0  Link1  Link2
        D6     D6
```

Possibile test:

```text
Link0 = fixed to world
Link1 = dynamic
Link2 = dynamic
gravity enabled
```

Sopra i rigid link viene posta una singola mesh skinnata.

Pipeline runtime:

```text
PhysX link transforms
        │
        ▼
read poses
        │
        ▼
physics_to_skeleton()
        │
        ▼
UsdSkelAnimation
        │
        ▼
continuous visual mesh
```

Questo è il vero **GO / NO-GO test** dell'intero progetto.

---

# 10. Bone frame

Non bisogna assumere che:

```text
physical rigid-body origin
=
visual bone origin
```

Per ogni rigid link deve essere definito un offset costante:

\[
O_i=T_{\text{link}\rightarrow\text{bone}}
\]

Durante il runtime:

\[
W^{bone}_i(t)
=
W^{link}_i(t)O_i
\]

dove:

- \(W^{link}_i(t)\) = trasformazione world del rigid body;
- \(O_i\) = trasformazione costante link→bone.

Per ottenere il transform locale del bone:

\[
L_i(t)=
(W^{bone}_{parent}(t))^{-1}
W^{bone}_i(t)
\]

Per il root:

\[
L_0(t)=
(W_{SkelRoot})^{-1}
W^{bone}_0(t)
\]

Questo mapping deve essere verificato già nello Spike 0.

Il criterio fondamentale è:

> premendo Play, la visual mesh non deve compiere alcun salto rispetto alla rest pose.

---

# 11. Update rate

La simulazione fisica può funzionare a una frequenza molto più alta del rendering.

Non è necessario aggiornare lo skeleton a ogni substep PhysX.

Pipeline consigliata:

```text
PHYSICS

│││││││││││││││││││││


VISUAL

│       │       │       │
```

Lo skeleton viene aggiornato una volta per render frame utilizzando la più recente posa fisica disponibile.

Questo riduce drasticamente il costo del bridge Python/USD.

---

# 12. Aggiornamenti batch

Non aggiornare ogni bone con una chiamata USD indipendente.

Evitare:

```python
for bone in bones:
    update_transform(bone)
```

Preferire:

```python
translations = [...]
rotations = [...]

animation.GetTranslationsAttr().Set(translations)
animation.GetRotationsAttr().Set(rotations)
```

in modo da aggiornare l'intero skeleton in batch.

---

# 13. LBS vs Dual Quaternion Skinning

Lo Spike 0 deve includere anche il confronto:

```text
classicLinear
vs
dualQuaternion
```

Lo stesso rig deve essere testato con bending progressivo:

```text
0°
15°
30°
45°
60°
```

Osservare:

- pinching;
- perdita di volume;
- twist;
- continuità;
- forma della sezione.

## Decisione

Non scegliere LBS o DQS a priori.

La scelta deve essere fatta sperimentalmente.

DQS è il candidato naturale nel caso in cui LBS mostri eccessiva perdita di volume in corrispondenza dei D6.

Corrective bones o intermediate bones devono rimanere fuori dallo scope iniziale.

---

# 14. Success criteria dello Spike 0

Prima di procedere alla geometria botanica devono essere soddisfatti:

- visual mesh unica e continua;
- nessun gap;
- binding corretto;
- runtime update dello skeleton funzionante;
- pose PhysX correttamente trasferite ai bones;
- zero salto quando parte la simulazione;
- zero offset iniziale;
- assenza di twist inattesi;
- bending di almeno 30-60° gestibile;
- confronto LBS/DQS eseguito;
- visual update disaccoppiato dai substep fisici;
- nessuna modifica alla simulazione dovuta alla visual mesh.

Solo dopo:

```text
SPIKE 0 = PASS
```

si procede.

---

# 15. PHASE 1 — Centerline extraction

Per ogni organo si estrae una centerline:

\[
C(s)
\]

dalla configurazione fisica.

Esempio:

```text
C0 ●──────● C1──────● C2──────● C3
```

La centerline rappresenta l'asse geometrico dell'organo.

Ogni punto lungo la centerline utilizza una coordinata longitudinale:

\[
s \in [0,L]
\]

Questa coordinata deve diventare una delle informazioni fondamentali associate ai vertici.

---

# 16. Attachment position come coordinata parametrica

Gli attachment non devono essere rappresentati internamente soltanto come:

```text
attach_link = 3
```

La visual pipeline deve convertirli in una posizione parametrica:

\[
s_a
\]

sulla centerline del parent.

Esempio:

```text
s = 0                          s = L
●──────────────────────────────●
                   ↑
                  s_a
```

Questo permette di calcolare indipendentemente dalla discretizzazione fisica:

- node swelling;
- punto iniziale del child;
- frame del child;
- flare.

---

# 17. Centerline visuale

La centerline fisica può essere piecewise linear.

```text
──────●
       \
        \
         ●──────
```

La visual centerline deve rimuovere i kink troppo evidenti.

```text
──────╮
       ╲
        ╰──────
```

La smussatura deve essere limitata.

Non deve modificare significativamente:

- lunghezza dell'organo;
- posizione degli attachment;
- relazione con i rigid links.

Possibili interpolatori:

- cubic Hermite;
- Catmull-Rom;
- B-spline constrained.

Per il primo prototipo è sufficiente anche una semplice interpolazione locale.

---

# 18. PHASE 2 — Parallel transport frames

Per ogni campione della centerline viene calcolato:

\[
T(s),N(s),B(s)
\]

con:

- \(T\) tangent;
- \(N\) primo asse trasversale;
- \(B\) secondo asse trasversale.

```text
          N
          ↑
          │
          ●────→ B
         /
        /
       T
```

Utilizzare un **parallel transport frame**.

Evitare il Frenet frame puro nei tratti quasi rettilinei.

---

# 19. Continuità parent → child

Quando nasce un ramo figlio, il suo frame non deve essere inizializzato indipendentemente.

```text
Parent PTF
    │
    ▼
frame at attachment s_a
    │
    ▼
rotate frame toward child tangent
    │
    ▼
Child PTF
```

Questo evita un phase shift arbitrario tra le sezioni.

La regola deve essere:

> il child eredita l'orientamento trasversale del parent nel punto di attachment.

Questo diventa particolarmente importante con sezioni non perfettamente circolari.

---

# 20. PHASE 3 — Sampling adattivo

Non utilizzare un numero fisso di campioni per rigid link come unico criterio.

Definire invece una distanza target:

\[
\Delta s_{target}
\]

e:

\[
N =
\max
\left(
N_{min},
\left\lceil\frac{L}{\Delta s_{target}}\right\rceil
\right)
\]

Possibile strategia:

\[
\Delta s_{target}=kD
\]

dove \(D\) è il diametro locale.

Questo permette di adattare automaticamente la densità a:

- trunk;
- laterals;
- petiole;
- rachis;
- truss;
- pedicel.

Aggiungere campioni extra:

- vicino ai D6;
- vicino agli attachment;
- nella zona di flare;
- dove la centerline curva molto.

---

# 21. Basic sweep mesh

In ogni campione viene creato un ring.

\[
P(s,\theta)=
C(s)+
r(s)
[N(s)\cos\theta+B(s)\sin\theta]
\]

con:

\[
0\leq \theta<2\pi
\]

Possibile valore iniziale:

```text
radial_segments = 8-12
```

Esempio:

```text
       • •
    •       •
   •    +    •
    •       •
       • •

+ = centerline
```

Gli anelli vengono connessi attraverso triangoli.

---

# 22. Smooth normals

Le normali devono essere esplicitamente smooth.

La mesh deve produrre visualmente:

```text
○
```

e non:

```text
⬡
```

anche con un numero relativamente piccolo di radial segments.

La gestione corretta delle normali deve far parte di `sweep_mesh.py`, non essere delegata implicitamente al renderer.

---

# 23. PHASE 4 — Taper longitudinale

Ogni organo deve poter restringersi progressivamente.

\[
r_{taper}(s)=
r_{base}+
(r_{tip}-r_{base})h(s)
\]

con \(h(s)\) smooth.

Visualmente:

```text
CURRENT

████████████████████████


TARGET

████████████████████████
 █████████████████████
  ███████████████████
```

Starting point sperimentale:

```text
base multiplier ≈ 1.05 - 1.15
tip multiplier  ≈ 0.85 - 0.95
```

Questi valori sono parametri visuali iniziali, non misure botaniche definitive.

---

# 24. PHASE 5 — Organic cross-section

La sezione non deve essere perfettamente circolare.

Utilizzare:

\[
r(s,\theta)
=
r_0(s)
[
1+
A_2(s)\cos(2\theta+\phi_2)
+
A_3(s)\cos(3\theta+\phi_3)
]
\]

dove:

- \(A_2\) controlla una lieve ellitticità;
- \(A_3\) introduce una leggera asimmetria.

Le ampiezze devono essere piccole.

Target:

```text
perfect cylinder       tomato-like organic section

       ○                         ◯
```

Non:

```text
highly irregular / rough geometry
```

---

# 25. Slow section rotation

È possibile ruotare molto lentamente la sezione lungo l'asse:

\[
\phi(s)=\phi_0+\omega s
\]

con \(\omega\) molto piccolo.

Questo evita sezioni ellittiche perfettamente allineate lungo tutto il ramo e rende le superfici meno artificiali.

---

# 26. Longitudinal variation

Aggiungere leggere variazioni a bassa frequenza:

\[
r(s)
=
r_{baseModel}(s)
[
1+\epsilon n(s)
]
\]

Non utilizzare noise indipendente per vertice.

Meglio generare pochi control point:

```text
s0      s1      s2      s3
●-------●-------●-------●
```

con variazioni piccole di raggio e interpolarli con una spline.

---

# 27. Randomness deterministica

Ogni organo deve avere parametri leggermente diversi, ma la generazione deve essere riproducibile.

Utilizzare:

```python
rng = Random(stable_hash(branch_id))
```

Variabili randomizzabili:

```text
taper
ellipticity
cross-section phase
longitudinal variation
flare strength
flare length
node swelling
```

La randomizzazione deve essere molto contenuta.

Non deve cambiare l'identità morfologica dell'organo.

---

# 28. PHASE 6 — Child root flare

Un ramo figlio non deve partire direttamente con il proprio raggio nominale.

Situazione da evitare:

```text
             │
             │
─────────────┼────────
```

Target:

```text
               ╭──────
             ╭─╯
            ╱
───────────╯
```

Possibile funzione:

\[
r_{child}(s)=
r_{nominal}(s)
[
1+
A_f e^{-(s/\sigma_f)^2}
]
\]

dove:

- \(A_f\) = intensità del flare;
- \(\sigma_f\) = lunghezza della transizione.

---

# 29. Flare relativo al diametro

Definire:

\[
L_{flare}=k_fD
\]

e non una lunghezza assoluta in metri.

Questo rende il metodo generalizzabile a tutti gli organi.

Starting point:

```text
flare length ≈ 1.5-3 × local diameter
```

da calibrare visivamente.

---

# 30. Parent node swelling

Il parent può essere leggermente ingrossato vicino all'attachment:

\[
r'_{parent}(s)=
r_{parent}(s)
\left[
1+
A_n
e^{-((s-s_a)/\sigma_n)^2}
\right]
\]

Target:

```text
BEFORE

──────────────┬──────
              ╲
               ╲─────


AFTER

────────────╮
            ╰╮
             ╰────────
```

L'effetto deve essere visibile ma non eccessivo.

---

# 31. Strategia iniziale per la junction

La prima implementazione utilizza:

```text
ParentVisualMesh
+
ChildVisualMesh
```

con leggero overlap.

Non si genera inizialmente una vera mesh topologicamente unificata.

La junction viene nascosta mediante:

- child flare;
- parent swelling;
- stessa famiglia di materiale;
- smooth shading;
- leggero overlap geometrico.

Una eventuale junction mesh reale rimane una possibile evoluzione futura.

---

# 32. PHASE 7 — Skin weights

Ogni vertice conosce la propria coordinata longitudinale:

\[
s
\]

e quindi la sua posizione rispetto ai rigid links/bones.

Lontano da un joint:

```text
Bone A = 1
Bone B = 0
```

Avvicinandosi:

```text
0.8 / 0.2
0.6 / 0.4
0.5 / 0.5
0.4 / 0.6
0.2 / 0.8
```

poi:

```text
Bone A = 0
Bone B = 1
```

Questa transizione impedisce la separazione visiva tra segmenti.

---

# 33. Blend zone

Definire:

\[
L_{blend}=k_bD
\]

oppure una funzione della lunghezza del link.

La zona deve essere sufficientemente larga da evitare una cerniera visivamente troppo netta, ma non così larga da rendere il ramo gommoso.

Il parametro va calibrato sperimentalmente.

---

# 34. Metadata per vertex

Durante la generazione può essere utile mantenere concettualmente:

```text
branch_id
s
theta
```

per ogni vertice.

Da queste informazioni possono essere ricavati:

- posizione;
- skin weights;
- UV longitudinali;
- sezione;
- variazioni;
- debugging.

Questo rende la mesh procedurale molto più semplice da analizzare.

---

# 35. Organ profiles

Utilizzare un generatore unico con profili differenti.

## Stem

```text
taper                  moderato
ellipticity            moderata
root/node swelling     evidente
randomness             piccola
```

## Lateral branch

```text
taper                  moderato-alto
root flare             evidente
ellipticity            moderata
randomness             piccola
```

## Petiole

```text
taper                  moderato
flare                  basso-moderato
cross-section          quasi circolare
```

## Rachis

```text
taper                  significativo
flare                  basso
cross-section          quasi circolare
```

## Truss

```text
taper                  significativo
flare                  moderato
section irregularity   molto bassa
```

## Pedicel

```text
taper                  significativo
flare                  basso
mesh resolution        ridotta
```

---

# 36. Possibile configurazione

```python
@dataclass
class VisualBranchProfile:
    radial_segments: int = 10

    target_ring_spacing_diameters: float = 1.2
    minimum_rings_per_link: int = 2

    taper_base_multiplier: float = 1.08
    taper_tip_multiplier: float = 0.92

    ellipticity: float = 0.025
    third_order_lobing: float = 0.01

    longitudinal_variation: float = 0.015
    section_twist_rate: float = 0.0

    attachment_flare_strength: float = 0.18
    attachment_flare_length_diameters: float = 2.0

    parent_node_swelling: float = 0.08
    parent_node_length_diameters: float = 1.5

    skin_blend_length_diameters: float = 1.5
```

I valori costituiscono solamente starting points.

---

# 37. Struttura software proposta

Solo dopo il successo dello Spike 0:

```text
src/exporterV2/core/visual/
│
├── centerline.py
├── frames.py
├── sampling.py
├── radius_profile.py
├── sweep_mesh.py
├── junctions.py
├── skinning.py
├── runtime_bridge.py
├── profiles.py
└── randomization.py
```

## `centerline.py`

```python
extract_centerline(...)
attachment_to_arc_position(...)
smooth_centerline(...)
```

## `frames.py`

```python
compute_parallel_transport_frames(...)
inherit_child_frame(...)
```

## `sampling.py`

```python
compute_adaptive_samples(...)
add_joint_samples(...)
add_attachment_samples(...)
```

## `radius_profile.py`

```python
compute_taper(...)
compute_cross_section(...)
compute_longitudinal_variation(...)
compute_child_flare(...)
compute_parent_swelling(...)
```

## `sweep_mesh.py`

```python
generate_ring(...)
connect_rings(...)
compute_smooth_normals(...)
generate_visual_branch_mesh(...)
```

## `skinning.py`

```python
compute_joint_weights(...)
create_usd_skeleton(...)
bind_mesh(...)
```

## `runtime_bridge.py`

```python
read_physics_link_poses(...)
compute_bone_world_transforms(...)
compute_local_skeleton_transforms(...)
update_skeleton_batch(...)
```

---

# 38. Ablation study

Salvare immagini dei seguenti step:

```text
A0 current cylinders

A1 basic continuous sweep

A2 + taper

A3 + organic cross-section

A4 + deterministic variation

A5 + child root flare

A6 + parent node swelling

A7 + skinning

A8 LBS vs DQS
```

Questo permette di isolare il contributo visuale di ogni componente.

---

# 39. Performance tests

Misurare almeno:

```text
triangle count
vertex count
USD size
frame rate
visual update time
```

Confrontare:

```text
visual disabled
visual enabled static
visual enabled + runtime skinning
```

Durante esperimenti fisici headless o parameter identification:

```text
VisualConfig.ENABLED = False
```

---

# 40. Physical regression tests

L'aggiunta della visual mesh non deve cambiare:

- pose iniziale dei rigid links;
- masse;
- COM;
- joint configuration;
- stiffness;
- damping;
- break force;
- collision behavior;
- bending response.

Un test utile è confrontare la posizione dei rigid links:

```text
physics-only
vs
physics + visual
```

e verificare che coincidano entro la precisione numerica prevista.

Il problema della cosiddetta "Y-pose" citato nella validazione esterna non viene considerato un blocker di questo lavoro se non è presente nella versione attuale della pianta; viene invece mantenuto il requisito generale che il visual layer non introduca regressioni nella posa fisica.

---

# 41. Go / No-Go gates

## Gate 0 — Runtime skinning

Procedere solo se:

```text
PhysX → UsdSkel → visual mesh
```

funziona in maniera stabile.

## Gate 1 — Basic geometry

Procedere solo se il sweep produce una mesh:

- continua;
- correttamente orientata;
- senza twist;
- sufficientemente leggera.

## Gate 2 — Junction

Procedere solo se flare + swelling rendono l'attachment sufficientemente naturale senza richiedere remeshing complesso.

## Gate 3 — Full plant

Integrare nell'exporter solo dopo aver validato un'articolazione isolata e un singolo branch botanico.

---

# 42. Ordine pratico di implementazione

## Step 1

Creare:

```text
src/experiments/usdskel_physx_bridge/
```

## Step 2

Implementare un tubo skinnato statico con 3 bones.

## Step 3

Animare i bones a runtime.

## Step 4

Creare una articolazione PhysX a 3 rigid links + 2 D6.

## Step 5

Pilotare lo skeleton dalle pose PhysX.

## Step 6

Confrontare LBS/DQS.

## Step 7

Implementare un semplice centerline sweep.

## Step 8

Aggiungere taper.

## Step 9

Aggiungere organic cross-section.

## Step 10

Aggiungere controlled deterministic randomness.

## Step 11

Creare test parent-child isolato.

## Step 12

Aggiungere child flare.

## Step 13

Aggiungere parent swelling.

## Step 14

Verificare continuità del frame parent-child.

## Step 15

Applicare il generatore a un singolo ramo reale proveniente da Exporter V2.

## Step 16

Applicarlo a stem + un lateral branch.

## Step 17

Solo infine estenderlo a tutti gli organi della pianta.

---

# 43. Primo milestone

Il primo milestone realmente significativo non è:

> "abbiamo generato una pianta più bella."

È:

> **una articolazione PhysX discreta composta da tre rigid body e due D6 joint controlla correttamente una singola mesh continua tramite skinning in Isaac Sim 4.5.**

Se questo funziona, l'architettura fondamentale è validata.

---

# 44. Secondo milestone

Il secondo milestone è:

> **un parent branch e un child branch presentano taper, sezione organica, child flare e parent swelling, ottenendo una connessione visivamente simile a un vero internodo di pomodoro pur mantenendo due strutture fisiche separate.**

---

# 45. Terzo milestone

Il terzo milestone è:

> **la stessa pipeline viene applicata automaticamente a stem, lateral branches, petiole, rachis, truss e pedicel della pianta esistente senza modificare la topologia GroIMP né i parametri fisici.**

---

# 46. Target finale

Fisica:

```text
              [child links]
                   /
                  /
[parent]--[parent]--[parent]
```

Visuale:

```text
                     ╭──────────
                  ╭──╯
                ╭─╯
───────────────╯
```

Con:

- superficie continua;
- taper;
- sezione quasi circolare ma non perfetta;
- leggere variazioni deterministiche;
- attachment largo alla base;
- transizione smooth verso il ramo;
- nessuna separazione visibile tra rigid links;
- nessun aumento della complessità botanica della pianta;
- nessuna modifica della fisica.

---

# 47. Principio finale

L'intero sistema deve rispettare questa separazione:

```text
BOTANICAL MODEL
       │
       ▼
PHYSICAL SKELETON
       │
       ├──────────────► physics / collisions
       │
       ▼
VISUAL CENTERLINE
       │
       ▼
PROCEDURAL ENVELOPE
       │
       ▼
SKINNING
       │
       ▼
REALISTIC VISUAL STEMS
```

Il modello fisico rimane discreto.

La rappresentazione visiva diventa continua.

Questo permette di migliorare drasticamente il realismo dei rami senza sacrificare stabilità, riproducibilità o validità meccanica del digital twin.