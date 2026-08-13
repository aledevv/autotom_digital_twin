Sì. **Un minimo di movimento è normale** con quella architettura, perché appena fai:

```text
articulation link ── FixedJoint (excludeFromArticulation) ── tomato rigid body
```

il `FixedJoint` diventa un **maximal-coordinate constraint** risolto iterativamente. PhysX stesso specifica che un FixedJoint può avere un certo drift/errore, mentre i joint interni di una reduced-coordinate articulation hanno un comportamento molto più rigido dal punto di vista del vincolo. ([nvidia-omniverse.github.io][1])

La buona notizia è che puoi ridurlo parecchio.

### Prima cosa: capire che artifact stai vedendo

Ci sono due casi leggermente diversi.

Se fai forza e il pomodoro:

```text
forza
  ↓
pomodoro si sposta 1-2 mm
  ↓
rilasci
  ↓
torna quasi nella posizione iniziale
```

quello è soprattutto **constraint compliance / solver error**.

Se invece succede:

```text
inizialmente fermo
  ↓
lo tocchi
  ↓
fa un piccolo "pop" verso l'alto
  ↓
rimane nella nuova posizione
```

sospetterei prima **constraint frame non perfettamente coincidenti oppure una collisione/interpenetrazione**. OpenUSD raccomanda che i due joint frame coincidano nello spazio world; quando non coincidono, il solver deve correggere quell'errore. Inoltre è pratica normale disabilitare le collisioni fra corpi collegati proprio per evitare che contatto e joint combattano fra loro. ([OpenUSD][2])

## Le cose che farei, in questo ordine

**1. Controllare i joint frame. Questa per me è la priorità #1.**

Per il FixedJoint:

```text
PedicelTip
    localPos0 ─────┐
                   ├── stesso punto WORLD
Tomato             │
    localPos1 ─────┘
```

deve valere, a simulazione appena inizializzata:

```python
world_joint_frame_body0 ≈ world_joint_frame_body1
```

sia in posizione sia in orientamento.

Se c'è anche solo qualche mm di errore, il pomodoro può apparire perfettamente fermo finché dorme; quando lo tocchi e viene svegliato, PhysX inizia a correggere il constraint e tu percepisci il famoso piccolo “salto”.

**Questo test lo farei assolutamente.**

---

**2. Eliminare completamente collisioni locali attorno all'attacco.**

Il vecchio `PlantBuilder` faceva già qualcosa di intelligente:

```text
rachis  X  pedicel_base
pedicel_base X pedicel_tip
pedicel_tip  X tomato
```

con `FilteredPairs`, e addirittura i collider dei due segmenti di pedicello erano disabilitati.

PhysX/OpenUSD raccomanda proprio di evitare collisioni tra corpi collegati quando queste possono interferire con il constraint. ([OpenUSD][2])

Io filtrerei almeno:

```text
Tomato       <-> PedicelTip
Tomato       <-> PedicelBase
PedicelTip   <-> PedicelBase
PedicelBase  <-> Rachis attachment
```

eventualmente anche:

```text
Tomato <-> rachis segment a cui appartiene quel pedicello
```

come test diagnostico.

Se togliendo queste collisioni il “sollevamento” sparisce, hai già trovato il colpevole: era **depenetration**, non il FixedJoint.

---

**3. Aumentare soprattutto le `position iterations`.**

PhysX dice esplicitamente che quando jointed bodies risultano gommosi, oscillanti o con errore di constraint, la prima leva numerica è aumentare le **position solver iterations**. Più iterazioni significano una soluzione più accurata del vincolo. ([nvidia-omniverse.github.io][3])

Per esempio sul tomato rigid body:

```python
from pxr import PhysxSchema

api = PhysxSchema.PhysxRigidBodyAPI.Apply(
    stage.GetPrimAtPath(tomato_path)
)

api.CreateSolverPositionIterationCountAttr().Set(64)
api.CreateSolverVelocityIterationCountAttr().Set(1)
```

Il punto importante è che adesso il pomodoro è un **RigidBody standalone**, quindi io imposterei esplicitamente anche su di lui le solver iterations, invece di configurare soltanto l'articulation. Isaac/PhysX espone infatti un iteration count specifico per i rigid body. ([NVIDIA Docs][4])

Farei:

```text
position iterations
16
32
64
```

e misurerei lo spostamento del FixedJoint sotto **la stessa forza**.

Non aumenterei per prima cosa le velocity iterations. Per un errore geometrico del joint sono principalmente le **position iterations** che ci interessano. ([nvidia-omniverse.github.io][5])

---

**4. Controllare il mass ratio proprio al confine articulation ↔ rigid body.**

PhysX suggerisce di evitare rapporti di massa > circa **10:1** nei sistemi con joint perché rallentano la convergenza e fanno apparire i constraint più “rubbery”. ([nvidia-omniverse.github.io][1])

E qui ho notato una cosa molto interessante nel tuo vecchio codice:

```python
pedicel_tip_mass = max(mass * 0.1, 0.015)
tomato_mass      = mass * 0.9
```

quindi grossomodo:

```text
Tomato : PedicelTip
  0.9  :   0.1

≈ 9 : 1
```

**Questa è praticamente perfetta rispetto alla raccomandazione di PhysX.**

Io manterrei quella proprietà nel modello nuovo:

```text
m_tomato / m_attachment_link <= ~10
```

piuttosto che avere, per esempio:

```text
tomato = 200 g
pedicel tip = 2 g

ratio = 100:1    ❌
```

che è numericamente molto più cattivo.

---

**5. Se ancora si muove: ridurre `dt`.**

PhysX raccomanda anche di ridurre il timestep quando l'aumento della qualità del solver non è sufficiente. ([nvidia-omniverse.github.io][6])

Tu nel vecchio test eri già a:

```text
120 Hz
dt = 1/120 ≈ 8.33 ms
```

Farei il test:

```text
120 Hz   → baseline
240 Hz   → test
```

Se a 240 Hz lo shift diminuisce drasticamente, hai conferma che è principalmente errore numerico del maximal constraint.

Non terrei necessariamente 240 Hz nella pianta finale: serve prima come **diagnostica**.

---

### E `maxDepenetrationVelocity`?

Può essere molto utile **solo se quello che vedi è un pop provocato dalle collisioni**.

PhysX consiglia di ridurre la maximum depenetration velocity quando corpi interpenetrati vengono separati troppo violentemente, perché così si riduce l'overshoot. ([nvidia-omniverse.github.io][6])

Quindi:

```text
se il tomato "salta":
    collision filtering
       ↓
    se persiste ed esistono contatti iniziali:
       ↓
    maxDepenetrationVelocity
```

Non la userei invece per mascherare un FixedJoint con frame sbagliati.

### Non partirei dalla joint projection

PhysX ha storicamente una feature di **joint projection** che forza i corpi a riavvicinarsi quando l'errore del FixedJoint supera una tolleranza. Ma NVIDIA avverte che tolleranze troppo strette possono a loro volta introdurre jitter; inoltre l'esposizione USD di `enableProjection` è diventata deprecata nelle versioni più recenti. ([nvidia-omniverse.github.io][7])

Quindi nel tuo Isaac Sim 4.5 **non la sceglierei come soluzione principale**.

---

## Nel tuo caso farei questo test molto semplice

Mantieni esattamente il truss che ora funziona e fai 4 run:

```text
A
current configuration
120 Hz
current iterations
current collisions

B
come A
+ tutte le collisioni attorno al FixedJoint filtrate

C
come B
+ positionIterations = 64
  anche sul Tomato rigid body

D
come C
+ 240 Hz
```

Applichi sempre la stessa forza, per esempio lateralmente:

```text
F = costante
duration = costante
```

e misuri:

```text
joint positional error =
|| worldFrame0.position - worldFrame1.position ||
```

Se ottieni una cosa tipo:

```text
A   3.0 mm
B   0.8 mm
C   0.2 mm
D   0.05 mm
```

hai sostanzialmente dimostrato che l'architettura è buona e quello che rimane è semplicemente **errore numerico controllabile del maximal FixedJoint**.

E per me c'è un punto particolarmente importante: **non cercherei di eliminarlo al 100%**. Il compromesso che stai facendo è:

```text
reduced-coordinate joint
        ↓
molto rigido / praticamente zero drift
        ↓
NON breakable

vs

excluded maximal FixedJoint
        ↓
breakable ✅
        ↓
piccolo solver error
```

PhysX documenta proprio il FixedJoint come utile quando serve breakability/constraint force, pur ammettendo che il solver possa introdurre drift. ([nvidia-omniverse.github.io][1])

Se riesci a portare quell'errore a qualcosa di trascurabile rispetto alle dimensioni del pomodoro/pedicello, **terrei assolutamente questa architettura**.

[1]: https://nvidia-omniverse.github.io/PhysX/physx/5.1.0/docs/Joints.html?utm_source=chatgpt.com "Joints — physx 5.1.0 documentation"
[2]: https://openusd.org/release/api/usd_physics_page_front.html?utm_source=chatgpt.com "Universal Scene Description: UsdPhysics : USD Physics Schema"
[3]: https://nvidia-omniverse.github.io/PhysX/physx/5.4.0/docs/Joints.html?utm_source=chatgpt.com "Joints — physx 5.4.0 documentation"
[4]: https://docs.omniverse.nvidia.com/kit/docs/omni_usd_schema_physics/104.2/class_physx_schema_physx_rigid_body_a_p_i.html?utm_source=chatgpt.com "PhysxSchemaPhysxRigidBodyAPI Class Reference"
[5]: https://nvidia-omniverse.github.io/PhysX/physx/5.8.0/_api_build/classPxRigidDynamic.html?utm_source=chatgpt.com "PxRigidDynamic — PhysX SDK Documentation"
[6]: https://nvidia-omniverse.github.io/PhysX/physx/5.1.0/docs/RigidBodyCollision.html?utm_source=chatgpt.com "Rigid Body Collision — physx 5.1.0 documentation"
[7]: https://nvidia-omniverse.github.io/PhysX/physx/5.1.0/_build/physx/latest/class_px_fixed_joint.html?utm_source=chatgpt.com "PxFixedJoint — physx 5.1.0 documentation"
