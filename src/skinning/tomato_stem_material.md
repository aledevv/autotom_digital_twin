Sì: ho controllato l’implementazione sul branch **`feature/realistic-skinning`**. Direi **GO**, con due correzioni piccole prima di estendere lo stesso sistema a stelo e pomodori.

La struttura è buona: hai centralizzato i materiali in `src/exporterV2/core/usd/materials.py`, hai un preset leggero `UsdPreviewSurface` e uno `OmniSurface`, riusi il materiale tramite `/World/Looks/...`, e le nuove leaf blade sono visual mesh separate, double-sided e con binding del materiale. Inoltre la geometria della foglia è già molto più interessante del vecchio piano piatto: profilo Merlice, piega lungo la nervatura, arch e sag statico.

### Due cose da sistemare sulla foglia

La prima è importante: attualmente in `leaf_blade.py` fai:

```python
material = get_or_create_tomato_leaf_material(stage)
```

quindi **usi sempre il preset di default `realtime`**, non `realistic`. L'OmniSurface esiste ma non viene selezionato dalla pianta.

Non è necessariamente un errore: anzi, come architettura mi piace. Farei semplicemente un config globale:

```python
class AppearanceConfig:
    MATERIAL_PRESET = "realtime"   # realtime | realistic
```

e poi:

```python
get_or_create_tomato_leaf_material(
    stage,
    preset=AppearanceConfig.MATERIAL_PRESET,
)
```

Così puoi benchmarkare la stessa pianta senza cambiare codice.

Seconda cosa: nel preset realtime hai `specularColor`, ma `UsdPreviewSurface` lo usa solo quando `useSpecularWorkflow = 1`. Di default sei nel metallic workflow, quindi quel parametro è sostanzialmente ignorato. ([OpenUSD][1])

Io nel realtime semplicemente toglierei `specularColor`. Con `metallic=0` hai già una risposta dielettrica sensata. In seguito puoi aggiungere `ior` se la tua versione lo gestisce bene.

Per il resto, bene.

---

# 1. Stelo: qui sfrutterei moltissimo quello che hai già

Per lo stelo sei messo particolarmente bene perché il backend attuale separa già:

```text
PHYSICS
/World/Stem/Vegetative
    rigid bodies
    joints

        ↓ drives

VISUAL
/World/PlantVisual
    organic tube mesh
```

La mesh visuale ha già taper, bulging alle giunzioni, root flare ecc.

Quindi **non toccherei assolutamente la fisica**. Aggiungerei il materiale direttamente alle mesh organiche.

In `materials.py` aggiungerei:

```python
get_or_create_tomato_stem_material(
    stage,
    preset="realtime",
)
```

con due preset analoghi alla foglia.

### Stem realtime

Per la pianta completa partirei da:

```python
"realtime": {
    "diffuseColor": Gf.Vec3f(0.26, 0.42, 0.23),
    "roughness": 0.68,
    "metallic": 0.0,
    "clearcoat": 0.04,
    "clearcoatRoughness": 0.45,
}
```

Lo stelo del pomodoro non deve essere plasticoso. Deve essere:

```text
diffuse green
████████████████

+ rough surface
~~~~~~~~~~~~~~~~

+ very weak wax reflection
--------------------------

+ perceived fuzz / trichomes
```

Gli steli di `Solanum lycopersicum` sono effettivamente ricchi di trichomi, sia glandulari sia non glandulari. ([PubMed Central (PMC)][2])

Per questo nel preset **realistic OmniSurface** userei soprattutto lo `Sheen`, invece di modellare fisicamente migliaia di peletti.

NVIDIA descrive proprio lo sheen come una componente per microfibre e cita anche leaf/peach fuzz tra gli impieghi. ([NVIDIA Docs][3])

Un punto di partenza:

```python
"realistic": {
    "diffuse_reflection_weight": 0.90,
    "diffuse_reflection_color": Gf.Vec3f(0.22, 0.40, 0.18),
    "diffuse_reflection_roughness": 0.20,

    "metalness": 0.0,

    "specular_reflection_weight": 0.45,
    "specular_reflection_roughness": 0.68,
    "specular_reflection_ior": 1.42,

    "thin_walled": False,

    # tiny amount of internal scattering
    "enable_diffuse_transmission": True,
    "subsurface_weight": 0.05,

    # trichomes
    "specular_retro_reflection_weight": 0.12,
    "specular_retro_reflection_color":
        Gf.Vec3f(0.40, 0.55, 0.30),
    "specular_retro_reflection_roughness": 0.65,
}
```

Qui `thin_walled=False`: a differenza della foglia, **lo stelo è un volume**.

---

## Attenzione a dove fai il binding dello stelo

Questo è l'aspetto architetturale più importante.

Il tuo builder supporta:

```text
skinned
static
rigid-single
segmented
```

Quindi **non mettere il materiale soltanto in `author_visual_axis()`**, altrimenti funzionerebbe solo nello skinned mode.

Farei qualcosa come:

```python
material = get_or_create_tomato_stem_material(
    stage,
    preset=AppearanceConfig.MATERIAL_PRESET,
)
```

e lo passerei a tutti:

```text
author_visual_axis()
author_static_visual_axis()
author_rigid_visual_axis()
author_segmented_visual_axis()
```

`author_plain_mesh()` supporta già `material=...`, quindi static/rigid/segmented sono praticamente pronti.

Per lo skinned invece aggiungi dopo aver creato `mesh`:

```python
UsdShade.MaterialBindingAPI.Apply(
    mesh.GetPrim()
).Bind(material)
```

---

# 2. Dopo il materiale: una texture procedurale dello stelo

Qui secondo me vale davvero la pena.

Non farei una foto-texture. Farei una texture procedurale molto semplice:

```text
ALBEDO
    variazione verde +/- 5%

ROUGHNESS
    noise lento
    0.58 → 0.75

NORMAL
    micro-striature longitudinali
    +
    piccolissima irregolarità
```

Dato che il tuo stelo è già costruito per rings, gli UV vengono praticamente gratis:

```python
u = radial_index / radial_segments

v = arc / texture_repeat_length
```

cioè:

```text
             v
             ↑
             │
stem         │  |||||||||||
surface      │  |||||||||||
             │  |||||||||||
             │  |||||||||||
             └──────────────→ u
                 0 ... 1
```

E puoi far ripetere la texture ogni, per esempio, 5–8 cm.

Non servono displacement reali: normal map + sheen dovrebbero già fare molto.

---

# 3. Pomodoro: qui farei un materiale diverso

Il pomodoro è quasi l'opposto dello stelo.

Uno stelo deve essere:

> rough + fuzzy + diffusely green

Un pomodoro deve essere:

> smooth + waxy + moderately glossy + slightly subsurface

Gli studi sulla cuticola del pomodoro mostrano proprio un rivestimento di cutina + cere epicuticolari; nel wild-type il frutto viene descritto come **moderatamente glossy**, con la rugosità microscopica che determina il bilanciamento tra riflessione speculare e diffusa. ([PubMed Central (PMC)][4])

Per questo OmniSurface è interessante soprattutto per due componenti:

```text
      AIR
       ↓

 ┌───────────────┐
 │ wax / cuticle │  ← COAT
 ├───────────────┤
 │               │
 │ tomato tissue │  ← diffuse + SUBSURFACE
 │ carotenoids   │
 │               │
 └───────────────┘
```

Il `coat` di OmniSurface è precisamente uno strato dielettrico trasparente sopra il materiale sottostante. ([NVIDIA Docs][5])

### Tomato realtime

Per la pianta completa proverei prima:

```python
{
    "diffuseColor": tomato_color,
    "roughness": 0.28,
    "metallic": 0.0,

    "clearcoat": 0.28,
    "clearcoatRoughness": 0.18,
}
```

È già un salto enorme rispetto a:

```python
sphere + displayColor
```

che è quello che hai adesso.

### Tomato realistic / OmniSurface

Poi:

```python
{
    "diffuse_reflection_weight": 0.80,
    "diffuse_reflection_color": tomato_color,
    "diffuse_reflection_roughness": 0.10,

    "metalness": 0.0,

    "specular_reflection_weight": 0.65,
    "specular_reflection_roughness": 0.30,
    "specular_reflection_ior": 1.45,

    "thin_walled": False,

    # tomato tissue
    "enable_diffuse_transmission": True,
    "subsurface_weight": 0.10,

    # waxy cuticle
    "coat_weight": 0.30,
    "coat_color": Gf.Vec3f(1.0, 1.0, 1.0),
    "coat_roughness": 0.18,
    "coat_ior": 1.45,
}
```

Qui starei molto più prudente col subsurface rispetto alla foglia.

La foglia può realmente “accendersi” controluce.

Il pomodoro no: se esageri diventa una lampadina rossa.

Quindi:

```text
0.00 SSS    plastic-looking

0.05
0.10       ← partirei qui
0.15

0.30+       probabilmente gummy/candle tomato
```

OmniSurface è pensato proprio per materiali organici in cui la luce entra e viene diffusa sotto la superficie. ([NVIDIA Docs][6])

---

# 4. Il problema interessante: maturazione

Qui farei attenzione a non rompere una cosa molto bella che hai già.

Attualmente:

```python
PlantColors.tomato_color(maturation)
```

calcola il colore dal valore di maturazione.

Quindi **non creare un singolo `TomatoMaterial` rosso**, perché perderesti questa informazione.

Farei:

```python
get_or_create_tomato_material(
    stage,
    maturation,
    preset="realtime",
)
```

e quantizzerei la maturazione:

```python
N_MATURATION_BUCKETS = 8

bucket = round(maturation * (N_MATURATION_BUCKETS - 1))
```

ottenendo:

```text
/World/Looks/Tomato/Maturation_0
/World/Looks/Tomato/Maturation_1
/World/Looks/Tomato/Maturation_2
...
/World/Looks/Tomato/Maturation_7
```

Così 100 pomodori non generano 100 shader differenti.

Massimo:

**8 materiali condivisi.**

---

# Cambierei anche la curva colore

Adesso fai un lerp diretto:

```text
GREEN -------------------------- RED
 0                               1
```

Visivamente però può attraversare colori marrone/oliva un po' artificiali.

Userei una ramp:

```text
maturation

0.00       0.30       0.50       0.70        1.00
 │           │          │          │            │
green → yellow-green → orange → red-orange → deep red
```

per esempio:

```python
TOMATO_RIPENING_COLORS = [
    (0.00, (0.18, 0.48, 0.08)),
    (0.30, (0.48, 0.58, 0.07)),
    (0.50, (0.82, 0.42, 0.05)),
    (0.70, (0.92, 0.20, 0.05)),
    (1.00, (0.75, 0.055, 0.025)),
]
```

con interpolazione tra gli stop.

Questo, secondo me, dà più realismo del passare mezz'ora a perfezionare lo shader.

---

# 5. Sul pomodoro farei anche una piccola modifica geometrica

Il materiale migliorerà moltissimo la sfera, ma rimarrà una **sfera perfetta**.

Nel tuo `create_sphere_rigid_body()` oggi hai:

```text
Tomato Xform
└── Sphere
    ├── visual
    └── collision
```

Non deformerei quella Sphere, perché la sua collisione e il comportamento del detachment funzionano già.

Passerei in futuro a:

```text
TomatoRigidBody
│
├── CollisionSphere
│      PhysX
│      exact simple sphere
│
└── TomatoVisual
       material
       NO COLLISION
       ↓
     slightly irregular mesh
```

E farei il visual:

```text
        ___
     __/   \__
   /           \
  |             |
  |             |
   \           /
     \_______/
```

invece di:

```text
      ______
    /        \
   |          |
    \________/
```

Bastano:

* Z scale `0.94–0.98`;
* variazione radiale `±1–2%`;
* leggerissima depressione vicino al pedicello.

Collisione e massa rimangono identiche.

---

# 6. E poi il dettaglio con il miglior ROI: il calice

Dopo il materiale, per il pomodoro **non farei subito normal map**.

Prima aggiungerei le 5 sepali verdi intorno all'attacco:

```text
          pedicel
             │
          \  │  /
           \ │ /
        ----\│/----
           tomato
         .--------.
       /            \
```

Cinque triangolini/leaf-like mesh molto semplici, visual-only.

Un pomodoro perfettamente rosso e glossy senza calice continua a sembrare una sfera rossa.

Un pomodoro anche relativamente semplice con:

* pedicello;
* calice;
* forma leggermente schiacciata;
* materiale ceroso;

legge immediatamente come **pomodoro**.

---

## Quindi la sequenza che adotterei

| Step  | Modifica                             |       Costo |       Guadagno |
| ----- | ------------------------------------ | ----------: | -------------: |
| **A** | fix preset foglia                    |       nullo |          medio |
| **B** | Stem UsdPreviewSurface               | quasi nullo |           alto |
| **C** | Tomato UsdPreviewSurface + clearcoat | quasi nullo | **molto alto** |
| **D** | ripening color ramp                  |       nullo |           alto |
| **E** | Stem OmniSurface + sheen             |   medio GPU |     medio/alto |
| **F** | Tomato OmniSurface + coat + SSS      |   medio GPU |           alto |
| **G** | calice visual-only                   |       basso | **molto alto** |
| **H** | visual tomato mesh separata          |       basso |           alto |
| **I** | procedural normal/roughness textures | basso/medio |     rifinitura |

La cosa che mi piace è che **non serve più intervenire sull'architettura fisica**: il ramo visuale è già separato dalla fisica, le leaf blade visuali sono già separate, e sul pomodoro possiamo fare la stessa separazione mantenendo intatto fixed joint/break force/collision sphere.

Quindi io implementerei ora **un unico appearance system in `materials.py` con `Leaf`, `Stem` e `Tomato`, ciascuno `realtime/realistic`**, invece di continuare ad aggiungere materiali ad hoc. È abbastanza piccolo da non diventare overengineering, ma ti lascia un sistema molto pulito anche da descrivere nella tesi.

[1]: https://openusd.org/dev/spec_usdpreviewsurface.html?utm_source=chatgpt.com "UsdPreviewSurface Specification — Universal Scene Description 26.08 documentation"
[2]: https://pmc.ncbi.nlm.nih.gov/articles/PMC4676884/?utm_source=chatgpt.com "The development of type VI glandular trichomes in the cultivated tomato Solanum lycopersicum and a related wild species S. habrochaites - PMC"
[3]: https://docs.omniverse.nvidia.com/materials-and-rendering/latest/templates/parameters/OmniSurface_Sheen.html?utm_source=chatgpt.com "Sheen — Omniverse Materials and Rendering"
[4]: https://pmc.ncbi.nlm.nih.gov/articles/PMC3912114/?utm_source=chatgpt.com "Analyses of Tomato Fruit Brightness Mutants Uncover Both Cutin-Deficient and Cutin-Abundant Mutants and a New Hypomorphic Allele of GDSL Lipase - PMC"
[5]: https://docs.omniverse.nvidia.com/materials-and-rendering/latest/templates/OmniSurface.html?utm_source=chatgpt.com "OmniSurfaceBase and OmniSurfaceLiteBase — Omniverse Materials and Rendering"
[6]: https://docs.omniverse.nvidia.com/materials-and-rendering/latest/templates/parameters/OmniSurface_Subsurface.html?utm_source=chatgpt.com "Subsurface — Omniverse Materials and Rendering"
