Sì. Ho guardato sia le immagini sia come stai generando adesso le foglie nell’`exporterV2`, e secondo me qui possiamo ottenere un salto visivo enorme senza appesantire quasi per niente la fisica.

La cosa importante è questa: **attualmente le tue leaf blade non hanno praticamente un vero materiale**. Nel repo vengono create come mesh piatta e ricevono semplicemente un `displayColor`; inoltre `_generate_leaf_mesh_data()` genera una superficie sul piano `y=0` con un contorno abbastanza semplice.  In `create_static_mesh()` poi assegni il colore e usi la stessa mesh anche per la collisione convex hull.

Quindi c'è molto margine.

## Materiale che userei: OmniSurface, non semplice OmniPBR

Per una foglia eviterei di fermarmi a `OmniPBR`.

`OmniPBR` è ottimo per albedo + roughness + normal map + opacity, e Isaac Sim lo supporta direttamente, ma una foglia ha una caratteristica fondamentale: **la luce la attraversa e viene diffusa internamente**.

NVIDIA indica esplicitamente `OmniSurface` per materiali come il foliage. Ancora più interessante: la documentazione dice specificamente che `Thin Walled` è adatto a oggetti sottili come **le foglie** e che, quando Thin Walled è attivo, il subsurface diventa di fatto una **diffuse transmission / translucency** attraverso la superficie. ([NVIDIA Docs][1])

In pratica:

```text
                     luce
                      ↓
               ☀ ────────────
                      ↓
           ┌──────────────────┐
           │ CUTICOLA         │ ← specular / roughness
           │                  │
           │ MESOFILLO        │ ← diffuse + scattering
           │ chlorofilla      │
           │                  │
           └──────────────────┘
                 ↓ ↓ ↓
             luce trasmessa

CAMERA → riflessione verde
DIETRO → translucenza verde/giallastra
```

È proprio quell'effetto che nelle foglie reali fa una differenza enorme.

---

# La ricetta che proverei sulla tua foglia di pomodoro

Dalle immagini che hai mandato ci sono almeno cinque caratteristiche da catturare: verde scuro non uniforme, superficie abbastanza opaca ma con highlight della cuticola, nervatura principale e secondarie molto evidenti, micro-rugosità, e una leggera peluria soprattutto sui bordi e sulle nervature.

Curiosamente anche NVIDIA suggerisce lo **Sheen** di OmniSurface per materiali come le foglie e il “peach fuzz”, perché simula microfibre visibili soprattutto agli angoli radenti. ([NVIDIA Docs][1])

Come punto di partenza userei:

| Proprietà OmniSurface                 |       Valore iniziale | Effetto                     |
| ------------------------------------- | --------------------: | --------------------------- |
| `diffuse_reflection_color`            | `(0.10, 0.30, 0.055)` | verde lamina                |
| `diffuse_reflection_weight`           |                `0.85` | componente diffusa          |
| `diffuse_reflection_roughness`        |                `0.12` | scattering superficiale     |
| `metalness`                           |                 `0.0` | ovviamente non metallica    |
| `specular_reflection_weight`          |                 `1.0` | riflessione della cuticola  |
| `specular_reflection_roughness`       |                `0.50` | highlight largo e morbido   |
| `specular_reflection_ior`             |            **`1.42`** | Fresnel realistico          |
| `thin_walled`                         |            **`True`** | fondamentale                |
| `enable_diffuse_transmission`         |            **`True`** | attiva translucenza         |
| `subsurface_weight`                   |           `0.25–0.35` | quantità di back-light      |
| `subsurface_transmission_color`       |  `(0.30, 0.55, 0.08)` | luce trasmessa verde/gialla |
| `specular_retro_reflection_weight`    |           `0.05–0.10` | micro peluria               |
| `specular_retro_reflection_roughness` |             `0.4–0.6` | fuzz molto morbido          |

Il valore **IOR ≈ 1.42** non è casuale. Misure storiche sulle pareti cellulari vegetali, incluse foglie di pomodoro, danno un valore medio vicino a 1.425; misure su tessuti vegetali idratati sono intorno a 1.4–1.42. ([PubMed Central (PMC)][2]) Anche la cuticola cerosa viene comunemente modellata attorno a 1.46. ([DOI][3])

Quindi siamo anche abbastanza difendibili a livello di tesi.

---

# Come lo inserirei nel tuo exporter

Io creerei:

```text
src/exporterV2/core/usd/
├── geometry.py
├── joints.py
├── ...
└── materials.py       ← nuovo
```

e dentro `materials.py` metterei qualcosa del genere:

```python
from pxr import UsdGeom, UsdShade, Sdf, Gf


TOMATO_LEAF_MATERIAL_PATH = "/World/Looks/TomatoLeaf"


def get_or_create_tomato_leaf_material(
    stage,
    material_path=TOMATO_LEAF_MATERIAL_PATH,
):
    existing = stage.GetPrimAtPath(material_path)
    if existing.IsValid():
        return UsdShade.Material(existing)

    # Shared material scope
    UsdGeom.Scope.Define(stage, "/World/Looks")

    material = UsdShade.Material.Define(stage, material_path)
    shader = UsdShade.Shader.Define(
        stage,
        f"{material_path}/Shader",
    )

    # Use NVIDIA OmniSurface MDL
    shader.CreateImplementationSourceAttr(
        UsdShade.Tokens.sourceAsset
    )
    shader.SetSourceAsset(
        "OmniSurface.mdl",
        "mdl",
    )
    shader.SetSourceAssetSubIdentifier(
        "OmniSurface",
        "mdl",
    )

    shader.CreateOutput(
        "out",
        Sdf.ValueTypeNames.Token,
    )

    material.CreateSurfaceOutput("mdl").ConnectToSource(
        shader.ConnectableAPI(),
        "out",
    )
    material.CreateDisplacementOutput("mdl").ConnectToSource(
        shader.ConnectableAPI(),
        "out",
    )
    material.CreateVolumeOutput("mdl").ConnectToSource(
        shader.ConnectableAPI(),
        "out",
    )

    # ------------------------------------------------------------------
    # TOMATO LEAF
    # ------------------------------------------------------------------

    shader.CreateInput(
        "diffuse_reflection_weight",
        Sdf.ValueTypeNames.Float,
    ).Set(0.85)

    shader.CreateInput(
        "diffuse_reflection_color",
        Sdf.ValueTypeNames.Color3f,
    ).Set(Gf.Vec3f(0.10, 0.30, 0.055))

    shader.CreateInput(
        "diffuse_reflection_roughness",
        Sdf.ValueTypeNames.Float,
    ).Set(0.12)

    shader.CreateInput(
        "metalness",
        Sdf.ValueTypeNames.Float,
    ).Set(0.0)

    # Waxy cuticle
    shader.CreateInput(
        "specular_reflection_weight",
        Sdf.ValueTypeNames.Float,
    ).Set(1.0)

    shader.CreateInput(
        "specular_reflection_roughness",
        Sdf.ValueTypeNames.Float,
    ).Set(0.50)

    shader.CreateInput(
        "specular_reflection_ior",
        Sdf.ValueTypeNames.Float,
    ).Set(1.42)

    # Leaf = geometrically thin surface
    shader.CreateInput(
        "thin_walled",
        Sdf.ValueTypeNames.Bool,
    ).Set(True)

    # Diffuse transmission / translucency
    shader.CreateInput(
        "enable_diffuse_transmission",
        Sdf.ValueTypeNames.Bool,
    ).Set(True)

    shader.CreateInput(
        "subsurface_weight",
        Sdf.ValueTypeNames.Float,
    ).Set(0.30)

    shader.CreateInput(
        "subsurface_transmission_color",
        Sdf.ValueTypeNames.Color3f,
    ).Set(Gf.Vec3f(0.30, 0.55, 0.08))

    # Very subtle tomato-leaf trichomes
    shader.CreateInput(
        "specular_retro_reflection_weight",
        Sdf.ValueTypeNames.Float,
    ).Set(0.07)

    shader.CreateInput(
        "specular_retro_reflection_color",
        Sdf.ValueTypeNames.Color3f,
    ).Set(Gf.Vec3f(0.35, 0.50, 0.22))

    shader.CreateInput(
        "specular_retro_reflection_roughness",
        Sdf.ValueTypeNames.Float,
    ).Set(0.50)

    return material
```

Il metodo di creare direttamente un MDL material tramite `UsdShade`, `sourceAsset` e `sourceAsset:subIdentifier` è supportato dal workflow USD di Omniverse; quindi si adatta molto bene al tuo exporter, perché **non sei obbligato a creare il materiale manualmente dentro la GUI di Isaac Sim**. ([NVIDIA Docs][4])

Poi alla tua mesh:

```python
material = get_or_create_tomato_leaf_material(stage)

UsdShade.MaterialBindingAPI.Apply(
    mesh.GetPrim()
).Bind(material)
```

e farei anche:

```python
mesh.CreateDoubleSidedAttr().Set(True)
```

per evitare qualsiasi problema di back-face con le tue leaf blade infinitamente sottili.

---

# Ma questo è solo il primo salto

Con questo materiale la tua foglia passerebbe già da:

```text
displayColor
       ↓
████████████
verde uniforme
```

a:

```text
                  LIGHT
                    ↓

             soft specular
                  ↙
        ╭────────────────╮
camera ←│  GREEN LEAF    │
        │   + surface    │
        │   scattering   │
        ╰────────────────╯
                   ↓
             transmitted
             green light
```

e soprattutto vedresti la foglia **accendersi controluce**, cosa che secondo me farà tantissimo sull'intera pianta.

Per arrivare però a quello che chiamerei davvero **“realismo notevole”**, manca il secondo passaggio:

```text
                Tomato Leaf
                     │
       ┌─────────────┼─────────────┐
       ↓             ↓             ↓
    ALBEDO         NORMAL       ROUGHNESS
       │             │             │
 chlorophyll     veins +       wax/micro
 variation       wrinkles       texture
       │
       └─────────────┐
                     ↓
                  OPACITY
                     │
              serrated border
```

### Ed è qui che faremo la grossa differenza

La normal map dovrebbe contenere:

**midrib → secondary veins → tertiary veins → microscopiche increspature della lamina.**

La geometria invece rimane leggerissima.

Questo è esattamente ciò che vedo bene soprattutto nella tua terza immagine: la nervatura principale non è soltanto più chiara, ma modifica fortemente le normali della superficie. Se la facciamo soltanto come linea chiara nell'albedo sembrerà stampata; se la mettiamo nella **normal map**, prenderà realmente la luce.

---

## E possiamo anche rendere seghettato il bordo senza aggiungere un sacco di triangoli

Attualmente il tuo `_generate_leaf_mesh_data()` produce un contorno molto liscio.

Una foglia di pomodoro invece ha:

```text
attuale:

        /\
      /    \
    /        \
   /          \
  /            \
 /              \


target:

         /\
      __/  \_
    _/       \_
  _/  \_    _  \_
 /      \__/ \   \
/             \___\
```

Possiamo farlo in due modi.

Il più economico è **opacity/cutout texture**. NVIDIA supporta opacity e opacity threshold nei propri materiali, proprio per creare dettaglio geometrico apparente su mesh leggere. ([NVIDIA Docs][1])

L'altro è aggiungere qualche vertice al profilo.

Per il tuo digital twin sceglierei inizialmente **opacity mask**, perché decine o centinaia di foglie rimangono estremamente leggere.

---

# Una cosa importantissima che ho notato nel tuo codice

Adesso `create_static_mesh()` fa anche:

```python
UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())

UsdPhysics.MeshCollisionAPI.Apply(
    mesh.GetPrim()
).GetApproximationAttr().Set("convexHull")
```

quindi **la stessa leaf mesh è sia visuale sia collision geometry**.

Finché aggiungiamo:

* materiale;
* UV;
* texture;
* normal map;
* roughness;
* opacity;

nessun problema: **la fisica rimane praticamente identica**.

Se però in futuro cominciamo ad aggiungere vere rughe, nervature geometriche o tanta tessellation, io separerei:

```text
LeafRigidBody
│
├── LeafCollision
│      └── mesh semplice
│          convexHull
│
└── LeafVisual
       └── mesh bella
           UV
           normal map
           material
           NO COLLISION
```

Questo secondo me è perfetto per il tuo progetto perché tiene separati **fidelity grafica** e **fidelity fisica**.

---

# Le foto che mi hai dato

Le userei come **reference**, non direttamente come texture finale.

La terza è ottima per capire nervature e colore, ma ha illuminazione già incorporata. La quarta mostra molto bene lamina, venature e peluria del bordo, ma è anche una foto con watermark.

Per una vera texture PBR preferirei una delle due strade:

**foto tua di una foglia vera**, ripresa quasi ortogonalmente con illuminazione molto diffusa;

oppure, cosa che mi piace ancora di più per la tua tesi, **texture procedurale generata da Python**.

Potremmo generare automaticamente:

```text
tomato_leaf_albedo.png
tomato_leaf_normal.png
tomato_leaf_roughness.png
tomato_leaf_opacity.png
```

in funzione delle dimensioni della foglia create da groIMP.

E sarebbe interessante perché il tuo exporter non avrebbe una texture presa a caso da Internet: avrebbe un **procedural tomato-leaf appearance model**.

---

# C'è però un piccolo pezzo che al momento ti manca: gli UV

Le tue mesh attuali hanno `points`, triangoli e indici, ma non vedo UV authorati.

Fortunatamente con la tua geometria sono facilissimi.

Dato che la foglia è parametrizzata da:

```python
x = larghezza
z = lunghezza
```

possiamo semplicemente definire:

```python
u = 0.5 + x / (2.0 * half_width)
v = z / length
```

e ottenere:

```text
UV

(0,1)               (1,1)
  ┌────────────────────┐
  │         ▲          │
  │        / \         │
  │      /     \       │
  │    /         \     │
  │   /           \    │
  │       LEAF          │
  │          │          │
  └──────────┼─────────┘
(0,0)      base        (1,0)
```

A quel punto albedo, veins, normal e roughness seguiranno automaticamente foglie di dimensioni diverse.

---

## Come procederei io adesso

Fare una modifica enorme direttamente sulla pianta sarebbe inutile.

Seguirei esattamente la filosofia che abbiamo usato per gli altri test:

**Test A → una singola foglia sospesa, materiale OmniSurface senza texture.**

Mettiamo una luce davanti e una luce dietro e verifichiamo separatamente:

`roughness → specular → thin_walled → translucency → sheen`.

Quando quella appare bene:

**Test B → UV + normal map procedurale delle nervature.**

Poi:

**Test C → albedo + roughness + bordo dentellato.**

Infine la riportiamo dentro `exporterV2`.

Questa strada dovrebbe dare un rapporto **realismo/costo computazionale molto migliore dello skinning**: la parte costosa diventa soprattutto shading/rendering, mentre non stai aumentando articolazioni o complessità PhysX.

Se vuoi, nel prossimo messaggio partirei proprio dal **Test A** e ti preparo direttamente i file Python + script `.sh` per una singola leaf blade sospesa in Isaac Sim 4.5, con tre preset `MATTE / REALISTIC / STRONG_TRANSLUCENCY`, così li confrontiamo visivamente prima di toccare la pianta vera.

[1]: https://docs.omniverse.nvidia.com/materials-and-rendering/latest/templates/OmniSurface.html "https://docs.omniverse.nvidia.com/materials-and-rendering/latest/templates/OmniSurface.html"
[2]: https://pmc.ncbi.nlm.nih.gov/articles/PMC541577/?page=-1&utm_source=chatgpt.com "Refractive Index of Soybean Leaf Cell Walls - PMC"
[3]: https://doi.org/10.1002/ajb2.70104?utm_source=chatgpt.com "American Journal of Botany"
[4]: https://docs.omniverse.nvidia.com/dev-guide/latest/programmer_ref/usd/materials/create-mdl-material.html?utm_source=chatgpt.com "Create an MDL Material — Omniverse Developer Guide"
