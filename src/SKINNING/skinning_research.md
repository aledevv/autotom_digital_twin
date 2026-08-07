## Panoramica
IsaacSim, basato su OpenUSD, supporta nativamente skinning scheletrico tramite lo schema **UsdSkel**, deformazione mesh via fisica **Deformable Body** (beta), e materiali PBR tramite **OmniPBR/MDL**. Nessuno di questi flussi è pensato specificamente per piante, ma tutti sono adattabili con limitazioni concrete che vengono descritte sotto insieme a codice funzionante e alternative dove IsaacSim non basta.
## Skinning e Rigging in IsaacSim (UsdSkel)
Lo standard USD per lo skinning è lo schema `UsdSkel`, che collega una mesh a uno scheletro tramite joint weights e una `SkelRoot`. Il prim che racchiude sia lo scheletro sia la geometria deformata deve essere di tipo `SkelRoot`, requisito indispensabile perché IsaacSim/Hydra applichino correttamente la deformazione. Il binding richiede la `SkelBindingAPI` applicata sulla mesh, con i primvar `jointIndices`, `jointWeights` e `geomBindXform` che definiscono l'influenza di ciascun bone sui vertici.[^1][^2][^3]

Un caso di studio pratico: se importi FBX con skeleton esterni (es. da Blender), il converter di IsaacSim spesso non ricostruisce correttamente il binding automaticamente, anche wrappando manualmente con `UsdSkelRoot` e `UsdSkel.Skeleton` in Python — è un bug/limite noto e recente (segnalato su Isaac Sim 5.1.0). Questo significa che per un albero rigato "a mano" in Blender/GroIMP, il percorso più sicuro è costruire il rig UsdSkel direttamente in USD/Python piuttosto che fare affidamento sull'Asset Converter automatico.[^4]
### Codice: creare uno scheletro e bind della mesh (Python, pxr.UsdSkel)
```python
from pxr import Usd, UsdSkel, UsdGeom, Sdf, Gf, Vt

stage = Usd.Stage.CreateNew("albero_rig.usda")

# 1. SkelRoot: contenitore obbligatorio per qualsiasi comportamento di skinning
skel_root = UsdSkel.Root.Define(stage, "/World/Albero")

# 2. Skeleton: definisce i joint (es. tronco -> rami primari -> rami secondari)
skeleton = UsdSkel.Skeleton.Define(stage, "/World/Albero/Skeleton")
joint_paths = ["Tronco", "Tronco/RamoPrincipale1", "Tronco/RamoPrincipale1/RamoSec1"]
skeleton.CreateJointsAttr().Set(Vt.TokenArray(joint_paths))

# bind transforms (posa a riposo, in world space)
bind_transforms = [Gf.Matrix4d(1.0) for _ in joint_paths]
skeleton.CreateBindTransformsAttr().Set(Vt.Matrix4dArray(bind_transforms))
skeleton.CreateRestTransformsAttr().Set(Vt.Matrix4dArray(bind_transforms))

# 3. Mesh skinnabile con SkelBindingAPI
mesh = UsdGeom.Mesh.Define(stage, "/World/Albero/Corteccia")
binding_api = UsdSkel.BindingAPI.Apply(mesh.GetPrim())
binding_api.CreateSkeletonRel().SetTargets([skeleton.GetPath()])

# joint weights/indices per vertice (elementSize = numero di influenze per vertice)
n_verts = len(mesh.GetPointsAttr().Get() or [])
indices_pv = binding_api.CreateJointIndicesPrimvar(constant=False, elementSize=2)
weights_pv = binding_api.CreateJointWeightsPrimvar(constant=False, elementSize=2)
indices_pv.Set(Vt.IntArray([0, 1] * n_verts))
weights_pv.Set(Vt.FloatArray([0.7, 0.3] * n_verts))

# geomBindTransform: trasformazione della mesh al momento del bind
binding_api.CreateGeomBindTransformAttr().Set(Gf.Matrix4d(1.0))

stage.Save()
```

Per leggere/validare il rig in fase di runtime, `UsdSkelCache` è l'API di query raccomandata: va popolata a partire dalla `SkelRoot` e permette di ottenere `UsdSkelSkinningQuery` per ogni mesh skinnata.[^5][^6][^7]

```python
from pxr import UsdSkel

cache = UsdSkel.Cache()
cache.Populate(skel_root, Usd.TraverseInstanceProxies())
skinning_query = cache.GetSkinningQuery(mesh.GetPrim())
skel_query = cache.GetSkelQuery(skeleton)
```
### Limitazioni pratiche per uno "skin" di albero
- Lo skinning UsdSkel è pensato per rig gerarchici (personaggi, bracci robotici), non per topologie ramificate complesse come un albero; funziona, ma il binding manuale dei pesi per centinaia di rami è oneroso e va scriptato (es. distribuendo i pesi in base alla distanza euclidea/geodetica dal joint più vicino).[^2][^3]
- Import automatico da FBX/Blender di skeleton complessi mostra problemi noti di riconoscimento dei joint in Isaac Sim 5.x — verificare sempre in `usdview` che il binding sia integro prima di portare l'asset in IsaacSim.[^4]
- Se l'obiettivo primario è simulare il **movimento del fusto/rami sotto vento o contatto** (piuttosto che un'animazione scriptata), l'approccio "deformable body" fisico descritto sotto è più naturale di uno skinning skeletale rigido.
## Deformazione Fisica: Deformable Body (alternativa/complemento allo skinning)
Per un albero che deve deformarsi in modo fisicamente plausibile (piegatura di rami, oscillazione), IsaacSim offre **Deformable Body (Beta)**, basata su FEM/PBD, separata da UsdSkel. Va abilitata da Edit > Preferences > Physics > General > "Enable Deformable schema Beta" e richiede restart.[^8]

Esistono due modalità:
- **Volume Deformable**: per geometrie chiuse/watertight (es. tronco solido) — richiede una mesh tetraedrica di simulazione generata automaticamente o tramite hex resolution personalizzata.[^9][^8]
- **Surface Deformable**: per membrane sottili non chiuse (es. foglie) — più adatto a superfici tipo lamina.[^8]

```python
import omni.kit.commands
from pxr import UsdGeom

# Applicare deformable body a una mesh già importata (es. tronco subdiviso)
omni.kit.commands.execute(
    "AddDeformableBodyComponentCommand",
    skin_mesh_path="/World/Albero/Tronco",
    simulation_hexahedral_resolution=20,  # aumentare la risoluzione = più dettaglio ma più costo GPU
)
```

Punti critici documentati da NVIDIA e dalla community:
- La mesh visiva deve essere **sufficientemente sottodivisa a monte** (in Blender/GroIMP), perché il sistema non la subdivide automaticamente per la simulazione — 10-30 suddivisioni per lato sono un punto di partenza tipico.[^10][^8]
- Il deformable body richiede **GPU Dynamics abilitata** esplicitamente via `physicsScenePath.enable_gpu_dynamics(True)` o dalla UI Physics Scene.[^9]
- La risoluzione della mesh deformabile supportata è "piuttosto grossolana" secondo gli sviluppatori NVIDIA: per strutture fini come rametti o nervature di foglie, il livello di dettaglio ottenibile è limitato e potrebbe non essere sufficientemente realistico.[^10]
- Per il **materiale fisico** (rigidità, damping) va creato un `Deformable Body Material` via Create > Physics > Physics Material e assegnato al prim tramite il pannello Properties.[^8]
## Generazione della Superficie Irregolare (Corteccia, Terreno)
Non esiste in IsaacSim un tool nativo di "sculpting procedurale" per generare geometria organica irregolare (corteccia, rughe di tronco) — questo compito va risolto **fuori da IsaacSim**, poi importato come mesh/USD.
### Opzione A — Displacement/Bump procedurale in Blender (consigliata per corteccia)
Il flusso standard nell'industria per corteccia realistica combina texture procedurali (Voronoi + Noise) usate sia come bump/normal sia come vero displacement geometrico in Cycles: si genera una distanza Voronoi, la si porta in un nodo Bump per il normal e in un nodo Displacement (Material Properties > Settings > Displacement, richiede feature set "Experimental" per displacement adattivo) per deformare realmente la mesh. Questo approccio produce sia il dettaglio visivo (normal map) sia geometria fisicamente presente se serve per collisioni ravvicinate.[^11][^12]

Passi chiave:
1. Creare un cilindro con loop cut abbondanti (la subdivision è necessaria: senza vertici sufficienti il displacement non ha nulla su cui agire).[^11]
2. Voronoi Texture (Distance) → Bump node → Normal del Principled BSDF, per il dettaglio fine.
3. Stesso segnale Voronoi (con Noise texture per distorsione, per evitare pattern troppo regolari) → Displacement node → Material Output, per la deformazione geometrica reale.[^12][^11]
4. Esportare come mesh ad alta risoluzione (o bake displacement in una height map) e poi convertire in USD per IsaacSim.
### Opzione B — Terreno/superfici irregolari via heightfield/noise (per suolo, non per corteccia)
Per superfici tipo terreno (non corteccia), Isaac Lab offre un modulo dedicato di **terrain generation procedurale** con curriculum e heightfield configurabili, utile se il digital twin richiede anche un suolo irregolare attorno alla pianta. Questo è nativo NVIDIA (Isaac Lab, non IsaacSim core) e supporta terreni "rough" generati proceduralmente da rumore, con bug noti recenti sulla curriculum feature da monitorare negli aggiornamenti.[^13][^14][^15]
### Limitazione centrale
IsaacSim/Omniverse non ha un motore di sculpting o displacement procedurale nativo equivalente a Blender: la generazione di irregolarità superficiali va fatta a monte (Blender per corteccia via nodi di displacement, oppure Isaac Lab per terreni) e poi importata come USD statico o come height/normal map da applicare al materiale in IsaacSim.
## Creazione di Materiali in IsaacSim (OmniPBR/MDL)
Il materiale di riferimento per superfici organiche opache come corteccia o foglie è **OmniPBR**, il materiale PBR di default di Omniverse, che gestisce diffuse, roughness, metallic, normal ed emissive tramite texture indipendenti.[^16][^17]
### Codice: creare e assegnare OmniPBR via API high-level (IsaacSim 4.5+/5.x)
```python
from isaacsim.core.experimental.materials import OmniPbrMaterial
from isaacsim.core.experimental.objects import Cube  # sostituire con la mesh dell'albero

material = OmniPbrMaterial("/World/Materials/Corteccia")
material.set_input_values("diffuse_texture", "/percorso/albedo_corteccia.png")
material.set_input_values("normal_texture", "/percorso/normal_corteccia.png")
material.set_input_values("reflection_roughness_texture", "/percorso/roughness_corteccia.png")
material.set_input_values("reflection_roughness_constant", 0.85)

albero_mesh = Cube("/World/Albero/Corteccia")  # in pratica la tua mesh importata
albero_mesh.apply_visual_materials(material)
```

Per workflow più basso-livello o compatibilità retro con versioni precedenti, `omni.kit.commands` con `CreateAndBindMdlMaterialFromLibrary` resta la via standard, utile anche per binding puntuale su prim specifici tramite `BindMaterial`:[^18][^19]

```python
import omni.kit.commands
from pxr import Sdf, UsdShade

mtl_created = []
omni.kit.commands.execute(
    "CreateAndBindMdlMaterialFromLibrary",
    mdl_name="OmniPBR.mdl",
    mtl_name="OmniPBR_Corteccia",
    mtl_created_list=mtl_created,
)
material_path = mtl_created

shader_prim = UsdShade.Material(get_prim_at_path(material_path)).GetPrim()
shader_prim.GetAttribute("inputs:diffuse_texture").Set("/percorso/albedo_corteccia.png")
shader_prim.GetAttribute("inputs:normalmap_texture").Set("/percorso/normal_corteccia.png")

omni.kit.commands.execute(
    "BindMaterial",
    prim_path=["/World/Albero/Corteccia"],
    material_path=material_path,
    strength=UsdShade.Tokens.weakerThanDescendants,
)
```

Per generare varianti automatiche (utile per dataset di training/segmentazione, coerente con l'uso di YOLO già noto nel tuo workflow), `omni.replicator.core.create.material_omnipbr` consente di randomizzare diffuse/roughness/metallic/normal in batch, sebbene i parametri esposti siano limitati rispetto all'API MDL completa — per normal map serve pre-creare i materiali manualmente e poi passarli come lista al randomizer.[^17][^20]
## Stable Diffusion per la Texture di Corteccia/Foglie
Stable Diffusion è effettivamente usato nella pratica per generare texture PBR seamless, ma **non produce direttamente un materiale completo**: genera solo la mappa diffuse/albedo, e le altre mappe (normal, roughness, height, AO) vanno derivate con un secondo passaggio.
### Flusso operativo consigliato
1. **Generazione albedo con SD**: usare un modello o LoRA specializzato in texture (es. "Texture Diffusion" su Civitai) con prompt del tipo `pbr, tree bark texture, close-up, detailed, photo, real, high detail` e l'opzione "tiling/seamless" attivata nel webUI (Automatic1111) per garantire la ripetibilità della texture.[^21][^22]
2. **Refinement con img2img/ControlNet**: per un controllo maggiore su rughe e pattern, ControlNet con depth/scribble permette di guidare la struttura della corteccia mantenendo la seamless tiling.[^23]
3. **Derivazione delle mappe PBR**: da un singolo albedo si derivano normal/roughness/height/AO tramite tool dedicati come **Materialize** (gratuito, desktop) o **Substance 3D Sampler** (metodo B2M, più controllabile del modulo "AI powered" secondo i tester). Esistono anche alternative browser-based gratuite (GenPBR, 3DKit, AITEXTURED) che fanno lo stesso lavoro via Sobel/Scharr gradient sulla luminanza, utili per iterazione rapida senza installare software.[^24][^25][^26][^21][^23]
4. **Import in IsaacSim**: le mappe risultanti (albedo, normal, roughness) vengono collegate direttamente ai relativi `inputs:` di OmniPBR come mostrato nel codice sopra.

Un approccio più avanzato e ancora sperimentale è **Material Palette**, un metodo di ricerca che fine-tuna Stable Diffusion su crop di una singola immagine reale per generare texture del materiale e poi le decompone in SVBRDF complete (albedo, normal, roughness) tramite una rete dedicata — utile se hai foto reali di corteccia e vuoi un materiale procedurale coerente, ma è un progetto di ricerca, non un tool production-ready plug-and-play.[^27]
### Limitazioni
- Le mappe normal/roughness derivate da un singolo albedo con questi tool sono **stime euristiche basate sulla luminanza**, non misurazioni fisiche reali del materiale: buone per plausibilità visiva, non per accuratezza fotometrica.[^24][^25]
- La tiling/seamless è essenziale se la texture verrà applicata su UV ripetute lungo un tronco lungo; senza l'opzione "seamless" in SD il pattern mostrerà giunzioni visibili.[^22]
- Per foglie con trasparenza (alpha) e subsurface scattering, OmniPBR base non copre bene questi effetti — serve valutare shader MDL più avanzati o materiali translucent dedicati, non discussi in profondità nelle fonti disponibili qui.
## Sintesi Operativa (Cosa è Fattibile Oggi in IsaacSim)
| Obiettivo | Fattibile nativamente in IsaacSim? | Metodo raccomandato |
|---|---|---|
| Skinning scheletrico di un albero rigato | Sì, ma manuale/laborioso | UsdSkel + binding pesi scriptato in Python[^5][^2][^3] |
| Import rig da Blender/FBX | Parzialmente, con bug noti | Validare in usdview prima; considerare ricostruzione manuale del binding[^4] |
| Deformazione fisica di rami/tronco | Sì (beta) | Deformable Body (Volume/Surface) + subdivisione a monte + GPU Dynamics[^8][^9] |
| Superficie irregolare di corteccia | No, va fatto a monte | Blender: Voronoi/Noise → Bump + Displacement node[^11][^12] |
| Terreno irregolare | Sì (Isaac Lab) | Terrain generation procedurale con heightfield[^13][^14] |
| Materiale PBR base | Sì, nativo | OmniPBR via API Python o omni.kit.commands[^16][^18][^19] |
| Texture da Stable Diffusion | Sì, con passaggio esterno | SD per albedo seamless + Materialize/Substance Sampler per normal/roughness/height[^21][^23] |

---

## References

1. [OpenUSD/pxr/usdImaging/bin/testusdview/testenv/testUsdviewSkinning/arm.usda at dev · PixarAnimationStudios/OpenUSD](https://github.com/PixarAnimationStudios/OpenUSD/blob/dev/pxr/usdImaging/bin/testusdview/testenv/testUsdviewSkinning/arm.usda) - Universal Scene Description. Contribute to PixarAnimationStudios/OpenUSD development by creating an ...

2. [Universal Scene Description: Schemas In-Depth](https://openusd.org/release/api/_usd_skel__schemas.html)

3. [Skinning Schemas for USD](https://openusd.org/files/SkinningOM.md.html) - This document motivates the desire for, and scope of, core support for skeletal deformation and anim...

4. [FBX → USD Conversion Not Preserving Skeleton/Rigging](https://forums.developer.nvidia.com/t/fbx-usd-conversion-not-preserving-skeleton-rigging/351333) - Isaac Sim Version 5.1.0 Operating System Ubuntu 22.04 FBX → USD Conversion Not Preserving Skeleton/R...

5. [API Introduction](https://openusd.org/dev/api/_usd_skel__a_p_i__intro.html) - a UsdSkelSkinningQuery provides convenient API for reading data related to primitives that are skinn...

6. [UsdSkelCache Class Reference](https://openusd.org/dev/api/class_usd_skel_cache.html)

7. [Overview](https://openusd.org/release/api/_usd_skel__o_m.html)

8. [Deformable Body - Isaac Sim Tutorials - hijimasa.github.io](https://hijimasa.github.io/isaac-sim-tutorials/latest/core_api/09_deformable_body/) - Isaac Sim チュートリアル集 / Isaac Sim Tutorial Collection

9. [[Isaac Sim Tutorial - Core API] Lecture3 Hello Deformable Objects](https://www.youtube.com/watch?v=lnpb3DYyWxM) - 🧑🏻‍💻 Github Repository for this video
► https://github.com/kimsooyoung/rb_isaac_edu

⚙️ More about R...

10. [Deformable Body Visualization Issues - Isaac Sim](https://forums.developer.nvidia.com/t/deformable-body-visualization-issues/330384) - Isaac Sim Version 4.5.0 Operating System Ubuntu 20.04 GPU Information Model: NVIDIA GeForce RTX 4090...

11. [Procedural Tree Bark (Blender Tutorial)](https://www.youtube.com/watch?v=6ECeHoATa74) - In this Blender tutorial we will create this Procedural Tree Bark Material. ... ● Timestamps: 0:00 2...

12. [precedural Mossy Tree Bark in Blender](https://www.youtube.com/watch?v=2BQGWHKisKc) - Trees can be a really rough kind of thing to make, just slapping a brown color in a cylender doesn't...

13. [Terrain Generation | isaac-sim/IsaacLab | DeepWiki](https://deepwiki.com/isaac-sim/IsaacLab/4.7-terrain-generation) - The terrain system in Isaac Lab provides a robust framework for creating, importing, and managing si...

14. [isaaclab.terrains — Isaac Lab Documentation](https://isaac-sim.github.io/IsaacLab/main/source/api/lab/isaaclab.terrains.html)

15. [[Bug Report] Terrain Curriculum Features Not Working Properly · Issue #1685 · isaac-sim/IsaacLab](https://github.com/isaac-sim/IsaacLab/issues/1685) - If you are submitting a bug report, please fill in the following details and use the tag [bug]. Desc...

16. [[isaacsim.core.experimental.materials] Isaac Sim ... - NVIDIA](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/py/source/extensions/isaacsim.core.experimental.materials/docs/index.html)

17. [OmniPBR material assign normal map texture - Isaac Sim](https://forums.developer.nvidia.com/t/omnipbr-material-assign-normal-map-texture/250624) - I’m trying to do material randomization by creating new material via omni.replicator.core.create.mat...

18. [Changing material properties through python - Isaac Sim](https://forums.developer.nvidia.com/t/changing-material-properties-through-python/342850) - Isaac Sim Version 5.0.0 Operating System Windows 11 Hello, I am trying to create a material in Isaac...

19. [Overview](https://docs.omniverse.nvidia.com/kit/docs/omni.kit.material.library/1.5.6/Overview.html)

20. [Randomization Snippets - Isaac Sim Documentation](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/replicator_tutorials/tutorial_replicator_isaac_randomizers.html)

21. [OUTDATED | How to Make Seamless Textures with AI - Stable Diffusion Tutorial 2023](https://www.youtube.com/watch?v=TbsseP7Kc4k) - Poly: https://withpoly.com – use coupon 'albert' for 1 month free Poly Infinity!

Links:
Model: http...

22. [Create tileable PBR textures with Stable Diffusion (AI)](https://www.youtube.com/watch?v=_GSV8w6wd_E) - AI is all the rage right now, but I think this might actually be something useful for a lot of peopl...

23. [Seamless PBR materials with AI / Stable Diffusion and Substance 3D Sampler TUTORIAL](https://www.youtube.com/watch?v=DTzN2P_sW-4) - In this video you will learn how to make various weird (and not so weird) PBR texture sets, with sea...

24. [Normal map generator](https://genpbr.com/normal-map-generator) - Free normal map generator online. Upload PNG or diffuse, preview in 3D, export PNG—no signup for cor...

25. [Free Normal Map Generator | Seamless, Tileable PBR Textures](https://normalmap.ai/) - FREE Normal Map Online Generator. Create PBR Textures, SEAMLESS TEXTURE, and TRANSPARENT PNG online ...

26. [Normal Map Generator — Free Online PBR Maps from an Image](https://www.3d-editor.com/tools/normal-map-generator) - Free online normal map generator. Turn an image or heightmap into normal, height, AO and roughness m...

27. [Material Palette - Astra-vision](https://astra-vision.github.io/MaterialPalette/) - Extraction of PBR Materials from a Single Image

