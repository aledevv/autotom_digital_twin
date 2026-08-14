Certo — lo riscrivo meglio, includendo **come sono fatti gli esperimenti dei paper**, così capisci quanto sono vicini o lontani dal tuo caso di digital twin.

## Paper / fonti consultate e contesto sperimentale

| Fonte                                                                                                                                        | Esperimento / contesto                                                                                                                                                                                                                                                                                                                                       | Cosa mi serve per il tuo modello                                                                                                                                                                    |
| -------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Gao et al., 2024 — “Discrete Element Model Building and Optimization of Tomato Stalks at Harvest”**                                        | Studio su **stalk di pomodoro raccolti a harvest**. Misurano proprietà fisiche e meccaniche dei fusti per costruire un modello DEM. I campioni sono segmenti di stalk, non una pianta viva intera. Il paper misura diametro esterno, spessore del bast, densità, contenuto d’acqua, modulo elastico e modulo di taglio tramite prove meccaniche. ([MDPI][1]) | È la fonte più utile per avere **valori numerici iniziali** per tomato stem/stalk: diametro, densità, (E), (G). Buona per ordine di grandezza, non per ground truth assoluta della tua pianta viva. |
| **Coutand et al., 2000 — “Biomechanical study of the effect of a controlled bending on tomato stem elongation: global mechanical analysis”** | Esperimento su **steli di pomodoro vivi** sottoposti a bending controllato. Il paper applica una deformazione/mechanical stimulus allo stelo e misura l’effetto sulla crescita/elongazione, con analisi globale di momento, curvatura e variabili meccaniche. ([PubMed][2])                                                                                  | Serve a giustificare che il pomodoro può essere trattato con un modello biomeccanico di bending, ma non lo uso direttamente per valori semplici da cantilever.                                      |
| **Coutand & Moulia, 2000 — “local strain sensing and spatial integration of the signal”**                                                    | Seconda parte dello studio sul pomodoro. Si concentra su come lo stelo percepisce localmente lo strain e integra spazialmente il segnale meccanico per modificare la crescita. ([PubMed][3])                                                                                                                                                                 | Serve a ricordare che un digital twin biologico non è solo trave elastica passiva: la pianta viva può modificare crescita e proprietà meccaniche in risposta al bending.                            |
| **Martin-Nelson et al., 2021 — “Axial variation in flexural stiffness of plant stem segments”**                                              | Studio metodologico sulla misura della **flexural stiffness (EI)** lungo segmenti/internodi di steli vegetali. Non è specifico sul pomodoro. Confronta metodi di misura della variazione assiale della rigidezza e discute l’amplificazione dell’errore quando si stimano segmenti singoli. ([Springer][4])                                                  | Serve a supportare l’idea che (EI) può variare lungo lo stelo e che bisogna stimarlo con attenzione, soprattutto se vuoi valori per internodo.                                                      |
| **Bergou et al., 2008 — “Discrete Elastic Rods”**                                                                                            | Paper computazionale su come rappresentare aste elastiche continue con modelli discreti. Non è biologico, ma è molto rilevante per passare da beam/rod continuo a link + hinge/joint.                                                                                                                                                                        | Serve per la logica teorica: il numero di segmenti deve essere una discretizzazione, non una proprietà fisica.                                                                                      |
| **OpenUSD / PhysX documentation**                                                                                                            | Documentazione tecnica su `UsdPhysicsDriveAPI`, D6 joints, solver, unità, angular drives, stiffness e damping.                                                                                                                                                                                                                                               | Serve perché nel tuo codice i D6 drive sono PhysX/USD. Punto critico: gli angular drive USD usano **degrees**, non radians.                                                                         |

## Fonte più utile per valori numerici: Gao et al. 2024

Il paper di Gao et al. è quello da cui si possono tirare fuori i numeri più direttamente usabili per un **cantilever sanity test**.

Però il contesto è importante:

non è una misura su un ramo vivo nella tua scena, ma su **tomato stalk harvested**, cioè fusti raccolti a maturità/harvest, pensati per modellazione DEM e interazione meccanica con macchinari agricoli. Quindi i valori sono ottimi per partire, ma non sono automaticamente “il valore vero” del tuo digital twin.

I valori utili riportati sono:

```text
Outer diameter d_o      ≈ 11.1 mm
Bast thickness          ≈ 3.64 mm
Density rho             ≈ 769.96 kg/m³
Young's modulus E       ≈ 50.64 MPa
Shear modulus G         ≈ 21.09 MPa
Moisture content        ≈ 75.9 %
```

## Come ho ricavato i valori attesi per il cantilever

Per un cantilever test semplice, considero uno stelo/ramo uniforme, orizzontale, incastrato alla base.

La relazione classica è:

[
\delta_\text{tip} = \frac{F L^3}{3EI}
]

dove:

```text
δ_tip = deflessione della punta
F     = forza applicata in punta
L     = lunghezza del cantilever
E     = modulo elastico
I     = secondo momento d’area
EI    = rigidezza flessionale
```

Dato che Gao et al. riportano anche lo spessore del bast, una stima più coerente è trattare lo stalk come **sezione cava**, non piena.

Uso quindi:

[
I = \frac{\pi(d_o^4 - d_i^4)}{64}
]

con:

```text
d_o = 11.1 mm
t   = 3.64 mm
d_i = d_o - 2t = 3.82 mm
```

Da cui viene circa:

```text
I  ≈ 7.35e-10 m⁴
EI ≈ 0.0372 N·m²
```

Questo (EI) è il valore principale che userei nel test.

## Valori attesi per cantilever con forza in punta

Assumendo:

```text
E  = 50.64 MPa
d_o = 11.1 mm
d_i = 3.82 mm
EI ≈ 0.0372 N·m²
```

ottieni questi valori attesi:

| Lunghezza cantilever | F = 0.01 N | F = 0.05 N | F = 0.10 N |
| -------------------: | ---------: | ---------: | ---------: |
|                 5 cm |   0.011 mm |   0.056 mm |   0.112 mm |
|                 8 cm |   0.046 mm |   0.229 mm |   0.459 mm |
|                10 cm |   0.090 mm |   0.448 mm |   0.896 mm |
|                15 cm |   0.302 mm |   1.512 mm |   3.024 mm |
|                20 cm |   0.717 mm |   3.584 mm |   7.167 mm |
|                30 cm |   2.419 mm |  12.095 mm |  24.189 mm |

Questi sono valori **molto utili per debug**, perché ti dicono l’ordine di grandezza. Per esempio, se fai:

```text
L = 20 cm
F = 0.05 N
```

allora ti aspetti una deflessione tip di circa:

```text
δ_tip ≈ 3.6 mm
```

Se il tuo modello con 3 link dà 20 mm e quello con 10 link dà 1 mm, allora non sta rappresentando lo stesso ramo.

## Valori attesi sotto solo peso proprio

Se il cantilever è orizzontale e lasci solo la gravità, puoi stimare la deflessione da carico distribuito:

[
\delta_\text{self-weight} = \frac{w L^4}{8EI}
]

dove:

[
w = \rho A g
]

Con la sezione cava sopra viene circa:

```text
w ≈ 0.644 N/m
```

e quindi:

| Lunghezza cantilever | Deflessione da peso proprio |
| -------------------: | --------------------------: |
|                 5 cm |                    0.014 mm |
|                 8 cm |                    0.089 mm |
|                10 cm |                    0.216 mm |
|                15 cm |                    1.096 mm |
|                20 cm |                    3.464 mm |
|                30 cm |                   17.535 mm |

Quindi un test molto pratico è:

```text
Cantilever orizzontale
L = 20 cm
solo gravità
EI ≈ 0.0372 N·m²
deflessione attesa ≈ 3.5 mm
```

Questo è comodo perché non devi applicare forze esterne.

## Valori attesi per stiffness dei D6

Per collegare il cantilever al tuo modello D6:

[
k_{\theta,\text{rad}} = \frac{EI}{\ell}
]

dove (\ell) è la lunghezza rappresentata dal joint.

Poi, siccome USD usa angular stiffness per **degree**:

[
k_\text{USD} = k_{\theta,\text{rad}} \frac{\pi}{180}
]

Per un ramo/stelo lungo:

```text
L = 20 cm
EI ≈ 0.0372 N·m²
```

ottieni:

| N link circa | lunghezza segmento | (k_\theta) N·m/rad | `DriveAPI stiffness` N·m/deg |
| -----------: | -----------------: | -----------------: | ---------------------------: |
|            3 |            6.67 cm |              0.558 |                      0.00974 |
|            5 |            4.00 cm |              0.930 |                      0.01623 |
|           10 |            2.00 cm |              1.860 |                      0.03247 |
|           20 |            1.00 cm |              3.721 |                      0.06494 |

Il punto chiave è questo:

```text
se N aumenta,
la stiffness numerica del singolo joint aumenta,
ma la flessibilità globale del ramo dovrebbe restare simile.
```

Quindi per un digital twin non dovresti confrontare:

```text
3 link con k = 0.03
10 link con k = 0.03
```

ma piuttosto:

```text
3 link con k ≈ 0.0097 N·m/deg
10 link con k ≈ 0.0325 N·m/deg
```

perché stanno rappresentando lo stesso (EI).

## Test minimo consigliato

Io farei questi tre test, in ordine.

### Test A — cantilever con forza nota

```text
Geometry:
  L = 0.20 m
  d_o = 0.0111 m
  d_i = 0.00382 m

Material:
  E = 50.64 MPa
  rho = 769.96 kg/m³

Physics:
  gravity off
  tip force F = 0.05 N

Expected:
  tip deflection ≈ 3.58 mm
```

Poi ripeti con:

```text
N = 3, 5, 10, 20
```

ricalcolando ogni volta la stiffness dei D6.

Risultato desiderato:

```text
N = 3   → magari errore visibile, ma stesso ordine di grandezza
N = 5   → più vicino
N = 10  → vicino
N = 20  → quasi convergente
```

### Test B — cantilever sotto gravità

```text
Geometry/material same as above
Gravity on
No external force
Horizontal cantilever

Expected:
  tip deflection ≈ 3.46 mm
```

Questo è molto utile perché assomiglia di più al caso ramo reale sotto peso proprio.

### Test C — stiffness D6 convergence

Per ogni (N), controlla che la stiffness per joint sia:

```text
N = 3   → k_USD ≈ 0.0097 N·m/deg
N = 5   → k_USD ≈ 0.0162 N·m/deg
N = 10  → k_USD ≈ 0.0325 N·m/deg
N = 20  → k_USD ≈ 0.0649 N·m/deg
```

Se facendo così la deflessione resta simile, la discretizzazione è corretta.

Se invece 10 link è ancora molto più duro o molto più morbido di 3, allora il problema probabilmente è uno di questi:

```text
- unità USD sbagliate
- stiffness ancora interpretata in rad invece che degree
- lunghezza totale che cambia con N
- massa totale che cambia con N
- collisioni tra link
- limiti angolari che entrano in gioco
- solver PhysX non convergente
- timestep/substeps insufficienti
- inertie o mass ratio problematici
```

## In sintesi

I valori attesi più utili da portarti dietro sono questi:

```text
Tomato stalk reference:
  E ≈ 50.64 MPa
  rho ≈ 770 kg/m³
  d_o ≈ 11.1 mm
  bast thickness ≈ 3.64 mm
  EI ≈ 0.0372 N·m²

Cantilever L = 20 cm:
  F = 0.05 N, gravity off → δ_tip ≈ 3.58 mm
  only self-weight, gravity on → δ_tip ≈ 3.46 mm

D6 stiffness for same L = 20 cm:
  N = 3  → k_USD ≈ 0.0097 N·m/deg
  N = 10 → k_USD ≈ 0.0325 N·m/deg
  N = 20 → k_USD ≈ 0.0649 N·m/deg
```

Questi non sono valori “finali” del tuo digital twin. Sono un **benchmark iniziale** per vedere se il modello fisico è coerente. Una volta che questo test converge, allora puoi calibrare (EI) sui tuoi dati reali di pianta, varietà, età, internodo, diametro e stato idrico.

[1]: https://www.mdpi.com/2077-0472/14/4/531?utm_source=chatgpt.com "Discrete Element Model Building and Optimization of Tomato Stalks at Harvest"
[2]: https://pubmed.ncbi.nlm.nih.gov/11113160/?utm_source=chatgpt.com "Biomechanical study of the effect of a controlled bending on tomato stem elongation: global mechanical analysis - PubMed"
[3]: https://pubmed.ncbi.nlm.nih.gov/11113161/?utm_source=chatgpt.com "Biomechanical study of the effect of a controlled bending on tomato stem elongation: local strain sensing and spatial integration of the signal - PubMed"
[4]: https://link.springer.com/article/10.1186/s13007-021-00793-8?utm_source=chatgpt.com "Axial variation in flexural stiffness of plant stem segments: measurement methods and the influence of measurement uncertainty | Plant Methods | Springer Nature Link"
