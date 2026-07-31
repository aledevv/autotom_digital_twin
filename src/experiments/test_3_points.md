Va bene, te lo scrivo direttamente in chat come testo normale.

***

# Piano per LLM: Implementazione test di Three-Point Bending su stelo di pomodoro in Isaac Sim

## 1. Obiettivo

Adattare la pipeline esistente (cantilever test: `generate_cantilever_usda.py`, `run_cantilever.py`, `run_cantilever_manual.py`, `plot_deflection.py`) per riprodurre un **three-point bending test** su una catena di link rigidi collegati da joint D6, così da calibrare il modulo elastico E dello stelo di pomodoro confrontando la deflessione simulata con un target di deflessione misurato (o stimato da letteratura).

## 2. Differenza fondamentale rispetto al cantilever esistente

Il cantilever ha un solo vincolo (incastro alla base, link 1) e la forza applicata in punta (link 10). Il three-point bending ha **due vincoli agli estremi della campata** (appoggi semplici, non incastri) e la **forza applicata al centro** della campata. Le due configurazioni hanno formule di conversione forza-deflessione diverse:

- Cantilever: deflessione = (F × L³) / (3 × E × I)
- Three-point bending: deflessione = (F × L³) / (48 × E × I)

Il fattore 48 (anziché 3) riflette il fatto che una trave appoggiata su due punti è molto più rigida a parità di L, F e EI rispetto a una trave a sbalzo. Questa è l'equazione (1) usata sia da Anisimov et al. (2025) sia da Shtein et al. (2020), quindi è lo standard di riferimento da implementare.

## 3. Riferimenti scientifici da usare come fonte di verità

**Paper 1 — Anisimov, A. et al. (2025).** *"Using the Inverse Three-Point Bending Test to Determine Mechanical Properties of Plant Stems"*, Methods and Protocols, 8(2), 32. DOI: 10.3390/mps8020032.

Da questo paper vanno riprese:
- Equazione (1): E = (F × L³) / (48 × ΔL × I) — dove F è il carico, L è la distanza tra gli appoggi, ΔL è la deflessione misurata al centro, I è il momento di inerzia assiale.
- Equazione (2): I = π(D⁴ − d⁴)/64 per sezione cava; per stelo pieno d = 0, quindi I = πD⁴/64.
- Criterio geometrico: rapporto lunghezza/diametro della campata dovrebbe essere maggiore di 20 per evitare che gli effetti di taglio (shear) sottostimino il modulo elastico misurato. Questo è un vincolo diretto sulla geometria della simulazione.
- Valori sperimentali di riferimento su stelo erbaceo non lignificato (lino): sopra il "punto di scatto" (tessuto giovane, parete cellulare primaria, sostegno turgore-based): E = 34.9 ± 10.7 MPa. Sotto il punto di scatto (tessuto maturo, pareti secondarie sviluppate): E = 1546.7 ± 294.0 MPa. Coleoptile d'avena (tessuto primario puro): E = 20.4 ± 6.3 MPa. Stelo maturo completamente lignificato: E = 24.2 ± 2.5 GPa. Questi valori danno il range plausibile in cui deve cadere il tuo E calibrato per uno stelo di pomodoro non lignificato: **ordine di grandezza atteso 10–50 MPa per tessuto giovane, centinaia di MPa se più maturo, mai GPa a meno che lo stelo non sia legnoso.**

**Paper 2 — Shtein, I. et al. (2020).** *"Solanales Stem Biomechanical Properties Are Primarily Determined by Morphology Rather Than Internal Structural Anatomy and Cell Wall Composition"*, Plants, 9(6), 678. DOI: 10.3390/plants9060678.

Questo è il paper più rilevante per analogia tassonomica: confronta patata (Solanum tuberosum, stessa famiglia Solanaceae del pomodoro, pianta auto-portante come il pomodoro), patata dolce e ipomea (rampicante). Da qui vanno riprese:
- Il protocollo sperimentale esatto: three-point bending su macchina universale (Instron 5965), velocità di spostamento 1 mm/min, campata (span) di 80 mm per patata e 30 mm per le Ipomoea, rapporto campata/diametro (SDR) tra 7.3 e 21.1 giudicato accettabile per generare deflessione da trave classica (effetti di taglio trascurabili).
- Equazione (4) identica a quella di Anisimov: EB = kB × L³ / (48 × I), dove kB è la pendenza iniziale della curva forza-spostamento (stiffness strutturale).
- Risultato chiave: il modulo elastico "effettivo" (E, proprietà di materiale) varia poco tra specie diverse anche se la geometria dello stelo cambia di ordini di grandezza — è la geometria (diametro, quindi I), non il materiale, il fattore dominante nella rigidità strutturale finale. Questo giustifica l'approccio di tenere E in un range di letteratura stretto e concentrare la calibrazione sulla geometria reale del tuo stelo di pomodoro.
- **Limite importante da comunicare all'LLM**: non esiste nei due paper un valore numerico di E specifico per stelo (o ramo laterale) di pomodoro. La patata è l'analogo tassonomico più vicino disponibile (stessa famiglia, stesso habitus di crescita auto-portante), ma i valori vanno trattati come **prior plausibile, non come ground truth**. Il piano deve includere un passaggio di validazione con un esperimento fisico reale su un campione vero di stelo di pomodoro.

## 4. Specifiche geometriche da usare nella generazione del modello USD

- Mantenere la struttura a catena di link cilindrici rigidi già presente in `generate_cantilever_usda.py` (classe `TrunkConfig`), ma verificare che il rapporto lunghezza-totale-campata / diametro sia ≥ 20 (idealmente) o almeno ≥ 10 (limite minimo accettabile secondo Anisimov et al., citando Fok & Smart 1993). Con `RADIUS = 0.005 m` (diametro 10 mm), la campata L tra i due appoggi dovrebbe essere almeno 0.1–0.2 m per rispettare questo vincolo — verificare se è compatibile con la lunghezza reale del ramo laterale che si vuole modellare, altrimenti applicare una correzione per taglio (shear correction) nella formula finale.
- Il numero di link nella campata deve essere sufficiente a approssimare una curva continua (10 link, come nel cantilever, è un buon punto di partenza, ma valutare se aumentare a 12-16 per campate più lunghe).

## 5. Modifiche architetturali richieste al codice esistente

### 5.1 In `generate_cantilever_usda.py` (da rinominare `generate_threepoint_usda.py`)

- Sostituire `anchor_link_to_world` (fissaggio singolo alla base) con **due punti di appoggio**: uno vicino al primo link, uno vicino all'ultimo link della campata.
- Ogni appoggio deve essere un **simple support**, non un incastro: blocca solo la traslazione verticale (asse Z, o l'asse di applicazione della forza), lascia liberi rotX/rotY (altrimenti si simula una trave doppiamente incastrata, che ha un fattore diverso da 48 nella formula, invalidando il calcolo).
- Il drive elastico (stiffness/damping da `calculate_physics_params`) va applicato solo ai joint D6 **interni** alla catena (quelli tra i link), non ai due joint di appoggio, che devono restare vincoli rigidi in traslazione ma liberi in rotazione.
- Aggiungere un parametro `SPAN_LENGTH` esplicito nella configurazione, distinto dalla lunghezza totale della catena, per poter modellare eventuali sbalzi oltre gli appoggi (nel three-point bending puro, gli appoggi coincidono con le estremità, quindi `SPAN_LENGTH` = lunghezza totale della catena).
- **Correggere il tipo di drive**: passare da drive di tipo `"force"` a `"acceleration"` nel `DriveAPI` (problema già identificato nella sessione precedente: con link leggeri, il force drive richiede stiffness irrealisticamente alte per compensare l'inerzia ridotta).

### 5.2 In `run_cantilever.py` (da rinominare `run_threepoint.py`)

- Identificare il link centrale della campata (indice N_LINKS // 2) come punto di applicazione della forza, non l'ultimo link.
- Applicare la forza in direzione verticale (asse Z, gravità) al link centrale, con una rampa di carico progressiva a step (come nel protocollo reale del paper Anisimov: step di spostamento di 125-250 μm, pausa di 30 s tra step) invece di una singola rampa continua, per replicare più fedelmente il protocollo sperimentale di riferimento.
- Registrare per ogni step: forza applicata F, deflessione verticale del link centrale ΔL (misurata rispetto alla linea che congiunge i due appoggi, non rispetto alla sola posizione iniziale — verificare che gli appoggi non abbiano ceduto, altrimenti sottrarre il loro spostamento).
- Alla fine della sequenza di carico, calcolare la pendenza iniziale della curva F-ΔL (stiffness strutturale kB) tramite regressione lineare sui primi punti della curva (dove la risposta è ancora lineare-elastica), e applicare l'equazione EB = kB × L³ / (48 × I) per ottenere il modulo elastico simulato.
- Mantenere la modalità `CALIBRATE` esistente ma adattarla: invece di cercare E che produce una deflessione target a forza fissa, cercare E che produce la stiffness strutturale kB target (più robusto, perché indipendente dal singolo punto di carico usato).

### 5.3 In `plot_deflection.py`

- Aggiungere un grafico Forza vs Deflessione (curva a step, come Figura 1 di Shtein et al. 2020) oltre al grafico deflessione-vs-tempo già esistente, per poter visualizzare la pendenza lineare iniziale usata per calcolare kB.

## 6. Protocollo di validazione a due livelli

**Livello 1 — validazione contro letteratura (ordine di grandezza):** far girare la simulazione con E impostato ai valori di riferimento del paper Anisimov (20-50 MPa per tessuto primario, 1500+ MPa per tessuto secondario) e verificare che la deflessione simulata sia nell'ordine di grandezza plausibile per la geometria del proprio stelo di pomodoro, senza dover ricorrere a valori assurdi (GPa).

**Livello 2 — validazione contro esperimento fisico reale (obbligatorio per un valore affidabile):** eseguire un vero three-point bending su un campione reale di ramo di pomodoro, replicando il protocollo di Shtein et al. (span 30-80 mm a seconda del diametro del ramo, rapporto campata/diametro tra 10 e 20, carico applicato con pesi noti o dinamometro, misura della deflessione con calibro o fotocamera), poi usare quel valore di kB reale come target per la calibrazione automatica in Isa