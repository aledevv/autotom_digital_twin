# Modellazione biomeccanica dello stelo di pomodoro per calibrazione stiffness in Isaac Sim

## Sintesi del problema riscontrato nel codice allegato

Nel tuo `generate_cantilever_usda.py` il modulo elastico `E` viene usato correttamente nella formula di Eulero-Bernoulli \( K = \dfrac{E I}{L} \), dove \(I = \dfrac{\pi r^4}{4}\) è il momento di inerzia della sezione circolare. Questa relazione è **direttamente proporzionale**, non inversa: aumentando `E` aumenta la stiffness `K` del drive del joint, e quindi *diminuisce* la deflessione a parità di forza. Se durante la calibrazione automatica (`run_cantilever.py`, modalità `CALIBRATE`) hai osservato che per ottenere la deflessione target dovevi alzare `E` fino a valori da acciaio/tungsteno (decine di GPa), il problema più probabile non è l'inversione della formula ma uno di questi tre fattori concorrenti, tutti tipici in setup a catena di joint D6 in PhysX/Isaac Sim.[^1]

## Perché ti servono E enormi: la vera causa

### 1. Effetto catena di joint in serie (compliance additiva)

Quando N link sono collegati da N-1 joint D6 con drive elastico, la deflessione totale non è quella di una singola trave continua di lunghezza L, ma la somma delle rotazioni concesse da ciascun joint discreto. Un drive "force" di tipo PD in PhysX applica \( \tau = K_{stiff}(\theta_{target}-\theta) + K_{damp}(\dot\theta_{target}-\dot\theta) \), quindi il tuo joint stiffness rotazionale equivalente deve essere derivato correttamente dalla rigidità flessionale continua **EI** discretizzata sull'elemento di lunghezza \(L_{link}\), tipicamente come \( K_\theta = \dfrac{EI}{L_{link}} \) per joint rotazionali (non traslazionali). Se il tuo calcolo tratta il drive come lineare (stiffness in N/m anziché N·m/rad) applicato su un asse angolare, l'unità di misura è sbagliata e il valore numerico di stiffness richiesto esplode, costringendoti a compensare gonfiando E in modo non fisico.[^2][^3]

### 2. Iterazioni del solver e drive impliciti PhysX

I drive D6 di PhysX sono spring impliciti risolti dal solver TGS; con un numero di link elevato (10 nel tuo caso) e posizioni-iterazioni insufficienti, l'errore di rigidità percepita si accumula lungo la catena, richiedendo stiffness molto più alte del valore teorico per ottenere la stessa deflessione macroscopica di una trave continua equivalente. La documentazione NVIDIA conferma che il drive stiffness di default nei DcD6JointProperties è dell'ordine di 1e5, e viene scalato in base a massa/inerzia dei link, quindi il valore "fisicamente corretto" derivato da un E di 150 MPa (ordine di grandezza reale per steli erbacei) può risultare visivamente "troppo cedevole" nel solver se le iterazioni sono poche o il timestep è largo.[^4][^5]

### 3. Target di deflessione irrealistico o mal misurato

Il tuo `TARGET_DEFLECTION_MM = 7.6` andrebbe verificato: se questo valore proviene da un esperimento reale ipotizzato (non misurato), la calibrazione automatica troverà semplicemente il valore di E che fa "quadrare" un target sbagliato, spiegando perché il risultato finale è irrealistico. Prima di ottimizzare il codice, va rifatto o validato l'esperimento fisico di riferimento.

## Valori reali di rigidità per steli di pomodoro dalla letteratura

La ricerca botanica su *Solanum lycopersicum* fornisce valori concreti utili per la calibrazione, distinti per tipo di tessuto e metodo di misura.

| Fonte | Materiale testato | Metodo | Modulo elastico misurato |
|---|---|---|---|
| Kuna-Broniowska et al. 2011 [^6] | Buccia (skin) frutto pomodoro | Trazione | 3.7–7.4 MPa |
| Wang et al. (Determination of elastic properties AFM) [^7] | Cellule mesocarpo frutto | Indentazione AFM | Ordine kPa-MPa a scala cellulare |
| Hiwasa/mch062 studio pareti cellulari | Cellule sospese in coltura | Micromanipolazione | ~7 MPa (parete cellulare) |
| Shah, Reynolds & Ramage 2017 [^1] | Rassegna steli erbacei in generale | Bending 2/3/4-point | Parenchima: 0.001–4 GPa; sclerenchima/xylema: 10–35 GPa |

Questi dati chiariscono un punto cruciale: **il modulo elastico di uno stelo erbaceo intero non è quello del singolo tessuto**, ma una media pesata tra il "guscio" esterno rigido (collenchima/sclerenchima, GPa) e il "core" midollare cedevole (parenchima, sub-GPa, dominato dalla pressione di turgore). Per uno stelo di pomodoro giovane, non lignificato, il modulo elastico apparente a livello macroscopico (bending dell'intero stelo, non del singolo tessuto) si colloca tipicamente nell'ordine delle decine di MPa, non centinaia di MPa né tantomeno GPa — un valore comparabile ai 150 MPa già impostato nel tuo `BioConfig.YOUNG_MODULUS`, che quindi era plausibile come punto di partenza.[^6][^1]

## Metodo sperimentale raccomandato: test di bending a sbalzo (cantilever)

La revisione più autorevole sul tema, Shah, Reynolds & Ramage (Journal of Experimental Botany, 2017), descrive nel dettaglio protocolli di flexural bending (a 2, 3 e 4 punti) applicabili a steli erbacei, incluso il caso a sbalzo che stai già implementando. Punti chiave da applicare al tuo setup:[^1]

- **Turgore**: la rigidità di uno stelo fresco dipende fortemente dalla pressione di turgore interna (0.1–2 MPa), che varia entro 15-25 minuti dal taglio, causando fino al 10% di calo di stiffness. Se il tuo esperimento fisico di riferimento non specifica il tempo trascorso dal taglio, introduce un errore sistematico enorme.[^1]
- **Sezione trasversale**: l'effetto della quarta potenza del raggio nel calcolo di \(I\) amplifica errori di misura del diametro; un errore del 10% sul raggio produce circa il 46% di errore su \(I\), quindi sulla stiffness derivata.[^1]
- **Metodo dei tre punti come alternativa più robusta**: un lavoro più recente (2025) propone un dispositivo di three-point bending inverso specificamente validato su steli di piante, capace di misurare moduli elastici da decine di MPa a decine di GPa con alta sensibilità — utile se vuoi affiancare al cantilever un secondo metodo di verifica incrociata.[^8]
- **Metodo della flessione assiale variabile**: un metodo più recente per la misura della variazione assiale della rigidità flessionale (EI) lungo lo stelo suggerisce di misurare EI direttamente (senza separare E e I) tramite regressione della curva di deflessione, un approccio che elimina l'incertezza sulla misura del raggio se applicato correttamente.[^9]

Per il tuo caso pratico, la formula da riprodurre sperimentalmente per un cantilever con carico puntuale all'estremità è:

\[ \delta = \frac{F L^3}{3 E I} \]

dove \(\delta\) è la deflessione in punta, \(F\) la forza applicata, \(L\) la lunghezza dello stelo (o della porzione a sbalzo), e \(EI\) la rigidità flessionale. Notare che questa è la formula per una **trave continua**, mentre la tua simulazione usa una catena discreta di 10 link — la conversione tra le due richiede attenzione, spiegata nella sezione successiva.[^10]

## Conversione corretta da EI continuo a stiffness del joint D6 discreto

Il punto tecnico più critico per il tuo caso è la trasformazione della rigidità flessionale continua \(EI\) (proprietà del materiale/sezione) nella rigidità del singolo drive angolare del joint D6 (proprietà del giunto discreto). Per una catena di N segmenti rigidi di lunghezza \(L_{link} = L/N\) che approssima una trave continua di rigidità EI, la stiffness rotazionale equivalente di ciascun joint deve essere:

\[ K_{\theta} = \frac{EI}{L_{link}} \]

e va applicata come **drive rotazionale** (unità: N·m/rad), non come drive traslazionale lungo `transX/Y/Z` come attualmente configurato in parte del tuo `configure_joint_drives`. Nel tuo codice il drive stiffness/damping viene applicato sugli assi `rotX` e `rotY` (corretto), ma verifica che l'unità risultante da `calculate_physics_params` sia effettivamente N·m/rad e non N/m: la formula `K = (E*I)/length` restituisce N·m/rad solo se `I` è in m^4 e `length` in metri, il che sembra corretto nel tuo script — quindi il problema è probabilmente a valle, nel modo in cui PhysX applica il drive (force vs acceleration drive) o nel numero di iterazioni del solver, non nella formula stessa.[^3][^2]

Il documento ufficiale PhysX sui drive impliciti conferma che la forza del drive è governata da:

\[ F = K_{stiff}(\theta_{target} - \theta) + K_{damp}(\dot\theta_{target} - \dot\theta) \]

e raccomanda l'uso di **acceleration drive** (che tiene conto della massa/inerzia del corpo) invece di **force drive** quando si lavora con catene di link leggeri come i tuoi trunk cilindrici da pochi grammi, perché il force drive richiede stiffness molto più alte per compensare l'inerzia ridotta, portando esattamente all'artefatto "serve un E enorme" che hai osservato.[^5][^3]

## Raccomandazioni pratiche per correggere il codice

1. **Passa da drive di tipo "force" a "acceleration"** nel `DriveAPI` (`drive.CreateTypeAttr().Set("acceleration")`), così la stiffness è normalizzata rispetto a massa/inerzia dei link e i valori di E richiesti torneranno nell'ordine fisico corretto (decine di MPa).[^3][^5]
2. **Verifica le unità del drive**: assicurati che lo stiffness/damping calcolato da `calculate_physics_params` sia assegnato esclusivamente agli assi rotazionali (`rotX`/`rotY`) e non anche a `transX/Y/Z`, dove attualmente stai solo bloccando i limiti (corretto) ma dovresti confermare che nessun drive residuo sia applicato lì.
3. **Aumenta le iterazioni di solver posizione** (`SolverPositionIterationCountAttr`) se non l'hai già fatto — il tuo script imposta già 128, un valore alto, quindi questo probabilmente non è la causa principale, ma vale la pena testare con 256 per escludere accumulo di errore lungo la catena a 10 link.
4. **Rifai l'esperimento fisico con un protocollo tracciabile**: misura raggio medio dello stelo (con calibro, in almeno 3 punti), lunghezza del tratto a sbalzo, forza applicata (dinamometro o massa nota appesa), e tempo trascorso dal taglio, seguendo il protocollo di Shah et al.  per minimizzare l'effetto turgore.[^1]
5. **Usa un E di partenza realistico**: 10–50 MPa per uno stelo di pomodoro giovane non lignificato (parenchima dominante), fino a 100–200 MPa se lo stelo è più maturo/lignificato con più tessuto sclerenchimatico, basandoti sui range di Shah et al. e Kuna-Broniowska et al.. Se dopo la correzione del drive (punto 1) continui a richiedere E nell'ordine dei GPa, il problema è nella conversione EI→K_joint o nella scala geometrica del modello, non nel valore di E stesso.[^6][^1]
6. **Valida con un secondo metodo**: applica anche un three-point bending virtuale sulla stessa catena di link, confrontando il valore di E stimato con quello del cantilever; se i due convergono, la calibrazione è robusta.[^8][^9]

## Nota sul benchmark e sui parametri drive D6 in Isaac Sim

I parametri di default documentati per un joint D6 in Isaac Sim (`DcD6JointProperties`) sono stiffness = 1e5, damping = 1e3, limitStiffness = 1e5, limitDamping = 1e3 — questi sono valori generici per robotica rigida, non calibrati per materiali biologici, e vanno sempre sovrascritti con i valori derivati da `EI/L_link` per applicazioni bio-meccaniche come la tua. Il forum NVIDIA conferma inoltre un problema comune: se la massa dei link viene calcolata automaticamente in modo errato dal generatore di stage (es. densità sbagliata o volume mal calcolato), il drive risulterà instabile o richiederà stiffness anomale indipendentemente dal valore di E, un altro punto da verificare nel tuo `compute_mass` (controlla che `PLANT_DENSITY = 1000.0` kg/m³ sia realistico per un tessuto vegetale turgido, che tipicamente è vicino alla densità dell'acqua, quindi il valore attuale sembra corretto).[^11][^5]

## Prossimi passi consigliati

Prima di rilanciare la calibrazione automatica, converti il drive da "force" ad "acceleration" e ripeti il test con E fissato a 20 MPa: se la deflessione simulata si avvicina all'ordine di grandezza del target reale senza dover alzare E oltre le centinaia di MPa, la causa principale era la modellazione del drive PhysX, non la formula di Eulero-Bernoulli. In parallelo, ripeti l'esperimento fisico reale con un protocollo controllato (raggio in 3 punti, tempo dal taglio noto, forza nota) per avere un target di deflessione affidabile su cui calibrare, invece di un valore ipotizzato.

---

## References

1. [run_cantilever.py](https://ppl-ai-file-upload.s3.amazonaws.com/web/direct-files/attachments/33031087/d1f45e58-d518-4fae-90e1-b7872c5b745b/run_cantilever.py?AWSAccessKeyId=ASIA2F3EMEYEWI45WWRN&Signature=ou9DreTiuTf4OFl1wPnswzlTdAM%3D&x-amz-security-token=IQoJb3JpZ2luX2VjEOP%2F%2F%2F%2F%2F%2F%2F%2F%2F%2FwEaCXVzLWVhc3QtMSJGMEQCIEtPZaz7WGJ9YCiA8lJdBPq%2Fh%2FuSufJ2NpaUWnTXtQQFAiBx2H8b%2BgkUNFAwl%2BcdAnDZE%2FaSwa3wrEO%2BPi130mlOsSr8BAis%2F%2F%2F%2F%2F%2F%2F%2F%2F%2F8BEAEaDDY5OTc1MzMwOTcwNSIMxYihoE5lwHwo07%2FnKtAE6ttNasT%2BwFDQZ7Yjz0eu9gqU2xIvIBfKTYsNnHBEmAGRPg2UW8sT7Y28AnGC2MSnW3lTBJO%2Farb%2F7RCW3%2B%2FVJtIORIIeqNNI3W4GCOcHiUZC1zXz5Yvi8htdr7x7NWR2HgcMAziAFxSF4fUgAht1LL3U%2FXV90Y%2FGxmq7Pl%2FUCuoqjRRZJlrU%2FKeSFzOw4fTnG5ltb0wgNaWhB2DfFULUFjwBEwhBi5kuy6XXPBwB7FOfjoutSRnPaA3stq7wH%2FlBleFh5HFeAaCtCVnDaQWm6uuS2e7DkzW7PQcZvjaqJdUZ5qZ2gefJz6UMKzkSIXbAyvyNp3hZnc7Tsl49ybdu4yijKj57EquUKo0a%2B8RmRAgXzm7fPRCtFO3Fw7tf2%2BRR0sIuGGyZReRLLkxHhoWvfsxPOnndChP2gBLhTawg2EfsKQqyRu7MNbKhrHP2QspNF9MZJFQ8Y0MwTMfrgi7uX9HoL%2FLsajDoY6sw%2BMxl8Cxqv%2FMN3Fh09m3TmbnxPjniU8xteoF3kYCaVnbDNw7feAnA%2BG0qGf%2BBLSlyuE9UxKqRiwTUDLjGdSOsTq6ux%2B1T3B7hi1REOMyRKQf2xMScVVRRtxmpPhOxVO9%2BRCZU1gwaKNuAwdRD%2FfqM8ns7Qo0OFPI6JiOPoYdvIFkzILT66w2JDnu%2Bypr1IS%2BRUCbHGdv2SFg35ZMMC9pn457GbCbKD6trJouIXHftjvEt09%2FNcKBxg%2Bb0AMK5NbItT0O195Tfxxqzu%2FmB8HgPVKYSixkiaZ8BdYCpxbPqZTLLmF3jGDCm9rHTBjqZAR%2F63%2F1dZe7lG6t0hIJyzatCaF0lyZiycfezv%2FQZMYZrNeWvK1i0430MWgqipv0FB6tEyG7JR02C7%2FIp4fGRBPznW0wSZXjTgeS%2B8y7yoQ6wMsyIZz7PEI9dTzNmmI86auzxuObfYXDRD3OgLGT5WRxg%2BRqMV0sqhzWnvHt%2F%2FvXgaTIR3V6EAlYOyXhTxMCJ4%2F0pPQxTvOo%2Fiw%3D%3D&Expires=1785497849) - import os
import sys
import numpy as np
import time
import csv

# Scegli la modalità di esecuzione:
...

2. [plot_deflection.py](https://ppl-ai-file-upload.s3.amazonaws.com/web/direct-files/attachments/33031087/0ef266e1-798e-48fd-a199-54e5f3936cf8/plot_deflection.py?AWSAccessKeyId=ASIA2F3EMEYEWI45WWRN&Signature=odKcjchTiSAIatyIYWuyfePSawI%3D&x-amz-security-token=IQoJb3JpZ2luX2VjEOP%2F%2F%2F%2F%2F%2F%2F%2F%2F%2FwEaCXVzLWVhc3QtMSJGMEQCIEtPZaz7WGJ9YCiA8lJdBPq%2Fh%2FuSufJ2NpaUWnTXtQQFAiBx2H8b%2BgkUNFAwl%2BcdAnDZE%2FaSwa3wrEO%2BPi130mlOsSr8BAis%2F%2F%2F%2F%2F%2F%2F%2F%2F%2F8BEAEaDDY5OTc1MzMwOTcwNSIMxYihoE5lwHwo07%2FnKtAE6ttNasT%2BwFDQZ7Yjz0eu9gqU2xIvIBfKTYsNnHBEmAGRPg2UW8sT7Y28AnGC2MSnW3lTBJO%2Farb%2F7RCW3%2B%2FVJtIORIIeqNNI3W4GCOcHiUZC1zXz5Yvi8htdr7x7NWR2HgcMAziAFxSF4fUgAht1LL3U%2FXV90Y%2FGxmq7Pl%2FUCuoqjRRZJlrU%2FKeSFzOw4fTnG5ltb0wgNaWhB2DfFULUFjwBEwhBi5kuy6XXPBwB7FOfjoutSRnPaA3stq7wH%2FlBleFh5HFeAaCtCVnDaQWm6uuS2e7DkzW7PQcZvjaqJdUZ5qZ2gefJz6UMKzkSIXbAyvyNp3hZnc7Tsl49ybdu4yijKj57EquUKo0a%2B8RmRAgXzm7fPRCtFO3Fw7tf2%2BRR0sIuGGyZReRLLkxHhoWvfsxPOnndChP2gBLhTawg2EfsKQqyRu7MNbKhrHP2QspNF9MZJFQ8Y0MwTMfrgi7uX9HoL%2FLsajDoY6sw%2BMxl8Cxqv%2FMN3Fh09m3TmbnxPjniU8xteoF3kYCaVnbDNw7feAnA%2BG0qGf%2BBLSlyuE9UxKqRiwTUDLjGdSOsTq6ux%2B1T3B7hi1REOMyRKQf2xMScVVRRtxmpPhOxVO9%2BRCZU1gwaKNuAwdRD%2FfqM8ns7Qo0OFPI6JiOPoYdvIFkzILT66w2JDnu%2Bypr1IS%2BRUCbHGdv2SFg35ZMMC9pn457GbCbKD6trJouIXHftjvEt09%2FNcKBxg%2Bb0AMK5NbItT0O195Tfxxqzu%2FmB8HgPVKYSixkiaZ8BdYCpxbPqZTLLmF3jGDCm9rHTBjqZAR%2F63%2F1dZe7lG6t0hIJyzatCaF0lyZiycfezv%2FQZMYZrNeWvK1i0430MWgqipv0FB6tEyG7JR02C7%2FIp4fGRBPznW0wSZXjTgeS%2B8y7yoQ6wMsyIZz7PEI9dTzNmmI86auzxuObfYXDRD3OgLGT5WRxg%2BRqMV0sqhzWnvHt%2F%2FvXgaTIR3V6EAlYOyXhTxMCJ4%2F0pPQxTvOo%2Fiw%3D%3D&Expires=1785497849)

3. [generate_cantilever_usda.py](https://ppl-ai-file-upload.s3.amazonaws.com/web/direct-files/attachments/33031087/7aa5c705-f87f-4a8b-b528-ee13f922ae2c/generate_cantilever_usda.py?AWSAccessKeyId=ASIA2F3EMEYEWI45WWRN&Signature=y0FWjbiixL2IzZK2F1I%2B%2BBpNrNs%3D&x-amz-security-token=IQoJb3JpZ2luX2VjEOP%2F%2F%2F%2F%2F%2F%2F%2F%2F%2FwEaCXVzLWVhc3QtMSJGMEQCIEtPZaz7WGJ9YCiA8lJdBPq%2Fh%2FuSufJ2NpaUWnTXtQQFAiBx2H8b%2BgkUNFAwl%2BcdAnDZE%2FaSwa3wrEO%2BPi130mlOsSr8BAis%2F%2F%2F%2F%2F%2F%2F%2F%2F%2F8BEAEaDDY5OTc1MzMwOTcwNSIMxYihoE5lwHwo07%2FnKtAE6ttNasT%2BwFDQZ7Yjz0eu9gqU2xIvIBfKTYsNnHBEmAGRPg2UW8sT7Y28AnGC2MSnW3lTBJO%2Farb%2F7RCW3%2B%2FVJtIORIIeqNNI3W4GCOcHiUZC1zXz5Yvi8htdr7x7NWR2HgcMAziAFxSF4fUgAht1LL3U%2FXV90Y%2FGxmq7Pl%2FUCuoqjRRZJlrU%2FKeSFzOw4fTnG5ltb0wgNaWhB2DfFULUFjwBEwhBi5kuy6XXPBwB7FOfjoutSRnPaA3stq7wH%2FlBleFh5HFeAaCtCVnDaQWm6uuS2e7DkzW7PQcZvjaqJdUZ5qZ2gefJz6UMKzkSIXbAyvyNp3hZnc7Tsl49ybdu4yijKj57EquUKo0a%2B8RmRAgXzm7fPRCtFO3Fw7tf2%2BRR0sIuGGyZReRLLkxHhoWvfsxPOnndChP2gBLhTawg2EfsKQqyRu7MNbKhrHP2QspNF9MZJFQ8Y0MwTMfrgi7uX9HoL%2FLsajDoY6sw%2BMxl8Cxqv%2FMN3Fh09m3TmbnxPjniU8xteoF3kYCaVnbDNw7feAnA%2BG0qGf%2BBLSlyuE9UxKqRiwTUDLjGdSOsTq6ux%2B1T3B7hi1REOMyRKQf2xMScVVRRtxmpPhOxVO9%2BRCZU1gwaKNuAwdRD%2FfqM8ns7Qo0OFPI6JiOPoYdvIFkzILT66w2JDnu%2Bypr1IS%2BRUCbHGdv2SFg35ZMMC9pn457GbCbKD6trJouIXHftjvEt09%2FNcKBxg%2Bb0AMK5NbItT0O195Tfxxqzu%2FmB8HgPVKYSixkiaZ8BdYCpxbPqZTLLmF3jGDCm9rHTBjqZAR%2F63%2F1dZe7lG6t0hIJyzatCaF0lyZiycfezv%2FQZMYZrNeWvK1i0430MWgqipv0FB6tEyG7JR02C7%2FIp4fGRBPznW0wSZXjTgeS%2B8y7yoQ6wMsyIZz7PEI9dTzNmmI86auzxuObfYXDRD3OgLGT5WRxg%2BRqMV0sqhzWnvHt%2F%2FvXgaTIR3V6EAlYOyXhTxMCJ4%2F0pPQxTvOo%2Fiw%3D%3D&Expires=1785497849) - """
generate_cantilever_usda.py

Generates a single articulated trunk chain for a Cantilever Bending...

4. [run_cantilever_manual.py](https://ppl-ai-file-upload.s3.amazonaws.com/web/direct-files/attachments/33031087/e44ccac7-6670-4c81-8b7c-0de569326bf8/run_cantilever_manual.py?AWSAccessKeyId=ASIA2F3EMEYEWI45WWRN&Signature=a97onsJy7tdwMPnUw1id7geAFuo%3D&x-amz-security-token=IQoJb3JpZ2luX2VjEOP%2F%2F%2F%2F%2F%2F%2F%2F%2F%2FwEaCXVzLWVhc3QtMSJGMEQCIEtPZaz7WGJ9YCiA8lJdBPq%2Fh%2FuSufJ2NpaUWnTXtQQFAiBx2H8b%2BgkUNFAwl%2BcdAnDZE%2FaSwa3wrEO%2BPi130mlOsSr8BAis%2F%2F%2F%2F%2F%2F%2F%2F%2F%2F8BEAEaDDY5OTc1MzMwOTcwNSIMxYihoE5lwHwo07%2FnKtAE6ttNasT%2BwFDQZ7Yjz0eu9gqU2xIvIBfKTYsNnHBEmAGRPg2UW8sT7Y28AnGC2MSnW3lTBJO%2Farb%2F7RCW3%2B%2FVJtIORIIeqNNI3W4GCOcHiUZC1zXz5Yvi8htdr7x7NWR2HgcMAziAFxSF4fUgAht1LL3U%2FXV90Y%2FGxmq7Pl%2FUCuoqjRRZJlrU%2FKeSFzOw4fTnG5ltb0wgNaWhB2DfFULUFjwBEwhBi5kuy6XXPBwB7FOfjoutSRnPaA3stq7wH%2FlBleFh5HFeAaCtCVnDaQWm6uuS2e7DkzW7PQcZvjaqJdUZ5qZ2gefJz6UMKzkSIXbAyvyNp3hZnc7Tsl49ybdu4yijKj57EquUKo0a%2B8RmRAgXzm7fPRCtFO3Fw7tf2%2BRR0sIuGGyZReRLLkxHhoWvfsxPOnndChP2gBLhTawg2EfsKQqyRu7MNbKhrHP2QspNF9MZJFQ8Y0MwTMfrgi7uX9HoL%2FLsajDoY6sw%2BMxl8Cxqv%2FMN3Fh09m3TmbnxPjniU8xteoF3kYCaVnbDNw7feAnA%2BG0qGf%2BBLSlyuE9UxKqRiwTUDLjGdSOsTq6ux%2B1T3B7hi1REOMyRKQf2xMScVVRRtxmpPhOxVO9%2BRCZU1gwaKNuAwdRD%2FfqM8ns7Qo0OFPI6JiOPoYdvIFkzILT66w2JDnu%2Bypr1IS%2BRUCbHGdv2SFg35ZMMC9pn457GbCbKD6trJouIXHftjvEt09%2FNcKBxg%2Bb0AMK5NbItT0O195Tfxxqzu%2FmB8HgPVKYSixkiaZ8BdYCpxbPqZTLLmF3jGDCm9rHTBjqZAR%2F63%2F1dZe7lG6t0hIJyzatCaF0lyZiycfezv%2FQZMYZrNeWvK1i0430MWgqipv0FB6tEyG7JR02C7%2FIp4fGRBPznW0wSZXjTgeS%2B8y7yoQ6wMsyIZz7PEI9dTzNmmI86auzxuObfYXDRD3OgLGT5WRxg%2BRqMV0sqhzWnvHt%2F%2FvXgaTIR3V6EAlYOyXhTxMCJ4%2F0pPQxTvOo%2Fiw%3D%3D&Expires=1785497849) - import os
import sys

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless":...

5. [Modelling the mechanical properties of single suspension-cultured tomato cells.](https://academic.oup.com/aob/article-pdf/93/4/443/497606/mch062.pdf) - ...modulus of elasticity of approx. 7 MPa, similar to a value found by the pressure probe method. Ho...

6. [Modelling the mechanical properties of single suspension-cultured tomato cells.](https://pmc.ncbi.nlm.nih.gov/articles/PMC4242341/) - ...modulus of elasticity of approx. 7 MPa, similar to a value found by the pressure probe method. Ho...

7. [Determination of the Elastic Properties of Tomato Fruit Cells with an Atomic Force Microscope](http://www.mdpi.com/1424-8220/13/9/12175/pdf) - ...intercellular adhesive properties determine the macro-mechanical properties of plants, a method f...

8. [Determination of the Elastic Properties of Tomato Fruit Cells with an Atomic Force Microscope](https://pmc.ncbi.nlm.nih.gov/articles/PMC3821342/) - ...intercellular adhesive properties determine the macro-mechanical properties of plants, a method f...

9. [Effect of tomato internal structure on its mechanical properties and degree of mechanical damage.](https://academicjournals.org/journal/AJB/article-full-text-pdf/8CF693819677.pdf) - As different tomatoes have different locular cavities and a particular tomato material is inhomogene...

10. [Solution for Determining Modulus of Elasticity of Natural Materials ...](https://pmc.ncbi.nlm.nih.gov/articles/PMC10224203/) - The article presents an original method for determining the modulus of elasticity of natural materia...

11. [Biomechanical study of the effect of a controlled bending on tomato ...](https://academic.oup.com/jxb/article/51/352/1825/538825?guestAccessKey=) - Abstract. In a previous paper it has been demonstrated that tomato stems, submitted to a controlled ...

