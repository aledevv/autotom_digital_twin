Ecco i test concreti da eseguire dentro Isaac Sim, con la formula analitica di riferimento e il confronto numerico da fare per ciascuno. Sono pensati per essere eseguiti con le stesse API che già usi (`RigidPrim`, `Articulation`, `apply_forces`, `get_measured_joint_forces`), in modalità headless o con GUI, uno alla volta.

## 1. Verifica massa totale della catena

Prima di tutto, controlla che la massa che Isaac Sim vede sia quella che ti aspetti dal calcolo analitico.

```python
mass_teorica = TrunkConfig.N_LINKS * compute_mass(TrunkConfig.RADIUS, TrunkConfig.HEIGHT)
# confronta con quella letta dalla simulazione
masse_sim = [RigidPrim(f"/World/Stem/Trunk_{i:02d}").get_masses() for i in range(1, N_LINKS+1)]
mass_sim_totale = sum(masse_sim)
assert abs(mass_sim_totale - mass_teorica) / mass_teorica < 0.01  # tolleranza 1%
```

Se questo fallisce, il problema è a monte nella generazione USD (densità, volume o `MassAPI` applicata male), e qualunque test successivo sarà inutile.

## 2. Verifica momento di inerzia della sezione (I)

Calcola I a mano con la formula I = π·r⁴/4 usando il raggio effettivo del tuo cilindro, e stampa il valore usato internamente da `calculate_physics_params`. Devono coincidere esattamente (sono la stessa riga di codice, ma è un controllo a costo zero per escludere refusi tipo raggio vs diametro scambiati).

```python
I_manuale = math.pi * (TrunkConfig.RADIUS ** 4) / 4.0
print(I_manuale)  # confronta a occhio col valore stampato dentro calculate_physics_params
```

Un errore classico qui: usare il diametro al posto del raggio fa sbagliare I di un fattore 16 (perché è elevato alla quarta), che si traduce esattamente nel tipo di errore "serve un E mille volte più alto" che avevi osservato all'inizio.

## 3. Test statico del singolo joint isolato (il più importante)

Crea temporaneamente (o isola nel debug) una catena di **soli 2 link**, imposta uno stiffness rotazionale K noto a mano (es. K = 1.0 N·m/rad, damping = 0), e applica una coppia esterna nota τ al link libero, aspettando l'equilibrio.

Valore atteso analiticamente:

θ_atteso = τ / K

```python
K_test = 1.0
tau_test = 0.1  # N*m
theta_atteso = tau_test / K_test  # in radianti

# in simulazione: applica tau_test come forza a distanza nota dal joint, aspetta stabilizzazione
# poi leggi l'angolo effettivo del joint (get_joint_positions() sull'Articulation)
theta_misurato = stem_articulation.get_joint_positions()[0][indice_joint]
errore_percentuale = abs(theta_misurato - theta_atteso) / theta_atteso * 100
```

Se questo test elementare con numeri "puliti" non torna entro pochi punti percentuale, il problema è nel drive (asse sbagliato, force vs acceleration drive, unità), e non ha senso procedere oltre finché non è risolto.

## 4. Test del periodo di oscillazione libera (verifica di K e damping insieme)

Con lo stesso joint singolo isolato, gravità a zero, applica un impulso e lascia oscillare liberamente senza damping (D = 0 temporaneamente). Il periodo di oscillazione torsionale atteso è:

T_atteso = 2π · √(J / K)

dove J è il momento d'inerzia di massa del link libero rispetto all'asse del joint (non il momento d'inerzia di sezione I — sono due grandezze diverse, occhio a non confonderle).

```python
J = mass_link * (TrunkConfig.HEIGHT ** 2) / 3.0  # approssimazione asta sottile rispetto a un estremo
T_atteso = 2 * math.pi * math.sqrt(J / K_test)

# in simulazione: misura il tempo tra due picchi consecutivi di deflessione durante l'oscillazione libera
T_misurato = t_picco2 - t_picco1
```

Questo test verifica contemporaneamente che K sia applicato correttamente E che la massa/inerzia del link sia quella attesa — se il test 1 e il test 3 sono già passati ma questo fallisce, il problema è specificamente nell'inerzia rotazionale, non nella massa totale.

## 5. Test di deflessione statica sotto peso proprio (nessuna forza esterna)

Con E impostato a un valore noto (es. 20 MPa) su tutta la catena a 10 link nella configurazione a due appoggi, lascia stabilizzare sotto sola gravità. La formula analitica per una trave appoggiata-appoggiata sotto carico uniformemente distribuito (il peso proprio) è:

δ_atteso = 5·w·L⁴ / (384·E·I)

dove w è il peso per unità di lunghezza (w = massa_totale·g / L).

```python
w = (mass_teorica * 9.81) / L_campata
delta_atteso = 5 * w * (L_campata**4) / (384 * E_test * I_manuale)
# confronta con la deflessione del link centrale misurata a riposo sotto sola gravità
```

Tolleranza ragionevole qui: 15-20%, perché la catena discreta a 10 link approssima ma non replica esattamente una trave continua con carico distribuito continuo.

## 6. Test di deflessione statica con carico puntuale centrale (il test vero, ma con numeri noti)

Stesso setup, ma azzera la gravità (per isolare l'effetto del solo carico applicato, evitando di sommare gli effetti del test 5) e applica una forza nota F al link centrale.

δ_atteso = F·L³ / (48·E·I)

```python
delta_atteso = (F_test * L_campata**3) / (48 * E_test * I_manuale)
errore_percentuale = abs(delta_misurata - delta_atteso) / delta_atteso * 100
```

Questo è il test che replica esattamente la formula del paper di Anisimov et al. (2025) e di Shtein et al. (2020) — se questo torna con un errore contenuto (sotto il 10-15%, dato il numero finito di link), la tua catena discreta è una buona approssimazione della trave continua e puoi fidarti della calibrazione successiva con dati reali.

## 7. Test di sovrapposizione degli effetti (linearità)

Con gravità attiva E forza esterna applicata insieme, la deflessione totale al centro deve essere circa uguale alla somma delle deflessioni misurate separatamente nei test 5 e 6 (vale solo in regime di piccole deformazioni, elastico lineare).

```python
delta_combinato_atteso = delta_gravita_test5 + delta_forza_test6
errore = abs(delta_combinato_misurato - delta_combinato_atteso) / delta_combinato_atteso * 100
```

Se questo non torna ma i test 5 e 6 separati sì, significa che sei uscito dal regime di piccole deformazioni (spostamenti troppo grandi rispetto alla lunghezza), e la formula lineare EB = FL³/48I non è più valida per il carico che stai usando.

## 8. Test di conservazione dell'energia (oscillazione libera, senza damping)

Nel test 4, con damping azzerato, calcola l'energia meccanica totale (cinetica + elastica) del link oscillante a più istanti nel tempo:

```python
E_meccanica = 0.5 * J * omega**2 + 0.5 * K_test * theta**2
```

Deve restare costante entro una piccola percentuale (2-5%) su diversi cicli di oscillazione. Se decresce visibilmente senza che tu abbia impostato damping, hai dissipazione numerica spuria dal solver (spesso legata a iterazioni insufficienti o timestep troppo grande), che andrebbe corretta prima di fidarti di qualunque misura di rigidità.

***