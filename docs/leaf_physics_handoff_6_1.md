# Ripresa del lavoro su Isaac Sim 6.1

Checkpoint del 16 settembre 2026. Il goal della foglia interattiva è **sospeso
su richiesta dell'utente**. Il trasferimento della cartella non riavvia il goal
né il budget temporale e non autorizza automaticamente nuovi test.

## Installazioni e trasferimento

- Progetto attuale: `~/isaacsim/autotom_digital_twin`.
- Nuova installazione verificata su disco: `~/isaacsim-6.1`.
- Versione dichiarata dal file VERSION: `6.1.0-rc.26+release.49347.2d230af4.gl`.
- Destinazione proposta del progetto: `~/isaacsim-6.1/autotom_digital_twin`.

Copiare la working directory completa con rsync, inclusi `.git`, file non
committati, repository in `external/` e `artifacts/`. Un normale git clone non
trasferisce le modifiche locali né i risultati ignorati. Escludere `.venv`, cache
Python e directory di sessione gestite `.codex`/`.agents`. Ricreare gli ambienti
Python nella nuova posizione quando necessario; i launcher dei virtualenv possono
contenere percorsi assoluti.

I launcher attuali non selezionano automaticamente Isaac 6.1 cambiando cartella:

```bash
export ISAACSIM_DIR="$HOME/isaacsim-6.1"
export ISAAC_PYTHON="$ISAACSIM_DIR/python.sh"
```

Queste variabili scelgono l'installazione, **non migrano le API**. Prima di eseguire
i vecchi banchi occorre adattarli. Non modificare la fisica della pianta per questa
prova isolata.

## Stato degli esperimenti 4.5

- `src/experiments/deformable_leaf/`: banco D0 volumetrico GPU, confronto
  60/120/240 Hz e diagnostica. Non ha superato l'accettazione; l'utente ha
  giudicato la GUI molto instabile. D1–D2.4 deformabili non sono stati promossi.
- È stato corretto il conteggio dei passi: un `world.step(render=False)` per
  passo fisico, con `world.render()` separato. Non tornare a usare il rendering
  come avanzamento implicito del clock fisico.
- `src/experiments/interactive_leaf/` e `run_interactive_leaf.sh`: alternativa
  sperimentale con cinque link D6 elastici, picciolo dinamico, superficie UsdSkel,
  sfera cinematica di pressione e pallina dinamica. PGS/CPU, riferimento 120 Hz.
- Alcune configurazioni hanno superato prove numeriche centrali, ripetute e fuori
  centro (`cycle-v5`, `repeated-120`, `offcenter-120` negli artifacts).
- La versione successiva disabilita lo sleep dell'articolazione. L'ultimo test
  `gui-awake-120` **non passa il recupero** (report: circa 26,38 mm di errore
  massimo nella finestra di recupero). Non attribuire automaticamente il difetto
  a una causa non verificata e non estendere alla versione attuale i precedenti
  risultati positivi. Nessuna accettazione finale dell'utente.
- Il lavoro è stato fermato esplicitamente. Il README del prototipo descrive
  anche verifiche previste: fare fede a report, log e hash dei singoli run.

Gli artifacts restano locali, con configurazioni, scene, pose, log e immagini.
I file del lavoro corrente sono salvati ma non committati. Nessun push effettuato.

## Nuove API già individuate, non ancora eseguite

Nell'installazione 6.1 sono presenti:

- `exts/isaacsim.core.experimental.prims/.../impl/deformable_prim.py`;
- `exts/isaacsim.core.experimental.materials/.../physics_materials/surface_deformable.py`;
- demo PhysX `SurfaceDeformableDemo.py`, `VolumeDeformableDemo.py` e
  `DeformableAttachmentsDemo.py`, sotto `extscache/omni.physx.demos-110.3.2+110.3.0.cp312.u7f4`.

La demo superficiale usa `create_auto_surface_deformable_hierarchy`,
`PhysxSurfaceDeformableBodyAPI` e materiale con rigidezze separate di stiramento,
taglio e flessione. È un candidato da studiare per foglie poco estensibili ma
flessibili; la sua presenza non prova stabilità alla scala di una foglia né
compatibilità dell'attachment con un link articolato dinamico.

Alla ripresa concordata: leggere istruzioni e API installate, verificare runtime
6.1 e costruire un nuovo caso minimo isolato. Mantenere distinta la variante
superficiale da quella volumetrica con tetraedri del piano originale. Conservare
il banco 4.5 come evidenza senza sovrascriverne i risultati.
