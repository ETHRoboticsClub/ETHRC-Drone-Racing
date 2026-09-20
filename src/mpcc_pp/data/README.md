# Model Validation & Residual Learning — `mpcc_pp/data`

Pipeline completa per raccogliere dati di validazione del modello durante la simulazione, analizzare i residui e allenare un MLP correttivo.

Nota importante: il file `train_residual_model.py` allena un MLP che corregge
`x_pred[1]` dopo il solve OCP. Questo e` utile per analisi e prototipazione,
ma non e` la stessa cosa del termine residuale di MPCC++, che entra direttamente
nella dinamica del modello usato dal solver.

---

## Struttura

```
data/
├── README.md                    # questo file
├── compute_residuals.py         # calcola residui e correlazioni da un CSV di logging
├── train_residual_model.py      # allena il modello MLP sui residui
├── model_validation_*.csv       # generati automaticamente durante la sim (se logging attivo)
├── *_residuals.csv              # generati da compute_residuals.py
├── *_correlations.csv           # generati da compute_residuals.py
└── residual_model.pt            # checkpoint PyTorch (generato da train_residual_model.py)
```

---

## 1. Raccolta dati durante la simulazione

Durante il racing, il controller registra per ogni step:

| Colonne | Descrizione |
|---|---|
| `sim_time_s` | Timestamp (monotonic) |
| `x_*` (19) | Stato augmented al passo t: `[p(3), q(4), v(3), ω(3), f(4), θ, vθ]` |
| `u_*` (5) | Controllo applicato: `[Δvθ, Δf0..Δf3]` |
| `xpred_*` (19) | Stato predetto dal modello al passo t+1 (da OCP) |
| `xreal_*` (13) | Stato reale osservato dal simulatore al passo t+1: `[p(3), v(3), q(4), ω(3)]` |

I file vengono salvati in questa cartella con nome `model_validation_YYYYMMDD_HHMMSS.csv`.

### Attivazione — flag nel codice

In `src/drone_control/launch/sim.launch.py`:

```python
MODEL_LOGGING = True   # ← cambia da False a True
```

### Attivazione — da terminale (senza toccare il codice)

```bash
ros2 launch drone_control sim.launch.py model_logging:=true
```

Il logging si attiva **solo per controller `mpcc` e `mpcc_pp`** e **solo durante il racing loop** (non durante l'hover iniziale MPC).

---

## 2. Analisi dei residui

Imposta `CSV_PATH` in `compute_residuals.py` con il path del file generato, poi esegui:

```bash
python3 compute_residuals.py
```

Produce due file:

### `*_residuals.csv`
Per ogni step: `res_p_n, res_p_e, res_p_d, res_q_*, res_v_*, res_w_*` (= xpred − xreal) più le norme `res_pos_norm`, `res_vel_norm`, `res_omega_norm`.

### `*_correlations.csv`
Matrice di Pearson r (righe = residui, colonne = stato + controllo). Identifica quali componenti dello stato o del controllo sono più correlate agli errori di predizione.

A schermo vengono anche stampate RMSE/mean/std per canale e tutte le coppie con |r| > 0.30.

---

## 3. Training del modello residuale

### Architettura
MLP feed-forward: input 24-dim `(x_aug[19] + u[5])` → hidden `(256, 256, 128)` con ReLU e Dropout → output 13-dim residuo su `[p, q, v, ω]`.

### Come allenare

1. Imposta `CSV_PATHS` in `train_residual_model.py` (lista di uno o più CSV di logging):

```python
CSV_PATHS = [
    Path(__file__).parent / 'model_validation_20260520_225200.csv',
    # aggiungine altri per dataset più ricchi
]
```

2. Lancia il training:

```bash
python3 train_residual_model.py
```

Il training usa Adam, learning rate scheduling (`ReduceLROnPlateau`) ed early stopping (patience 20 epoche). Al termine stampa RMSE per canale sul validation set e salva `residual_model.pt`.

### Variante piu` vicina al paper: fit polinomiale lineare

Se vuoi seguire piu` da vicino la struttura del paper MPCC++, usa:

```bash
python3 fit_paper_residual_model.py
```

Questo script:

- usa feature polinomiali leggere in stile paper;
- stima un modello lineare con least-squares;
- produce `paper_residual_model.json`;
- salva coefficienti interpretabili per residui su accelerazione lineare body-frame e accelerazione angolare body-frame.

Il file generato puo` essere caricato dentro `mpcc_pp/mpcc_pp/drone_model.py`
tramite `parameters.yaml`, quindi il residuo entra nella dinamica usata da
ACADOS invece di correggere solo la predizione a posteriori.

---

## 4. Attivazione del modello correttivo nel controller

Una volta allenato, il modello viene caricato a runtime e applica una correzione additiva a `x_pred[1][0:13]` dopo ogni solve OCP (non modifica il planner, solo la predizione reported).

### Da flag nel codice

In `src/drone_control/launch/sim.launch.py`:

```python
RESIDUAL_MODEL_PATH = '/home/mattia/ETHRC-Drone-Racing/src/mpcc_pp/data/residual_model.pt'
```

### Da terminale

```bash
ros2 launch drone_control sim.launch.py \
    residual_model_path:=/home/mattia/ETHRC-Drone-Racing/src/mpcc_pp/data/residual_model.pt
```

Per disabilitarlo: lasciare `RESIDUAL_MODEL_PATH = ''` o non passare il parametro.

### Attivazione del residuo "paper-style" dentro il modello OCP

In `src/mpcc_pp/mpcc_pp/parameters.yaml`:

```yaml
model:
  residual:
    enabled: true
    coefficients_path: /home/mattia/ETHRC-Drone-Racing/src/mpcc_pp/data/paper_residual_model.json
```

Quando abiliti questa opzione, il solver ACADOS viene rigenerato per includere
il termine residuale dentro `v_dot` e `w_dot`. Questo e` l'approccio piu`
coerente con MPCC++.

---

## Flusso completo

```
1. sim.launch.py  (MODEL_LOGGING=True)
        ↓
   model_validation_*.csv

2. compute_residuals.py
        ↓
   *_residuals.csv  +  *_correlations.csv

3. train_residual_model.py
        ↓
   residual_model.pt

3b. fit_paper_residual_model.py
        ↓
   paper_residual_model.json

4. sim.launch.py  (RESIDUAL_MODEL_PATH=...)
        ↓
   Correzione residuale attiva a runtime

4b. parameters.yaml  (model.residual.enabled=true)
        ↓
   Residuo polinomiale attivo dentro la dinamica OCP
```
