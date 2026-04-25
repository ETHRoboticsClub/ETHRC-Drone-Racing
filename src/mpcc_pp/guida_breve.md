# Guida Breve: Tuning MPCC++ con TuRBO Bayesian Optimization

Questa è una versione compatta in italiano della guida lunga `guide.md`.
L'obiettivo è spiegare, in modo pratico, come aggiungere il tuning automatico
dei parametri di `mpcc_pp` usando Bayesian Optimization con TuRBO, come nel
paper MPCC++.

---

## 1. Obiettivo

Vogliamo ottimizzare automaticamente i parametri del controller MPCC++ invece di
tunarli a mano.

L'idea è trattare MPCC++ come una funzione black-box:

```text
parametri controller -> episodio in simulazione -> reward
```

TuRBO propone un set di parametri, il simulatore esegue alcuni giri di pista, si
calcola una reward, e TuRBO usa quel risultato per proporre parametri migliori.

Nel paper, un episodio corrisponde a:

```text
3 giri consecutivi
```

La reward è:

```text
R(phi) = - mean(lap_times) - 100 * solver_failure_rate
```

Dove:

- `lap_times` sono i tempi dei giri completati.
- `solver_failure_rate` è la frazione di step in cui ACADOS fallisce o ritorna
  uno status non accettabile.
- Episodi con crash, timeout o violazioni gravi ricevono una reward molto bassa.

Quindi TuRBO cerca di massimizzare la reward, cioè:

- ridurre il tempo medio sul giro,
- evitare configurazioni instabili,
- mantenere il solver affidabile.

---

## 2. Stato Attuale del Codice

Il package `mpcc_pp` ha già quasi tutto il controller:

- `mpcc_pp/drone_model.py`: modello dinamico aumentato del drone.
- `mpcc_pp/drone_opt.py`: problema ACADOS con costo MPCC++, progress reward e
  soft tunnel constraint.
- `mpcc_pp/drone_mpcc_pp_controller.py`: wrapper del controller usato dal nodo
  ROS.
- `mpcc_pp/path_utils.py`: spline del tracciato, pesi `qc`, tunnel e geometria
  dei gate.
- `drone_control/config/gates_config.yaml`: configurazione delle piste e dei
  parametri manuali.

La parte mancante è una pipeline di tuning:

```text
TuRBO -> parametri -> episodio -> metriche -> reward -> nuovo punto TuRBO
```

---

## 3. Primo Problema da Risolvere: Evitare Rebuild ACADOS

Nel codice attuale, molti pesi del costo sono fissati quando viene creato il
solver ACADOS:

```python
cost_lag      = w["ql"] * el**2
cost_omega    = w["Qw"] * dot(w, w)
cost_dvt      = w["r_dvt"] * dvt**2
cost_df       = w["R_df"] * dot(df, df)
cost_progress = -w["mu"] * vt
cost_track    = w["track_weight"] * barrier
```

Questo significa che se TuRBO cambia un peso, ACADOS rischia di rigenerare e
ricompilare il solver. Con centinaia di episodi sarebbe troppo lento.

La prima modifica importante è quindi:

```text
spostare i parametri tunabili da costanti compile-time a parametri runtime
ACADOS.
```

In pratica:

- estendere il vettore `p` di ACADOS,
- appendere i pesi del tuning agli stage parameters,
- rimuovere i pesi dall'hash di build del solver.

Così il solver viene compilato una volta sola e TuRBO può cambiare parametri
senza rebuild.

---

## 4. Parametri da Ottimizzare

Per una prima versione semplice e compatibile con il codice attuale, ottimizzare
questi 8 parametri:

```text
ql
qc_nom
qc_gate
Qw
r_dvt
R_df
mu
track_weight
```

Significato:

- `ql`: peso sull'errore lag, cioè l'errore lungo la direzione della traiettoria.
- `qc_nom`: peso di contouring lontano dai gate.
- `qc_gate`: peso di contouring vicino ai gate.
- `Qw`: penalità sui body rates.
- `r_dvt`: penalità sull'accelerazione della velocità di progresso.
- `R_df`: penalità sul rateo di variazione delle spinte dei rotori.
- `mu`: reward di progresso, quindi quanto il controller vuole andare veloce.
- `track_weight`: peso della soft barrier del tunnel.

Una scelta ancora più fedele al paper sarebbe separare:

```text
Qc orizzontale / Qc verticale
Qomega roll-pitch / Qomega yaw
```

ma conviene farlo in una seconda fase. Prima meglio far funzionare il tuning con
i parametri già presenti.

---

## 5. Search Space

TuRBO lavora meglio se tutti i parametri sono normalizzati in:

```text
[0, 1]
```

Pero i pesi del controller variano su ordini di grandezza, quindi bisogna usare
una trasformazione logaritmica:

```python
physical = 10 ** (log10(low) + z * (log10(high) - log10(low)))
```

Range iniziali consigliati:

```text
ql:            1e-2 .. 1e2
qc_nom:        1e-3 .. 1e1
qc_gate_ratio: 1    .. 50
Qw:            1e-7 .. 1e-2
r_dvt:         1e-7 .. 1e-1
R_df:          1e-7 .. 1e-1
mu:            1e-2 .. 1e2
track_weight:  1    .. 1e3
```

Invece di ottimizzare direttamente `qc_gate`, conviene ottimizzare:

```text
qc_gate_ratio
```

e poi calcolare:

```python
qc_gate = qc_nom * qc_gate_ratio
```

Questo garantisce che:

```text
qc_gate >= qc_nom
```

cioè vicino ai gate il tracking resta almeno tanto forte quanto nel resto della
pista.

---

## 6. Modifiche a `drone_opt.py`

Attualmente gli stage parameters hanno 15 campi:

```text
p_ref(3), t_ref(3), qc(1), n_ref(3), b_ref(3), W(1), H(1)
```

Bisogna estenderli aggiungendo i pesi runtime:

```text
ql
Qw
r_dvt
R_df
mu
track_weight
```

Il nuovo vettore avrà quindi:

```text
15 + 6 = 21 parametri
```

In `drone_opt.py`:

1. cambiare:

```python
NP = 15
```

in:

```python
NP = 21
```

2. aggiungere simboli CasADi per i nuovi parametri:

```python
ql_p = ca.MX.sym("ql", 1)
Qw_p = ca.MX.sym("Qw", 1)
r_dvt_p = ca.MX.sym("r_dvt", 1)
R_df_p = ca.MX.sym("R_df", 1)
mu_p = ca.MX.sym("mu", 1)
track_weight_p = ca.MX.sym("track_weight", 1)
```

3. appendere questi simboli a `params`.

4. usare questi simboli nel costo:

```python
cost_lag      = ql_p * el**2
cost_omega    = Qw_p * ca.dot(w_k, w_k)
cost_dvt      = r_dvt_p * dvt_k**2
cost_df       = R_df_p * ca.dot(df_k, df_k)
cost_progress = -mu_p * vt_k
cost_track    = track_weight_p * barrier_sum
```

5. rimuovere i pesi dal `config_hash`, altrimenti ACADOS ricompila a ogni trial.

---

## 7. Modifiche a `drone_mpcc_pp_controller.py`

Il controller deve poter ricevere nuovi parametri durante il tuning.

Aggiungere un dizionario interno:

```python
self._runtime_weights = {
    "ql": tp.get("ql", 7.0),
    "Qw": tp.get("Qw", 0.0001),
    "r_dvt": tp.get("r_dvt", 0.0002),
    "R_df": tp.get("R_df", 0.0003),
    "mu": tp.get("mu", 7.0),
    "track_weight": tp.get("track_weight", 30.0),
}
```

Aggiungere un metodo:

```python
def set_tuning_parameters(self, params: dict) -> None:
    self._runtime_weights.update({
        "ql": float(params["ql"]),
        "Qw": float(params["Qw"]),
        "r_dvt": float(params["r_dvt"]),
        "R_df": float(params["R_df"]),
        "mu": float(params["mu"]),
        "track_weight": float(params["track_weight"]),
    })
    self._qc_nom = float(params["qc_nom"])
    self._qc_gate = float(params["qc_gate"])
```

Quando `_run_step()` costruisce gli stage parameters, appendere i pesi:

```python
stage_params = self._path.get_tunnel_params(...)
stage_params = self._append_weight_params(stage_params)
```

con:

```python
def _append_weight_params(self, stage_params):
    extra = np.array([
        self._runtime_weights["ql"],
        self._runtime_weights["Qw"],
        self._runtime_weights["r_dvt"],
        self._runtime_weights["R_df"],
        self._runtime_weights["mu"],
        self._runtime_weights["track_weight"],
    ])
    repeated = np.repeat(extra.reshape(1, -1), stage_params.shape[0], axis=0)
    return np.hstack([stage_params, repeated])
```

---

## 8. Metriche Necessarie

Ogni episodio deve produrre almeno:

```text
success
lap_times
solver_fail_rate
min_tunnel_margin
timeout
failure_reason
reward
```

### Solver failure rate

In `drone_opt.py`, `solve()` deve restituire lo status ACADOS:

```python
status = self.solver.solve()
info = {
    "status": int(status),
    "success": status in (0, 2),
}
```

Nel controller si contano:

```text
solve_count
solver_fail_count
solver_fail_rate
```

### Lap times

Quando `lap_count` aumenta, salvare un timestamp.

Poi calcolare:

```python
lap_time = timestamp_lap_i - timestamp_lap_i_minus_1
```

Attenzione al lead-in iniziale: la parte dal punto di partenza al primo gate non
dovrebbe essere contata come giro.

### Tunnel margin

Per ogni stato del drone:

```python
e = p - p_ref
e_n = dot(n_ref, e)
e_b = dot(b_ref, e)

h1 = W + e_n
h2 = W - e_n
h3 = H + e_b
h4 = H - e_b

margin = min(h1, h2, h3, h4)
```

Interpretazione:

```text
margin >= 0  -> drone dentro il tunnel
margin < 0   -> violazione del tunnel
```

---

## 9. Reward

Implementare una funzione piccola e testabile:

```python
def compute_reward(result, gamma=100.0, failed_episode_reward=-1000.0):
    if not result.success:
        return failed_episode_reward
    if not result.lap_times:
        return failed_episode_reward
    return -np.mean(result.lap_times) - gamma * result.solver_fail_rate
```

Questa reward segue il paper.

Successivamente, se serve, si può aggiungere una penalità esplicita per
violazioni del tunnel:

```python
reward -= safety_weight * max(0.0, -min_tunnel_margin)
```

Ma nella prima versione conviene restare fedeli al paper e usare solo lap time +
solver failures.

---

## 10. Struttura dei File Consigliata

Creare:

```text
src/mpcc_pp/mpcc_pp/tuning/
  __init__.py
  search_space.py
  reward.py
  metrics.py
  episode_runner.py
  turbo.py
  storage.py
  cli.py
```

Ruoli:

- `search_space.py`: mapping da `[0, 1]^d` a parametri fisici.
- `reward.py`: calcolo reward.
- `metrics.py`: lap time, solver stats, tunnel margin.
- `episode_runner.py`: esegue un episodio in simulazione.
- `turbo.py`: implementa TuRBO / Bayesian Optimization.
- `storage.py`: salva trial, checkpoint e best params.
- `cli.py`: entry point da terminale.

In `setup.py` aggiungere:

```python
entry_points={
    "console_scripts": [
        "mpcc_pp_turbo_tune = mpcc_pp.tuning.cli:main",
    ],
}
```

---

## 11. Logging

Ogni run deve salvare tutto in una cartella:

```text
runs/mpcc_pp_tuning/YYYYMMDD_HHMMSS/
  config.yaml
  trials.csv
  trials.jsonl
  best_params.yaml
  checkpoints/
  episodes/
```

Ogni trial deve contenere:

```text
trial index
seed
parametri normalizzati
parametri fisici
reward
success
lap_times
solver_fail_rate
min_tunnel_margin
failure_reason
```

Il file più importante è:

```text
best_params.yaml
```

che deve poter essere copiato dentro `gates_config.yaml`.

---

## 12. Implementazione TuRBO

Partire con TuRBO-1, non TuRBO-8.

Pipeline:

```python
space = SearchSpace()
runner = EpisodeRunner(...)
optimizer = TurboOptimizer(dim=space.dim)

for trial in range(num_episodes):
    z = optimizer.ask()
    params = space.decode(z)
    result = runner.evaluate(params)
    reward = compute_reward(result)
    optimizer.tell(z, reward)
    storage.append_trial(...)
```

Prima di usare il Gaussian Process, raccogliere campioni iniziali Sobol:

```text
n_init = 2 * dim
```

Per 8 parametri:

```text
n_init = 16
```

Poi:

- fit del Gaussian Process,
- acquisizione UCB,
- scelta del prossimo punto nella trust region,
- update della trust region in base a successi/fallimenti.

UCB:

```text
alpha(phi) = mu(phi) + beta * sigma(phi)
```

Valore iniziale consigliato:

```text
beta = 2.0
```

---

## 13. TuRBO-8 Solo Dopo

Il paper usa TuRBO-8, cioè 8 istanze locali in parallelo.

Nel vostro caso conviene procedere così:

```text
1. Sobol search sequenziale
2. TuRBO-1 sequenziale
3. TuRBO-1 stabile con logging e resume
4. TuRBO-8 sequenziale
5. TuRBO-8 parallelo
```

Per parallelizzare davvero servono istanze isolate:

```text
ROS_DOMAIN_ID diverso
porte MAVLink diverse
istanze PX4/Gazebo diverse
cartelle run diverse
seed diversi
```

Se queste cose non sono isolate, i worker si contaminano a vicenda.

---

## 14. Protocollo di Validazione

Dopo il training:

1. prendere i migliori parametri trovati,
2. eseguire 10 episodi di test,
3. ogni episodio deve durare 3 giri,
4. riportare:

```text
mean lap time
std lap time
success rate
solver failure rate
minimum tunnel margin
crash count
timeout count
```

Come nel paper:

```text
10 episodi = 30 giri di test
```

Una policy è accettabile solo se:

```text
success rate = 100%
solver failure rate basso
nessuna violazione grave del tunnel
lap time migliore del baseline manuale
```

---

## 15. Ordine di Implementazione Consigliato

Seguire questo ordine:

```text
1. Aggiungere status solver e contatori di failure.
2. Aggiungere lap time tracking.
3. Aggiungere reset delle metriche episodio.
4. Rendere i pesi MPCC++ parametri runtime ACADOS.
5. Verificare che ACADOS non venga ricompilato a ogni candidato.
6. Implementare SearchSpace.
7. Implementare reward.
8. Implementare logging JSONL/CSV.
9. Implementare EpisodeRunner minimale.
10. Testare 10-20 candidati Sobol.
11. Implementare TuRBO-1.
12. Eseguire 50 candidati TuRBO-1.
13. Esportare best_params.yaml.
14. Validare con 10 episodi / 30 giri.
15. Solo dopo aggiungere TuRBO-8.
```

---

## 16. Comandi Target

Smoke test:

```bash
ros2 run mpcc_pp mpcc_pp_turbo_tune \
  --track src/drone_control/config/gates_config.yaml \
  --controller mpcc_pp \
  --episodes 10 \
  --laps-per-episode 1 \
  --out runs/mpcc_pp_tuning_smoke
```

Training completo stile paper:

```bash
ros2 run mpcc_pp mpcc_pp_turbo_tune \
  --track src/drone_control/config/gates_config.yaml \
  --controller mpcc_pp \
  --episodes 600 \
  --laps-per-episode 3 \
  --out runs/mpcc_pp_tuning
```

Valutazione finale:

```bash
ros2 run mpcc_pp mpcc_pp_turbo_tune \
  --track src/drone_control/config/gates_config.yaml \
  --eval-only runs/mpcc_pp_tuning/latest/best_params.yaml \
  --eval-episodes 10 \
  --laps-per-episode 3
```

Output desiderato:

```text
Best reward: -5.23
Mean lap time: 5.21 s
Lap time std: 0.06 s
Success rate: 100 %
Solver failure rate: 0 %
Minimum tunnel margin: 0.08 m
Best params written to: runs/mpcc_pp_tuning/.../best_params.yaml
```

---

## 17. In Breve

La cosa più importante è non partire subito da TuRBO-8.

Prima bisogna rendere il controller:

- misurabile,
- resettabile,
- loggabile,
- parametrizzabile runtime senza rebuild ACADOS.

Poi si può fare:

```text
Sobol -> TuRBO-1 -> validazione -> TuRBO-8
```

Questo è il percorso più sicuro per arrivare a un tuning automatico simile al
paper, ma integrato bene nel codice attuale.

