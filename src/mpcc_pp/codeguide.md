# MPCC++ Optuna Tuning Code Guide

Questa guida spiega le modifiche fatte per ottimizzare automaticamente i
parametri di `mpcc_pp` con Optuna, cosa e' stato cambiato nel codice, perche'
era necessario farlo, e come usare la nuova pipeline.

L'obiettivo non e' solo "provare valori diversi", ma costruire un ciclo
ripetibile:

```text
Optuna -> parametri MPCC++ -> episodio in simulazione -> metriche -> reward
```

Optuna massimizza la reward. Nel nostro caso una reward piu' alta significa:

- giri piu' veloci;
- meno fallimenti del solver ACADOS;
- episodi completi senza timeout.

---

## 1. File modificati e aggiunti

Modificati:

- `mpcc_pp/drone_opt.py`
- `mpcc_pp/drone_mpcc_pp_controller.py`
- `drone_control/drone_controller_node.py`
- `mpcc_pp/setup.py`

Aggiunti:

- `mpcc_pp/tuning/__init__.py`
- `mpcc_pp/tuning/search_space.py`
- `mpcc_pp/tuning/optuna_tuner.py`
- `mpcc_pp/tuning/gazebo_trial_runner.py`
- `codeguide.md`

---

## 2. Perche' serviva cambiare `drone_opt.py`

Prima della modifica, i pesi principali del costo MPCC++ erano inseriti
direttamente nell'espressione simbolica CasADi/ACADOS:

```python
cost_lag      = w['ql']    * el**2
cost_omega    = w['Qw']    * dot(w_k, w_k)
cost_dvt      = w['r_dvt'] * dvt_k**2
cost_df       = w['R_df']  * dot(df_k, df_k)
cost_progress = -w['mu']   * vt_k
cost_track    = w['track_weight'] * barrier_sum
```

Questo andava bene per tuning manuale, ma non per Bayesian optimization.
Optuna cambia questi valori a ogni trial. Se quei valori sono costanti
compile-time dentro il problema ACADOS, ogni trial puo' causare rigenerazione e
ricompilazione del solver. Con decine o centinaia di trial diventa
impraticabile.

La modifica importante e':

```text
i pesi ottimizzati non sono piu' costanti compile-time;
sono parametri runtime dello stage ACADOS.
```

Risultato:

- il primo avvio puo' ricompilare perche' il numero di parametri ACADOS e'
  cambiato;
- i trial successivi possono cambiare `ql`, `Qw`, `r_dvt`, `R_df`, `mu` e
  `track_weight` senza ricompilare il solver;
- l'hash di build ACADOS non include piu' i valori correnti di questi pesi.

Cambiare ancora `n_nodes`, `t_horizon`, vincoli, modello, solver options o
`track_barrier_alpha` puo' invece richiedere rebuild, perche' cambia davvero la
struttura del problema.

---

## 3. Nuovo layout degli stage parameters ACADOS

Prima gli stage parameters erano 15:

```text
[0:3]   p_ref
[3:6]   t_ref
[6]     qc
[7:10]  n_ref
[10:13] b_ref
[13]    W
[14]    H
```

Ora sono 21:

```text
[0:3]   p_ref
[3:6]   t_ref
[6]     qc
[7:10]  n_ref
[10:13] b_ref
[13]    W
[14]    H
[15]    ql
[16]    Qw
[17]    r_dvt
[18]    R_df
[19]    mu
[20]    track_weight
```

I primi 15 valori descrivono geometria della pista, frame locale del tunnel e
peso `qc` lungo l'orizzonte. Gli ultimi 6 sono i pesi runtime ottimizzati da
Optuna.

`path_utils.py` continua a produrre solo i primi 15 valori. Questa scelta e'
voluta: la path utility deve conoscere la geometria della pista, non i dettagli
del tuning. I pesi vengono aggiunti nel controller.

---

## 4. Cosa cambia in `DroneOptimizerMpccPp`

In `mpcc_pp/drone_opt.py`:

- `NP` passa da `15` a `21`;
- vengono aggiunti i simboli CasADi:

```python
ql_p
Qw_p
r_dvt_p
R_df_p
mu_p
track_weight_p
```

- il costo usa questi simboli runtime:

```python
cost_lag      = ql_p * el**2
cost_omega    = Qw_p * dot(w_k, w_k)
cost_dvt      = r_dvt_p * dvt_k**2
cost_df       = R_df_p * dot(df_k, df_k)
cost_progress = -mu_p * vt_k
cost_track    = track_weight_p * barrier_sum
```

- il terminal cost usa gli stessi parametri runtime;
- il build hash non contiene piu' `sorted(w.items())`;
- `solve()` ora restituisce:

```python
u_opt, x_pred, info
```

`info` contiene:

```text
status
success
finite_solution
time_tot
time_lin
time_sim
time_qp
u_pred
```

Per compatibilita', `solve()` accetta ancora una matrice legacy `(N+1, 15)`.
In quel caso appende i pesi default caricati dal file YAML. Il controller
aggiornato pero' invia gia' `(N+1, 21)`.

---

## 5. Cosa cambia in `DroneMpccPpController`

In `mpcc_pp/drone_mpcc_pp_controller.py` ora esiste un dizionario:

```python
self._runtime_weights
```

Contiene:

```text
ql
Qw
r_dvt
R_df
mu
track_weight
```

Questi valori vengono letti da `parameters.yaml` come prima, ma ora possono
anche essere cambiati a runtime tramite:

```python
controller.set_tuning_parameters(params)
```

Il metodo accetta anche:

```text
qc_nom
qc_gate
qc_gate_ratio
```

Se passi `qc_gate_ratio`, il controller calcola:

```python
qc_gate = qc_nom * qc_gate_ratio
```

Questo vincolo e' utile perche' evita trial in cui il tracking vicino ai gate e'
piu' debole del tracking nominale tra i gate.

Ogni step del controller ora fa:

```python
stage_params = path.get_tunnel_params(...)
stage_params = self._append_weight_params(stage_params)
u_opt, x_pred, info = self._opt.solve(x_aug, stage_params)
```

Quindi la geometria resta separata dai pesi, ma ACADOS riceve tutti i 21
parametri necessari.

---

## 6. Metriche solver aggiunte

Per ottimizzare serve sapere se una configurazione e' veloce ma instabile.
Per questo il controller ora conta:

```text
solve_count
solver_fail_count
solver_fail_rate
status_counts
last_status
```

Metodi aggiunti:

```python
controller.reset_episode_stats()
controller.get_solver_stats()
```

`solver_fail_rate` e':

```text
solver_fail_count / solve_count
```

Un solve viene considerato riuscito se ACADOS restituisce uno status accettato
e la soluzione predetta e' finita. Gli status accettati restano `0` e `2`, come
nella logica gia' usata nella guida.

---

## 7. Logging nel nodo ROS

In `drone_control/drone_controller_node.py`, quando viene completato un giro,
il nodo logga gia':

```text
[Lap] percorso 1 completato in 12.34 s
```

Ora subito dopo logga anche:

```text
[SolverStats] solve_count=... solver_fail_count=... solver_fail_rate=... last_status=...
```

Questo serve al runner Optuna: puo' lanciare la simulazione come processo
esterno, leggere stdout, estrarre lap time e solver failure rate, poi calcolare
la reward.

---

## 8. Search space Optuna

Il file `mpcc_pp/tuning/search_space.py` definisce la search space iniziale.

Parametri ottimizzati:

```text
ql
qc_nom
qc_gate_ratio
Qw
r_dvt
R_df
mu
track_weight
```

Range:

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

Tutti questi parametri usano scala logaritmica. Il motivo e' che pesi come
`Qw`, `R_df` e `track_weight` possono cambiare effetto su molti ordini di
grandezza. Campionare in scala lineare sprecherebbe molti trial in zone poco
informative.

`qc_gate` non viene suggerito direttamente. Viene derivato:

```python
qc_gate = qc_nom * qc_gate_ratio
```

---

## 9. Runner Optuna

Il file `mpcc_pp/tuning/optuna_tuner.py` aggiunge il comando:

```bash
mpcc_pp_optuna_tune
```

Il runner:

1. carica un `parameters.yaml` base;
2. chiede a Optuna un set di parametri;
3. scrive un file `parameters.yaml` specifico del trial;
4. lancia il comando di simulazione;
5. esporta `MPCC_PP_PARAMETERS_FILE` verso quel comando;
6. legge i log `[Lap]` e `[SolverStats]`;
7. calcola la reward;
8. salva metriche e parametri del trial;
9. aggiorna lo studio Optuna;
10. esporta `best_parameters.yaml`.

La reward default e':

```python
reward = -mean(lap_times) - gamma * solver_fail_rate
```

con:

```text
gamma = 100
```

Se un episodio fallisce, riceve:

```text
failed_reward = -1e6
```

Un episodio fallisce se:

- non completa il numero richiesto di giri;
- va in timeout;
- supera `max_solver_fail_rate`.

---

## 10. Come usarlo

Prima ricompila e risorsa l'ambiente ROS:

```bash
colcon build --packages-select mpcc_pp drone_control
source install/setup.bash
```

Per verificare solo che la generazione del file trial funzioni:

```bash
mpcc_pp_optuna_tune \
  --dry-run \
  --output-dir runs/mpcc_pp_optuna
```

Questo crea:

```text
runs/mpcc_pp_optuna/dry_run/parameters.yaml
runs/mpcc_pp_optuna/dry_run/trial_params.json
```

Per lanciare Optuna con il simulatore ROS:

```bash
mpcc_pp_optuna_tune \
  --n-trials 30 \
  --laps-per-episode 3 \
  --timeout-s 90 \
  --output-dir runs/mpcc_pp_optuna \
  --command "ros2 launch drone_control sim.launch.py controller_type:=mpcc_pp rviz:=false plotjuggler:=false"
```

Il comando di valutazione non deve per forza contenere il path del file YAML:
il runner imposta gia' l'ambiente:

```text
MPCC_PP_PARAMETERS_FILE=/path/to/trial/parameters.yaml
```

Se vuoi usare placeholder espliciti:

```bash
mpcc_pp_optuna_tune \
  --command "python3 my_eval.py --params {parameter_file} --metrics {metrics_file}"
```

Placeholder disponibili:

```text
{parameter_file}
{trial_dir}
{metrics_file}
```

---

## 11. Output prodotto

Per ogni trial:

```text
runs/mpcc_pp_optuna/trial_00000/parameters.yaml
runs/mpcc_pp_optuna/trial_00000/trial_params.json
runs/mpcc_pp_optuna/trial_00000/episode.log
runs/mpcc_pp_optuna/trial_00000/trial_metrics.json
```

Alla fine:

```text
runs/mpcc_pp_optuna/best_parameters.yaml
runs/mpcc_pp_optuna/best_summary.json
runs/mpcc_pp_optuna/study.db
```

`best_parameters.yaml` e' il file che puoi usare per rilanciare il controller
con i migliori parametri trovati.

---

## 12. Sampler disponibili

Default:

```bash
--sampler tpe
```

TPE e' una scelta robusta per cominciare: e' un metodo bayesiano/sequenziale
molto usato in Optuna e lavora bene quando gli episodi sono rumorosi.

Puoi anche usare:

```bash
--sampler random
```

per baseline, oppure:

```bash
--sampler gp
```

se la tua versione di Optuna include `GPSampler`. Se `GPSampler` non e'
disponibile, il runner torna automaticamente a `TPESampler`.

---

## 13. Evaluator custom con JSON metriche

Il runner puo' anche leggere metriche da un file JSON scritto dal tuo evaluator.
Il path e' passato in:

```text
MPCC_PP_METRICS_FILE
```

Schema consigliato:

```json
{
  "success": true,
  "lap_times": [12.4, 12.1, 12.0],
  "solver_fail_rate": 0.02,
  "solve_count": 1800,
  "solver_fail_count": 36,
  "timeout": false,
  "failure_reason": null
}
```

Se il JSON contiene anche `reward`, il runner usa quella reward direttamente.
Questo e' utile se in futuro vuoi includere penalita' piu' ricche, per esempio:

- distanza minima dai gate;
- violazione massima del tunnel;
- crash detector;
- saturazioni attuatori;
- energia o aggressivita' del comando.

---

## 14. Gestione automatica di PX4/Gazebo

Per un tuning serio non conviene tenere Gazebo aperto a mano tra i trial. Se
Gazebo resta vivo, il trial successivo puo' partire dallo stato fisico lasciato
dal trial precedente: posizione, velocita', stato PX4, eventuali warning o
setpoint residui. Questo introduce rumore nella reward e rende meno chiaro se
un parametro e' davvero migliore o ha solo ereditato una condizione iniziale
favorevole.

Per questo e' stato aggiunto il comando:

```bash
mpcc_pp_gazebo_trial
```

Questo comando gestisce un singolo episodio:

1. avvia PX4 SITL + Gazebo;
2. aspetta un pattern di readiness nei log PX4;
3. attende un piccolo delay extra;
4. avvia il launch ROS del controller;
5. inoltra tutti i log a stdout, con prefissi `[PX4]` e `[ROS]`;
6. quando Optuna termina il trial, pulisce PX4, Gazebo e ROS.

Optuna puo' quindi usarlo come comando di valutazione:

```bash
mpcc_pp_optuna_tune \
  --n-trials 30 \
  --laps-per-episode 3 \
  --timeout-s 120 \
  --output-dir runs/mpcc_pp_optuna \
  --command "mpcc_pp_gazebo_trial --px4-dir /home/mattia/ETHRC-Drone-Racing/px4/PX4-Autopilot --px4-model gz_zephyros --ros-command 'ros2 launch drone_control sim.launch.py controller_type:=mpcc_pp rviz:=false plotjuggler:=false'"
```

Il modello PX4/Gazebo e' parametrico. Per esempio, se vuoi usare X500:

```bash
--px4-model gz_x500
```

oppure:

```bash
--px4-model gz_x500_mono_cam
```

Puoi verificare senza avviare nulla:

```bash
mpcc_pp_gazebo_trial \
  --px4-dir /home/mattia/ETHRC-Drone-Racing/px4/PX4-Autopilot \
  --px4-model gz_zephyros \
  --dry-run
```

Il runner scrive anche log separati nella directory del trial:

```text
trial_00000/px4_gazebo.log
trial_00000/ros_launch.log
trial_00000/episode.log
```

`episode.log` e' prodotto da `mpcc_pp_optuna_tune` e contiene tutto quello che
serve per estrarre `[Lap]` e `[SolverStats]`. I due log separati servono per
debug quando PX4/Gazebo parte male o il controller non si connette.

Se PX4 non stampa i pattern di readiness previsti, il runner continua comunque
dopo `--startup-timeout-s`, a meno che tu non passi:

```bash
--strict-px4-ready
```

Puoi aggiungere pattern custom:

```bash
--ready-pattern "Ready for takeoff"
```

La scelta di default e' non essere troppo rigidi, perche' i log PX4/Gazebo
cambiano con modello, versione e world. Per campagne definitive, pero', ha
senso usare `--strict-px4-ready` quando hai identificato un pattern affidabile.

---

## 15. Perche' non usare direttamente `parameters.yaml` manualmente

Si potrebbe far generare a Optuna un file YAML e basta. Funzionerebbe, ma senza
la refactor runtime ogni trial potrebbe ricompilare ACADOS. Il costo vero non e'
scrivere YAML: e' rigenerare il solver.

La separazione attuale e' quindi:

```text
parametri strutturali -> possono richiedere rebuild
parametri di tuning del costo -> runtime, niente rebuild
```

Questa e' la differenza tra una pipeline usabile e una che si blocca dopo pochi
trial.

---

## 16. Cosa resta da migliorare

La pipeline ora e' pronta per tuning automatico, ma ci sono miglioramenti
naturali:

- aggiungere un nodo/evaluator ROS che pubblichi metriche JSON invece di
  affidarsi solo ai log;
- aggiungere crash detection geometrica;
- misurare violazione massima del tunnel;
- salvare bag o telemetria ridotta per i trial migliori;
- passare da TPE a TuRBO/Botorch quando il loop episodio-metriche e' stabile;
- ottimizzare anche parametri separati orizzontale/verticale, piu' fedeli al
  paper MPCC++.

La scelta fatta qui e' intenzionale: prima una pipeline stabile e veloce, poi
una search space piu' ambiziosa.
