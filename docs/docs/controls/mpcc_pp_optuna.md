# MPCC++ — Optuna and tuning

**Owners:** [Max Hassdenteufel](https://github.com/Max1284), [Mattia De Martino](https://github.com/mattiademartino)

The tuning pipeline treats MPCC++ as a black box: Optuna proposes controller weights, a fresh PX4/Gazebo episode evaluates them, and lap, progress, crash, missed-gate, and solver metrics are reduced to one reward. The implementation is under `src/mpcc_pp/mpcc_pp/tuning` and is deliberately separate from the controller internals documented on the [MPCC++ page](mpcc_pp.md).

Status: the full study/evaluator pipeline is implemented, including SQLite resume, repeated episodes, TuRBO through OptunaHub, trial-specific YAML files, log retention, and best-parameter export. Use a dry run and a one-trial smoke test before starting a long study.

## Pipeline

```text
Optuna sampler
      │ proposes 8 values
      ▼
trial_N/parameters.yaml ──► MPCC_PP_PARAMETERS_FILE
                                  │
                                  ▼
                         mpcc_pp_gazebo_trial
                         ├─ PX4 + Gazebo
                         └─ drone_control sim.launch.py
                                  │
                 [Lap] [Progress] [Crash]
                 [MissedGate] [SolverStats]
                                  │
                                  ▼
                       reward per repeat
                                  │
                 mean / median / minimum
                                  │
                                  ▼
                   study.db + best_parameters.yaml
```

One Optuna trial is a parameter vector. It may be evaluated in several fresh simulator repeats; `repeats_per_trial` therefore multiplies the number of simulator episodes without changing `n_trials`.

## Components

| File / executable | Responsibility |
| :--- | :--- |
| `tuning/search_space.py` | parameter bounds, log scaling, derived `qc_gate`, YAML patch |
| `tuning/optuna_tuner.py` / `mpcc_pp_optuna_tune` | study, trial files, episode execution, parsing, reward, resume, export |
| `tuning/gazebo_trial_runner.py` / `mpcc_pp_gazebo_trial` | lifecycle of one clean PX4/Gazebo + ROS episode |
| `tuning/optuna_config.yaml` | checked-in study configuration |
| `scripts/run_autotuning.sh` | older environment-variable wrapper; see known limitations |

## Search space

All eight variables use logarithmic sampling:

| Optuna name | Range | Applied as | Effect |
| :--- | :--- | :--- | :--- |
| `ql` | `1e-2 .. 1e2` | `cost.ql` | lag-error penalty |
| `qc_nom` | `1e-3 .. 1e1` | `path.contouring.qc_nom` | contouring away from gates |
| `qc_gate_ratio` | `1 .. 50` | derived | gate/nominal contouring ratio |
| `Qw` | `1e-7 .. 1e-2` | `cost.Qw` | body-rate regularization |
| `r_dvt` | `1e-7 .. 1e-1` | `cost.r_dvt` | progress-acceleration penalty |
| `R_df` | `1e-7 .. 1e-1` | `cost.R_df` | rotor-thrust-rate penalty |
| `mu` | `1e-2 .. 1e2` | `cost.mu` | progress reward |
| `track_weight` | `1 .. 1e3` | `cost.track_weight` | soft tunnel-barrier penalty |

The derived value is:

```text
qc_gate = qc_nom * qc_gate_ratio
```

This guarantees that tracking at a gate is never weighted below nominal tracking. `SearchSpace.decode()` also supports normalized `[0, 1]^8` coordinates, used by the dry run to create a geometric center point.

Each trial begins with a deep copy of `base_parameters`; only these eight physical values are patched. Model, constraints, horizon, solver, residual settings, visualization, and all other values remain identical across the study.

!!! warning "The checked-in baseline is outside one search bound"
    `parameters.yaml` currently has `qc_nom: 20.0`, while the search-space upper bound is `10.0`. A fresh study enqueues the checked-in parameters as trial 0, so that baseline is not representative of the nominal search domain. Align the bound or baseline before comparing studies.

## Reward and episode validity

For a completed, non-crash episode with the required laps:

```text
reward = 1000
       - sum(required lap times)
       - gamma * solver_fail_rate
       - missed_gate_penalty * missed_gates
```

For an incomplete, crashed, or rejected episode:

```text
reward = 1000 * progress
       - timeout_penalty_multiplier * timeout_s
       - gamma * solver_fail_rate
       - missed_gate_penalty * missed_gates
```

Higher is better. Unlike the formula in the older package guides, the current completed-episode reward uses the **sum**, not the mean, of the required lap times. Incomplete runs retain a progress signal so a near-finish ranks above an immediate failure.

If `solver_fail_rate` exceeds `max_solver_fail_rate`, the episode is marked unsuccessful. A metrics JSON may supply its own `reward`; when present, that value overrides the built-in formula.

For repeated episodes, one trial uses one of:

- `mean`: default; failed repeats continue to affect the result;
- `median`: less sensitive to one outlier;
- `min`: optimize the worst repeat.

## Metrics contract

The tuner exports these environment variables to every evaluation:

| Variable | Meaning |
| :--- | :--- |
| `MPCC_PP_PARAMETERS_FILE` | trial-specific full controller YAML |
| `MPCC_PP_TRIAL_DIR` | directory for this repeat |
| `MPCC_PP_METRICS_FILE` | optional evaluator-written JSON |

If `MPCC_PP_METRICS_FILE` exists, it takes precedence over stdout parsing. Supported JSON fields include `success`, `lap_times`, `solver_fail_rate`, counts, timeout/return status, progress fields, crash/collision, missed gates, and an optional reward.

Without that file, stdout is parsed for:

- `[Lap] ... completato in <seconds> s`;
- `[Progress]` values, current-lap fraction, or completed laps;
- `[Crash]` or `[Collision]`;
- `[MissedGate]`, optionally with `missed_gates=<count>`;
- `[SolverStats] solve_count=... solver_fail_count=... solver_fail_rate=...`.

`racing_controller_node.py` emits these lines in simulation mode. It reports progress every 0.5 s by default, detects sustained ground contact after liftoff as a crash, compares drone-center distance with half the smaller aperture dimension when crossing each gate's progress, and reports solver statistics after a lap.

The tuner ends a repeat immediately on crash, at its wall-clock timeout, or about two seconds after the last required lap to allow final metrics to flush.

## Gazebo trial lifecycle

`mpcc_pp_gazebo_trial` manages one repeat:

1. Optionally terminate stale PX4/Gazebo processes associated with this workspace's PX4 tree.
2. Start `make px4_sitl gz_zephyros` in `px4/PX4-Autopilot`.
3. Wait for a PX4 readiness pattern, then apply `startup_delay_s`.
4. Start `ros2 launch drone_control sim.launch.py controller_type:=mpcc_pp rviz:=false plotjuggler:=false`.
5. Forward only metric lines by default while writing full PX4 and ROS logs.
6. On completion, signal, or failure, terminate both process groups; use SIGKILL only after the grace period.

The runner is headless by default. `--gui`, `--sim-speed-factor`, custom readiness regexes, strict readiness, command overrides, and log-directory overrides are available from its CLI. Its `--dry-run` prints the resolved commands without starting processes.

## Configuration and precedence

Start from the checked-in configuration:

```bash
ros2 run mpcc_pp mpcc_pp_optuna_tune \
  --config src/mpcc_pp/mpcc_pp/tuning/optuna_config.yaml
```

YAML keys use the argparse destination names with underscores; hyphenated forms are also accepted. Unknown keys fail early. Values from the configuration file become parser defaults, so explicit CLI flags win.

The checked-in file currently selects:

| Setting | Value |
| :--- | :--- |
| Sampler | TuRBO through OptunaHub |
| Study | `optuna_turbo_1` |
| Trials | 10 |
| Repeats per trial | 3 |
| Required laps per repeat | 3 |
| Repeat aggregation | mean |
| Repeat timeout | 120 s |
| TuRBO trust regions | 8 |
| Startup trials | 16 per sampler configuration |
| Log retention | failed repeats only |

At three repeats, 10 trials launch 30 fresh episodes. The requested study is also smaller than the configured TuRBO startup budget, so it should be considered a pipeline test rather than a mature Bayesian-optimization run.

Common CLI overrides:

```bash
ros2 run mpcc_pp mpcc_pp_optuna_tune \
  --config src/mpcc_pp/mpcc_pp/tuning/optuna_config.yaml \
  --n-trials 2 \
  --repeats-per-trial 1 \
  --laps-per-episode 1 \
  --timeout-s 90 \
  --sampler tpe \
  --stream-logs
```

Available samplers are `tpe`, `random`, `gp`, and `turbo`. `gp` falls back to TPE when the installed Optuna lacks `GPSampler`. TuRBO loads `samplers/turbo` from OptunaHub and additionally needs OptunaHub, PyTorch, and SciPy.

## Recommended workflow

Build and source both involved packages:

```bash
colcon build --symlink-install --packages-select mpcc_pp drone_control
source install/setup.bash
```

First validate file generation without launching a simulator:

```bash
ros2 run mpcc_pp mpcc_pp_optuna_tune \
  --config src/mpcc_pp/mpcc_pp/tuning/optuna_config.yaml \
  --dry-run
```

This writes `dry_run/parameters.yaml` and `dry_run/trial_params.json` under the configured output directory.

Then run one short random or TPE smoke trial:

```bash
ros2 run mpcc_pp mpcc_pp_optuna_tune \
  --config src/mpcc_pp/mpcc_pp/tuning/optuna_config.yaml \
  --study-name mpcc_pp_smoke \
  --output-dir /tmp/mpcc_pp_smoke \
  --sampler random \
  --n-trials 1 \
  --repeats-per-trial 1 \
  --laps-per-episode 1 \
  --stream-logs
```

Check that PX4 becomes ready, ROS starts, lap/progress lines reach the tuner, cleanup stops all child processes, and `episode_metrics.json` explains the reward. Only then increase laps, repeats, and trial count.

## Resume and outputs

The output directory contains one SQLite study and per-trial artifacts:

```text
output_dir/
├── study.db
├── best_parameters.yaml
├── best_summary.json
└── trial_00000/
    ├── parameters.yaml
    ├── trial_params.json
    ├── trial_metrics.json
    └── repeat_00/
        ├── episode_metrics.json
        ├── episode.log          # depending on retention policy
        ├── px4_gazebo.log       # depending on retention policy
        └── ros_launch.log       # depending on retention policy
```

Optuna uses `load_if_exists=True`. Reusing both `output_dir` and `study_name` resumes the database; `n_trials` means additional trials for that invocation. The checked-in baseline is enqueued only when the study has no trials.

After every run, including interruption, the tuner attempts to export the current best trial:

- `best_parameters.yaml` is the complete base parameter file with the winning tuning patch applied, ready for the controller;
- `best_summary.json` records the reward, trial number, expanded parameters, user attributes, and YAML path.

Evaluate a winner by pointing the normal controller launch at that full YAML:

```bash
MPCC_PP_PARAMETERS_FILE=/absolute/path/to/best_parameters.yaml \
  ros2 launch drone_control sim.launch.py controller_type:=mpcc_pp
```

Do not promote a single optimization result directly to hardware. Re-evaluate it with new seeds/episodes, retain a known baseline, inspect missed gates and solver health, and compare on the same track and simulator settings.

## Log retention

`log_retention` controls only `*.log` files after metrics have been parsed:

| Value | Result |
| :--- | :--- |
| `all` | keep every repeat log |
| `failed` | keep logs for unsuccessful repeats only |
| `none` | remove all repeat logs; keep YAML/JSON metrics |

The episode summary records how many logs were removed and clears its `log_path` if that file was deleted.

## Troubleshooting

| Symptom | Likely cause | Action |
| :--- | :--- | :--- |
| `ros2 run` cannot find an executable | package not rebuilt or workspace not sourced | rebuild `mpcc_pp`, then source `install/setup.bash` |
| PX4 directory does not exist | workspace layout differs from the runner's source/install inference | run `mpcc_pp_gazebo_trial` with `--px4-dir` while diagnosing; adjust the runner for the study |
| Readiness warning, then ROS starts too early | console output does not match default readiness patterns | add `--ready-pattern` for direct runner tests or fix the pattern set; use strict readiness while debugging |
| No lap metrics | racing node did not start in simulation/MPCC mode, or metric lines were not forwarded | use `--stream-logs`; inspect `ros_launch.log` and `episode.log` |
| Every incomplete run has similar score | no progress lines are arriving | verify `[Progress] completed_laps=... lap_fraction=...` in the logs |
| TuRBO import fails | missing OptunaHub/PyTorch/SciPy or unavailable hub package | install compatible dependencies or use `--sampler tpe` |
| ACADOS recompiles unexpectedly | a structural parameter changed between trials or generated cache is stale | keep base model/solver settings fixed; inspect ACADOS generation output |
| Existing trial results appear unexpectedly | same SQLite path and study name resumed | choose a new output directory or study name for a fresh study |
| Useful logs disappeared | `log_retention` removed successful/all logs | use `--log-retention all` while debugging |

## Known limitations

- `scripts/run_autotuning.sh` still passes `--command`, but the current `mpcc_pp_optuna_tune` parser no longer defines that option. Use the YAML/CLI invocation above until the wrapper is updated.
- Solver failure statistics can under-count hard failures because `DroneOptimizerMpccPp.solve()` raises before the controller records the returned status, and the alternative per-step status print parsed by the tuner is commented out.
- Missed-gate detection uses Euclidean distance to the gate center at a progress crossing; it is not a geometric gate-plane intersection test.
- Crash detection is a simulation heuristic based on sustained NED-down ground contact after liftoff, not a general collision detector.
- The tuner runs trials sequentially. `n_trust_region: 8` configures TuRBO's search behavior; it does not launch eight simulator workers.
- Relative paths in `optuna_config.yaml` are resolved from the process working directory. Run from the workspace root or use absolute paths.
- `setup.py` declares `optunahub` but not PyTorch, which both TuRBO and the residual MLP may require.
- Generated best files are only as comparable as the fixed base parameters, track, simulator version, startup state, and metric behavior used by their study.
