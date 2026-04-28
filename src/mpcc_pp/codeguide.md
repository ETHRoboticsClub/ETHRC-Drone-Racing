# MPCC++ Automatic Tuning: Code Guide

This guide describes the full automatic tuning pipeline for the `mpcc_pp` controller.
It covers what was implemented, what was changed in the code, why those changes were
necessary, and the recommended path toward TuRBO Bayesian Optimization as described in
the MPCC++ paper.

The objective is to build a repeatable loop:

```text
optimizer -> parameter vector phi -> simulation episode -> metrics -> reward -> optimizer
```

The optimizer maximizes a scalar reward. Higher reward means faster laps, fewer ACADOS
solver failures, and complete episodes without timeout.

---

## 1. Objective

The MPCC++ paper treats the controller as a black-box function. For a given parameter
vector `phi`, run an episode in simulation, collect the resulting trajectory, and compute
one scalar reward.

In the paper, one tuning episode is:

```text
M = 3 consecutive laps around the track
```

The reward is:

```text
R(phi) = -mean(lap_times) - gamma * solver_failure_rate
```

with:

```text
gamma = 100
```

where:

- `lap_times` are the times for completed laps in the episode,
- `solver_failure_rate` is the fraction of controller steps where the ACADOS solver did
  not return an acceptable status,
- failed or crashed episodes receive a very low fixed reward.

The optimizer maximizes `R(phi)`. Since lap time enters negated, faster laps produce a
higher reward. Since solver failures are penalized with weight 100, the optimizer learns
to avoid aggressive parameter regions that may be fast but numerically unstable.

---

## 2. What Already Exists in the Repository

The `mpcc_pp` package already contains the full MPCC++ controller:

- `mpcc_pp/drone_model.py`: augmented quadrotor model with position, attitude,
  velocity, body rates, rotor thrust states, path progress `theta`, and progress speed
  `vtheta`.
- `mpcc_pp/drone_opt.py`: ACADOS OCP with contouring cost, lag cost, progress reward,
  rotor-rate penalties, body-rate penalties, and a soft tunnel barrier.
- `mpcc_pp/drone_mpcc_pp_controller.py`: controller wrapper used by the ROS 2 control
  node.
- `mpcc_pp/path_utils.py`: arc-length path, dynamic contouring weight `qc`, gated
  tunnel geometry, and tunnel half-width/half-height computation.
- `drone_control/config/gates_config.yaml`: track definitions and current manual MPCC++
  parameters.

What was missing was an automatic tuning harness. The sections below describe what was
added and what was changed to make tuning possible.

---

## 3. Why the Code Needed Refactoring First

### The Compile-Time Weight Problem

Before the modification, the main MPCC++ cost weights were embedded directly into the
symbolic CasADi/ACADOS cost expression:

```python
cost_lag      = w['ql']           * el**2
cost_omega    = w['Qw']           * dot(w_k, w_k)
cost_dvt      = w['r_dvt']        * dvt_k**2
cost_df       = w['R_df']         * dot(df_k, df_k)
cost_progress = -w['mu']          * vt_k
cost_track    = w['track_weight'] * barrier_sum
```

This is fine for manual tuning, but fatal for Bayesian Optimization. Each trial proposes
a new parameter vector. If those weights are baked into the symbolic problem as compile-
time constants, ACADOS may need to regenerate and recompile the solver for every trial.
With hundreds of trials this becomes completely impractical.

Additionally, the optimizer included the current weight values in the ACADOS build hash:

```python
config_str = (... f"{sorted(w.items())}:" ...)
```

This guaranteed a rebuild every time any weight changed.

### The Required Fix

Move all tunable controller parameters from compile-time constants to runtime ACADOS
stage parameters. This allows TuRBO or Optuna to change `ql`, `Qw`, `r_dvt`, `R_df`,
`mu`, and `track_weight` between episodes without touching the solver binary.

Result:
- the first startup may recompile because the total number of ACADOS stage parameters
  changed,
- subsequent trials reuse the same compiled solver and only change `acados_solver.set()`
  values,
- the build hash no longer includes the tuning weight values.

Changing `n_nodes`, `t_horizon`, constraints, model structure, solver options, or
`track_barrier_alpha` still requires a rebuild because those actually change the
generated solver code.

---

## 4. Files Modified and Added

### Modified

- `mpcc_pp/drone_opt.py`: extended stage parameters from 15 to 21, added runtime weight
  symbols, updated cost and terminal cost expressions, updated build hash.
- `mpcc_pp/drone_mpcc_pp_controller.py`: added `_runtime_weights`, `set_tuning_parameters`,
  `_append_weight_params`, solver statistics counters, lap timestamp tracking,
  `reset_episode_stats`, `get_solver_stats`.
- `drone_control/drone_controller_node.py`: added logging of solver statistics on lap
  completion, reads `MPCC_PP_PARAMETERS_FILE` environment variable.
- `mpcc_pp/setup.py`: added `mpcc_pp_optuna_tune` and `mpcc_pp_gazebo_trial` console
  scripts, added optional `tuning` extras.

### Added

- `mpcc_pp/tuning/__init__.py`
- `mpcc_pp/tuning/search_space.py`
- `mpcc_pp/tuning/optuna_tuner.py`
- `mpcc_pp/tuning/gazebo_trial_runner.py`

---

## 5. New ACADOS Stage Parameter Layout

Before the change, stage parameters were 15 fields:

```text
[0:3]   p_ref         path reference position
[3:6]   t_ref         tangent vector
[6]     qc            dynamic contouring weight (from path_utils)
[7:10]  n_ref         normal vector
[10:13] b_ref         binormal vector
[13]    W             tunnel half-width
[14]    H             tunnel half-height
```

After the change, they are 21 fields:

```text
[0:3]   p_ref
[3:6]   t_ref
[6]     qc
[7:10]  n_ref
[10:13] b_ref
[13]    W
[14]    H
[15]    ql            lag error weight           (runtime tunable)
[16]    Qw            body-rate regularization   (runtime tunable)
[17]    r_dvt         progress accel penalty     (runtime tunable)
[18]    R_df          rotor thrust-rate penalty  (runtime tunable)
[19]    mu            progress reward            (runtime tunable)
[20]    track_weight  soft tunnel barrier weight (runtime tunable)
```

The first 15 values describe path geometry and the local tunnel frame. The last 6 are
the runtime tunable weights.

`path_utils.py` continues to produce only the first 15 values. This is intentional:
path geometry should not know about tuning. The 6 weight columns are appended by the
controller in `_append_weight_params`.

---

## 6. Changes in `drone_opt.py`

`NP` changes from `15` to `21`.

New CasADi symbols are added to the parameter vector:

```python
ql_p           = ca.MX.sym("ql", 1)
Qw_p           = ca.MX.sym("Qw", 1)
r_dvt_p        = ca.MX.sym("r_dvt", 1)
R_df_p         = ca.MX.sym("R_df", 1)
mu_p           = ca.MX.sym("mu", 1)
track_weight_p = ca.MX.sym("track_weight", 1)

params = ca.vertcat(
    p_ref, t_ref, qc, n_ref, b_ref, W, H,
    ql_p, Qw_p, r_dvt_p, R_df_p, mu_p, track_weight_p,
)
```

The stage cost uses these runtime symbols:

```python
cost_lag      = ql_p           * el**2
cost_omega    = Qw_p           * ca.dot(w_k, w_k)
cost_dvt      = r_dvt_p        * dvt_k**2
cost_df       = R_df_p         * ca.dot(df_k, df_k)
cost_progress = -mu_p          * vt_k
cost_track    = track_weight_p * barrier_sum
```

The terminal cost uses the same runtime symbols.

The build hash no longer contains `sorted(w.items())`. It only contains structural
quantities: horizon length, number of nodes, model hash, constraint structure, parameter
count, barrier slope, and solver options.

`solve()` now returns three values:

```python
return u_opt, x_pred, info
```

`info` contains:

```text
status          ACADOS integer status code
success         True if status in (0, 2)
finite_solution True if predicted trajectory contains no NaN or Inf
time_tot        total ACADOS solve time in seconds
time_lin        linearization time
time_sim        simulation time inside ACADOS
time_qp         QP solve time
u_pred          full predicted input sequence
```

For backward compatibility, `solve()` accepts a legacy matrix of shape `(N+1, 15)`.
In that case it appends default weights loaded from YAML. Controllers already updated
to the new layout send `(N+1, 21)`.

---

## 7. Changes in `drone_mpcc_pp_controller.py`

A runtime weight dictionary is added:

```python
self._runtime_weights = {
    "ql":           tp.get("ql", 7.0),
    "Qw":           tp.get("Qw", 0.0001),
    "r_dvt":        tp.get("r_dvt", 0.0002),
    "R_df":         tp.get("R_df", 0.0003),
    "mu":           tp.get("mu", 7.0),
    "track_weight": tp.get("track_weight", 30.0),
}
```

The tuning harness updates these values before each episode by calling:

```python
controller.set_tuning_parameters(params)
```

`set_tuning_parameters` accepts:

```text
ql
Qw
r_dvt
R_df
mu
track_weight
qc_nom
qc_gate
qc_gate_ratio
```

If `qc_gate_ratio` is passed, the controller computes:

```python
qc_gate = qc_nom * qc_gate_ratio
```

This constraint prevents trials where the tracking weight near gates falls below the
nominal tracking weight elsewhere on the track.

Every step, the controller builds stage parameters as:

```python
stage_params = self._path.get_tunnel_params(...)   # shape (N+1, 15)
stage_params = self._append_weight_params(stage_params)  # shape (N+1, 21)
u_opt, x_pred, info = self._opt.solve(x_aug, stage_params)
```

`_append_weight_params`:

```python
def _append_weight_params(self, stage_params: np.ndarray) -> np.ndarray:
    extra = np.array([
        self._runtime_weights["ql"],
        self._runtime_weights["Qw"],
        self._runtime_weights["r_dvt"],
        self._runtime_weights["R_df"],
        self._runtime_weights["mu"],
        self._runtime_weights["track_weight"],
    ], dtype=np.float64)
    repeated = np.repeat(extra.reshape(1, -1), stage_params.shape[0], axis=0)
    return np.hstack([stage_params, repeated])
```

---

## 8. Solver Metrics

To distinguish fast-but-unstable from slow-but-reliable parameter sets, the controller
now tracks solver statistics per episode.

Counters added:

```text
_solve_count
_solver_fail_count
_status_counts
_last_status
```

A solve is counted as a failure if ACADOS returns a non-accepted status or if the
predicted trajectory contains NaN or Inf. Accepted statuses are `0` and `2`.

Exposed via:

```python
controller.get_solver_stats()
```

Returns:

```text
solve_count
solver_fail_count
solver_fail_rate      = solver_fail_count / solve_count
status_counts         dict from status int to count
last_status
```

Reset at the start of each episode:

```python
controller.reset_episode_stats()
```

This also resets `lap_count`, path progress, and optionally warm-start state.

---

## 9. Lap Time Tracking

The controller adds a timestamp every time `lap_count` increments:

```python
self._lap_timestamps.append(time.monotonic())
```

Lap times are computed as differences between consecutive timestamps:

```python
def get_lap_times(self) -> list[float]:
    if len(self._lap_timestamps) < 2:
        return []
    return [
        self._lap_timestamps[i] - self._lap_timestamps[i - 1]
        for i in range(1, len(self._lap_timestamps))
    ]
```

The lead-in from start position to the first gate should not count as a lap. Two
conventions work:

- start timing only after the first lap wrap, and require `laps_per_episode + 1`
  internal wraps,
- record a `race_start_time` when the controller first completes the lead-in theta,
  and compute the first lap interval from that point.

The first is simpler. The second is cleaner if the lead-in takes variable time.

---

## 10. ROS Node Logging

When a lap completes, `drone_controller_node.py` logs:

```text
[Lap] circuit 1 completed in 12.34 s
[SolverStats] solve_count=1800 solver_fail_count=2 solver_fail_rate=0.001 last_status=0
```

The Optuna runner reads these lines from the subprocess stdout to extract metrics without
needing a ROS service or shared memory. This makes the process-based runner simple and
robust.

---

## 11. Tunable Parameter Set

### Fast Integration Space (Implemented)

Tune these 8 parameters:

```text
ql             lag error weight
qc_nom         contouring weight away from gates
qc_gate_ratio  multiplier: qc_gate = qc_nom * qc_gate_ratio
Qw             body-rate regularization
r_dvt          progress acceleration penalty
R_df           rotor thrust-rate penalty
mu             progress reward magnitude
track_weight   soft tunnel barrier weight
```

Meaning:

- `ql`: higher values force the drone to stay on schedule along the path direction.
- `qc_nom`: lower values allow wider shortcuts in open track sections.
- `qc_gate_ratio`: ensures gate tracking is at least as tight as open-track tracking.
- `Qw`: higher values smooth angular motion but reduce agility.
- `r_dvt`: higher values damp noisy progress speed changes.
- `R_df`: higher values smooth rotor thrust commands but reduce aggressiveness.
- `mu`: higher values push the controller to fly faster at the cost of safety margins.
- `track_weight`: higher values enforce the tunnel constraint more aggressively.

### Paper-Faithful Space (Future)

The MPCC++ paper splits contouring and angular-rate penalties by axis:

```text
Ql
Qc_horizontal
Qc_vertical
Qomega_horizontal
Qomega_vertical
Rvtheta
Rdelta_f
mu
```

Implementing this requires replacing:

```python
cost_contour = qc * ca.dot(ec, ec)
cost_omega   = Qw * ca.dot(w_k, w_k)
```

with:

```python
e_n = ca.dot(n_ref, e)
e_b = ca.dot(b_ref, e)

cost_contour = qc_h * e_n**2 + qc_v * e_b**2
cost_omega   = Qw_xy * (w_k[0]**2 + w_k[1]**2) + Qw_z * w_k[2]**2
```

Build the minimal version first. Add the paper-faithful version after the pipeline is
working reliably.

---

## 12. Search Space Design

All parameters are tuned in log scale because controller weights span many orders of
magnitude. A linear search space would waste most trials in uninformative regions.

The normalized coordinate `z` lives in `[0, 1]`. The physical value is:

```python
physical = 10 ** (log10(low) + z * (log10(high) - log10(low)))
```

Recommended ranges:

```text
ql:            1e-2 .. 1e2
qc_nom:        1e-3 .. 1e1
qc_gate_ratio: 1e0  .. 5e1
Qw:            1e-7 .. 1e-2
r_dvt:         1e-7 .. 1e-1
R_df:          1e-7 .. 1e-1
mu:            1e-2 .. 1e2
track_weight:  1e0  .. 1e3
```

`qc_gate` is not tuned directly. It is derived:

```python
qc_gate = qc_nom * qc_gate_ratio
```

`search_space.py` recommended API:

```python
from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class ParameterSpec:
    name: str
    low: float
    high: float
    scale: str = "log"


class SearchSpace:
    def __init__(self):
        self.specs = [
            ParameterSpec("ql", 1e-2, 1e2),
            ParameterSpec("qc_nom", 1e-3, 1e1),
            ParameterSpec("qc_gate_ratio", 1.0, 50.0),
            ParameterSpec("Qw", 1e-7, 1e-2),
            ParameterSpec("r_dvt", 1e-7, 1e-1),
            ParameterSpec("R_df", 1e-7, 1e-1),
            ParameterSpec("mu", 1e-2, 1e2),
            ParameterSpec("track_weight", 1.0, 1e3),
        ]

    @property
    def dim(self) -> int:
        return len(self.specs)

    def decode(self, z: np.ndarray) -> dict:
        params = {}
        for value, spec in zip(z, self.specs):
            if spec.scale == "log":
                lo = np.log10(spec.low)
                hi = np.log10(spec.high)
                params[spec.name] = float(10 ** (lo + value * (hi - lo)))
            else:
                params[spec.name] = float(spec.low + value * (spec.high - spec.low))
        params["qc_gate"] = params["qc_nom"] * params.pop("qc_gate_ratio")
        return params
```

---

## 13. Reward

Keep reward computation isolated and testable in `reward.py`.

```python
from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class EpisodeResult:
    success: bool
    lap_times: list[float]
    solver_fail_rate: float
    min_tunnel_margin: float | None
    timeout: bool
    reason: str


def compute_reward(
    result: EpisodeResult,
    gamma: float = 100.0,
    failed_episode_reward: float = -1000.0,
) -> float:
    if not result.success or not result.lap_times:
        return failed_episode_reward
    return -float(np.mean(result.lap_times)) - gamma * float(result.solver_fail_rate)
```

For failed episodes the Optuna implementation uses:

```text
failed_reward = -1e6
```

An episode is marked as failed if:

- it does not complete the required number of laps,
- it times out,
- `solver_fail_rate` exceeds `max_solver_fail_rate`.

If TuRBO finds policies that aggressively cut through the tunnel, add an optional
safety penalty later:

```python
reward -= safety_weight * max(0.0, -min_tunnel_margin)
```

Start with `safety_weight = 20`. Do not add this term in the first version; keep the
reward faithful to the paper.

---

## 14. Tunnel Safety Metric

Log the actual tunnel margin during every episode. For current vehicle position `p`,
path reference `p_ref`, local normal `n_ref`, binormal `b_ref`, half-width `W`, and
half-height `H`:

```python
e = p - p_ref
e_n = np.dot(n_ref, e)
e_b = np.dot(b_ref, e)

h1 = W + e_n
h2 = W - e_n
h3 = H + e_b
h4 = H - e_b

margin = min(h1, h2, h3, h4)
```

Interpretation:

```text
margin >= 0  drone is inside the tunnel
margin < 0   drone is outside the tunnel by abs(margin) meters
```

Log three quantities per episode:

```text
min_tunnel_margin
mean_tunnel_margin
max_tunnel_violation = max(0.0, -min_tunnel_margin)
```

Do not add this to the reward initially. The tunnel is already penalized inside the
MPCC++ cost via `track_weight`. Use margin as a diagnostic and for early termination.

---

## 15. Episode Runner

The runner evaluates one parameter vector in simulation.

### Process-Based Runner (Implemented)

`optuna_tuner.py` uses a process-based approach:

1. load a base `parameters.yaml`,
2. ask the optimizer for a parameter vector,
3. write a trial-specific `parameters.yaml`,
4. launch the simulation command as a subprocess,
5. export `MPCC_PP_PARAMETERS_FILE` pointing to the trial config,
6. read `[Lap]` and `[SolverStats]` lines from stdout,
7. compute reward,
8. save trial metrics,
9. update the optimizer.

Passing parameters through environment variables and log parsing requires no changes to
the ROS node API and works with any launch command.

### In-Process Runner (Future)

An in-process runner creates `DroneMpccPpController` directly, calls `initialize()`,
and simulates dynamics in Python using the ACADOS model or a simple integrator. This
is faster to iterate and easier to parallelize, but does not reflect PX4/MAVSDK command
delays, Gazebo dynamics, or offboard control latency.

Strategy:

```text
Use the process-based runner for final tuning and validation.
Use the in-process runner only if simulation throughput becomes the bottleneck.
```

---

## 16. Optimizer: Optuna (Current)

`optuna_tuner.py` adds the ROS executable:

```bash
ros2 run mpcc_pp mpcc_pp_optuna_tune
```

The loop:

```python
space = SearchSpace()
study = optuna.create_study(direction="maximize", sampler=sampler, storage=storage)

for trial_idx in range(n_trials):
    trial = study.ask()
    params = space.suggest(trial)
    result = runner.evaluate(params)
    reward = compute_reward(result)
    study.tell(trial, reward)
    storage.append_trial(...)
```

Available samplers:

```text
tpe     TPE (default, robust, sequentially Bayesian)
random  random search baseline
gp      Gaussian Process sampler (requires recent Optuna)
```

TPE is a strong default for noisy episodes. It handles missing or failed trials
gracefully and does not require many initial points to be useful.

---

## 17. Optimizer: TuRBO (Planned)

The MPCC++ paper uses TuRBO with a Gaussian Process surrogate and UCB acquisition. This
is the recommended upgrade path after the Optuna loop is stable.

Dependencies:

```text
torch
botorch
gpytorch
```

### Initial Sobol Samples

Before fitting a GP, collect low-discrepancy initial samples:

```text
n_init = 2 * dim = 16  (for 8 parameters)
```

```python
from torch.quasirandom import SobolEngine

sobol = SobolEngine(dimension=dim, scramble=True, seed=seed)
X_init = sobol.draw(n_init)
```

### GP Surrogate

```python
from botorch.models import SingleTaskGP
from botorch.fit import fit_gpytorch_mll
from gpytorch.mlls import ExactMarginalLogLikelihood

train_X = torch.tensor(X, dtype=torch.double)
train_Y = torch.tensor(Y, dtype=torch.double).unsqueeze(-1)

model = SingleTaskGP(train_X, train_Y)
mll = ExactMarginalLogLikelihood(model.likelihood, model)
fit_gpytorch_mll(mll)
```

Normalize rewards before fitting:

```python
Y_norm = (Y - Y.mean()) / (Y.std() + 1e-8)
```

### UCB Acquisition

```python
from botorch.acquisition import UpperConfidenceBound

acq = UpperConfidenceBound(model, beta=2.0)
```

### Trust Region

TuRBO runs local BO within a trust region around the current best point.

State:

```python
length = 0.8
length_min = 0.5 ** 7
length_max = 1.6
success_counter = 0
failure_counter = 0
success_tolerance = 3
failure_tolerance = ceil(max(4, dim) / batch_size)
```

After each evaluation:

- new reward improves best: increment `success_counter`, reset `failure_counter`.
- otherwise: increment `failure_counter`, reset `success_counter`.

If `success_counter == success_tolerance`:

```python
length = min(2.0 * length, length_max)
```

If `failure_counter == failure_tolerance`:

```python
length = length / 2.0
```

If `length < length_min`: restart the trust region from a new Sobol point or from the
best known candidate.

Trust region bounds:

```python
lower = torch.clamp(x_center - length / 2.0, 0.0, 1.0)
upper = torch.clamp(x_center + length / 2.0, 0.0, 1.0)
```

Candidate generation:

```python
from botorch.optim import optimize_acqf

candidate, _ = optimize_acqf(
    acq_function=acq,
    bounds=torch.stack([lower, upper]),
    q=1,
    num_restarts=10,
    raw_samples=256,
)
```

### TuRBO-1 Loop

```python
space = SearchSpace()
runner = EpisodeRunner(...)
turbo = TurboOptimizer(dim=space.dim)

for trial_idx in range(total_episodes):
    z = turbo.ask()
    params = space.decode(z)
    result = runner.evaluate(params, seed=base_seed + trial_idx)
    reward = compute_reward(result)
    turbo.tell(z, reward)
    storage.append_trial(...)
```

---

## 18. From TuRBO-1 to TuRBO-8

Do not start with TuRBO-8.

Build in this order:

```text
1. Sobol random search works end-to-end.
2. TuRBO-1 works for 30-50 episodes.
3. Logs, checkpoints, and resume work.
4. Final validation works.
5. Only then add TuRBO-8.
```

TuRBO-8 runs 8 independent trust regions and evaluates candidates in parallel. Each
worker needs isolated simulation resources:

```text
ROS_DOMAIN_ID         different per worker
PX4 instance ID       different per worker
MAVLink UDP ports     different per worker
Gazebo namespace      different per worker
run subdirectory      different per worker
random seed           different per worker
```

Example allocation:

```text
worker 0: ROS_DOMAIN_ID=30, MAVLink 14540
worker 1: ROS_DOMAIN_ID=31, MAVLink 14541
...
worker 7: ROS_DOMAIN_ID=37, MAVLink 14547
```

If multi-instance PX4/Gazebo is not reliable, run TuRBO-8 with 8 trust regions but
evaluate candidates sequentially. This still improves global exploration without
wall-clock speedup.

---

## 19. Passing Parameters to the ROS Controller

There are three options for delivering a new parameter vector to the running controller
between episodes.

### Option 1: Temporary YAML File (Implemented)

For each trial, write a trial-specific `parameters.yaml` and pass its path via:

```bash
MPCC_PP_PARAMETERS_FILE=/path/to/trial/parameters.yaml
```

The controller reads this file at startup. This restarts the controller process for
every trial, which is slow but completely reliable and requires no ROS service changes.

### Option 2: ROS Service (Recommended Long-Term)

Add a service to `drone_controller_node`:

```text
/mpcc_pp/set_tuning_parameters
```

The service receives parameter names and values, then calls:

```python
self.controller.set_tuning_parameters(params)
self.controller.reset_episode_stats()
```

This allows one long-lived controller process. Much faster between trials.

### Option 3: ROS Parameters

Declare ROS parameters `mpcc_pp.ql`, `mpcc_pp.qc_nom`, etc., and use dynamic parameter
updates. Idiomatic ROS, but nested dictionaries and type coercion become messy. A
dedicated service is usually clearer.

Recommended sequence:

```text
Start with temporary YAML files.
Move to a ROS service once reward logic is correct.
```

---

## 20. Automatic PX4/Gazebo Management

Keeping Gazebo open between trials introduces noise: the next trial inherits the
physical state, velocity, PX4 mode, and residual setpoints of the previous one. This
makes it unclear whether a parameter set is genuinely better or just received a favorable
initial condition.

The ROS executable `mpcc_pp_gazebo_trial` manages one clean episode:

1. starts PX4 SITL and Gazebo,
2. waits for PX4 readiness patterns in the log,
3. applies a small extra startup delay,
4. starts the ROS controller launch,
5. forwards all logs to stdout with `[PX4]` and `[ROS]` prefixes,
6. on episode end, cleans up PX4, Gazebo, and ROS processes.

Usage with the Optuna tuner:

```bash
ros2 run mpcc_pp mpcc_pp_optuna_tune \
  --n-trials 30 \
  --laps-per-episode 3 \
  --timeout-s 120 \
  --output-dir src/mpcc_pp/mpcc_pp_optuna_runs \
  --command "ros2 run mpcc_pp mpcc_pp_gazebo_trial \
    --px4-model gz_zephyros \
    --startup-delay-s 1.0 \
    --ros-command 'ros2 launch drone_control sim.launch.py \
      controller_type:=mpcc_pp rviz:=false plotjuggler:=false'"
```

The trial runner uses Gazebo headless mode by default, so no Gazebo GUI, RViz, or
PlotJuggler windows are launched. To inspect one run visually, add `--gui` to the
`mpcc_pp_gazebo_trial` command. To try faster-than-real-time simulation, add for
example `--sim-speed-factor 2.0`; keep this modest and verify lap detection still
looks sane.

Before each trial, `mpcc_pp_gazebo_trial` clears stale PX4/Gazebo processes from
this workspace. This avoids `PX4 server already running for instance 0` after an
interrupted run. Add `--keep-existing` only if you intentionally want to attach to
an already running simulator.

PX4 and ROS logs are written to each trial directory by default, but are not mirrored
to the terminal. Add `--stream-logs` to `mpcc_pp_gazebo_trial`, or run the helper
with `STREAM=1`, when debugging startup.

Dry-run to verify without starting anything:

```bash
ros2 run mpcc_pp mpcc_pp_gazebo_trial \
  --px4-model gz_zephyros \
  --dry-run
```

The runner writes separate logs per trial:

```text
trial_00000/px4_gazebo.log
trial_00000/ros_launch.log
trial_00000/episode.log
```

`episode.log` contains the merged stdout that `mpcc_pp_optuna_tune` parses for `[Lap]`
and `[SolverStats]` lines. The separate logs are for debugging startup failures.

If PX4 does not emit expected readiness patterns, the runner continues after
`--startup-timeout-s` unless `--strict-px4-ready` is set. Start without strict mode
until you identify a reliable readiness pattern for your PX4/Gazebo version.

---

## 21. Custom Metrics via JSON

The runner can also read metrics from a JSON file written by a custom evaluator. The
path is passed as:

```text
MPCC_PP_METRICS_FILE
```

Recommended schema:

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

If the JSON contains a `reward` field, the runner uses it directly. This is useful for
adding richer penalties later:

- minimum gate clearance,
- maximum tunnel violation distance,
- crash detection from geometry,
- actuator saturation rate,
- total energy consumption.

---

## 22. Storage and Logging

Every trial must be saved immediately. Do not buffer. If a run is interrupted, no trial
should be lost.

Run directory layout:

```text
src/mpcc_pp/mpcc_pp_optuna_runs/
  trial_00000/
    parameters.yaml
    trial_params.json
    episode.log
    trial_metrics.json
  trial_00001/
    ...
  best_parameters.yaml
  best_summary.json
  study.db
```

Per trial, one JSON line in `trials.jsonl`:

```json
{
  "trial": 12,
  "seed": 1234,
  "z": [0.1, 0.7, 0.4, 0.6, 0.3, 0.5, 0.8, 0.2],
  "params": {
    "ql": 3.1,
    "qc_nom": 0.2,
    "qc_gate": 12.5,
    "Qw": 0.00004,
    "r_dvt": 0.0001,
    "R_df": 0.0002,
    "mu": 8.5,
    "track_weight": 25.0
  },
  "reward": -5.31,
  "success": true,
  "lap_times": [5.28, 5.35, 5.30],
  "solver_fail_rate": 0.0,
  "min_tunnel_margin": 0.11,
  "reason": "completed"
}
```

Also write `trials.csv` for quick plotting in any spreadsheet or Python script.

Update `best_parameters.yaml` whenever a new best reward appears:

```yaml
reward: -5.31
params:
  ql: 3.1
  qc_nom: 0.2
  qc_gate: 12.5
  Qw: 0.00004
  r_dvt: 0.0001
  R_df: 0.0002
  mu: 8.5
  track_weight: 25.0
episode:
  lap_times: [5.28, 5.35, 5.30]
  solver_fail_rate: 0.0
  min_tunnel_margin: 0.11
```

This file is directly portable into `drone_control/config/gates_config.yaml`.

---

## 23. Integration Back Into `gates_config.yaml`

Once tuning is complete, copy the best values into the active track section:

```yaml
track:
  SHOW_TUNNEL: true
  W_nominal: 3.0
  H_nominal: 3.0
  sigma_gate: 3.0
  qc_nom: 0.42
  qc_gate: 12.7
  sigma_qc: 0.5
  track_weight: 31.5
  ql: 4.2
  Qw: 0.00003
  r_dvt: 0.00018
  R_df: 0.00021
  mu: 8.9
```

`DroneMpccPpController.__init__` already reads these optional keys:

```python
self._runtime_weights = {
    "ql":           tp.get("ql", 7.0),
    "Qw":           tp.get("Qw", 0.0001),
    "r_dvt":        tp.get("r_dvt", 0.0002),
    "R_df":         tp.get("R_df", 0.0003),
    "mu":           tp.get("mu", 7.0),
    "track_weight": tp.get("track_weight", 30.0),
}
```

This keeps manual and tuned workflows unified under the same configuration file.

---

## 24. CLI Usage

Build and source:

```bash
colcon build --packages-select mpcc_pp drone_control
source install/setup.bash
```

Because `mpcc_pp` is an `ament_python` package, `setup.cfg` installs console scripts
under `install/mpcc_pp/lib/mpcc_pp`, which is where `ros2 run` looks for them. They are
not installed as bare shell commands on `PATH`, so `mpcc_pp_optuna_tune` by itself can
print `command not found` even when the package is installed correctly.

Rebuild `mpcc_pp`, source the workspace setup file from the repository root, and invoke
the executable through `ros2 run`:

```bash
colcon build --symlink-install --packages-select mpcc_pp drone_control
source install/setup.bash
ros2 run mpcc_pp mpcc_pp_optuna_tune --help
```

Run `source install/setup.bash` again in every new terminal before using the command. If
there is no `install/` directory, the workspace has not been built in that environment
yet. The direct installed path, after building, is usually:

```bash
./install/mpcc_pp/lib/mpcc_pp/mpcc_pp_optuna_tune
```

Verify file generation without launching simulation:

```bash
ros2 run mpcc_pp mpcc_pp_optuna_tune \
  --dry-run \
  --output-dir src/mpcc_pp/mpcc_pp_optuna_runs
```

This creates:

```text
src/mpcc_pp/mpcc_pp_optuna_runs/dry_run/parameters.yaml
src/mpcc_pp/mpcc_pp_optuna_runs/dry_run/trial_params.json
```

Short smoke test with ROS simulation:

```bash
ros2 run mpcc_pp mpcc_pp_optuna_tune \
  --n-trials 10 \
  --laps-per-episode 1 \
  --timeout-s 60 \
  --output-dir src/mpcc_pp/mpcc_pp_optuna_runs_smoke \
  --command "ros2 launch drone_control sim.launch.py \
    controller_type:=mpcc_pp rviz:=false plotjuggler:=false"
```

Full tuning run (paper budget):

```bash
ros2 run mpcc_pp mpcc_pp_optuna_tune \
  --n-trials 600 \
  --laps-per-episode 3 \
  --timeout-s 120 \
  --output-dir src/mpcc_pp/mpcc_pp_optuna_runs \
  --command "ros2 run mpcc_pp mpcc_pp_gazebo_trial \
    --px4-model gz_zephyros \
    --startup-delay-s 1.0 \
    --ros-command 'ros2 launch drone_control sim.launch.py \
      controller_type:=mpcc_pp rviz:=false plotjuggler:=false'"
```

Evaluate best result:

```bash
ros2 run mpcc_pp mpcc_pp_optuna_tune \
  --eval-only src/mpcc_pp/mpcc_pp_optuna_runs/best_parameters.yaml \
  --eval-episodes 10 \
  --laps-per-episode 3
```

Using explicit placeholder substitution:

```bash
ros2 run mpcc_pp mpcc_pp_optuna_tune \
  --command "python3 my_eval.py --params {parameter_file} --metrics {metrics_file}"
```

Available placeholders:

```text
{parameter_file}   path to trial parameters.yaml
{trial_dir}        path to trial output directory
{metrics_file}     path where evaluator should write metrics JSON
```

Expected final output:

```text
Best reward: -5.23
Mean lap time: 5.21 s
Lap time std: 0.06 s
Success rate: 100.0 %
Solver failure rate: 0.0 %
Minimum tunnel margin: 0.08 m
Best params written to: src/mpcc_pp/mpcc_pp_optuna_runs/best_parameters.yaml
```

---

## 25. Validation Protocol

After training, evaluate the best parameter vector with paper-style protocol:

1. select the vector with the highest training reward,
2. run 10 test episodes,
3. each episode has 3 laps (30 total test laps),
4. report:

```text
mean lap time
lap time standard deviation
success rate
solver failure rate
minimum tunnel margin across all episodes
number of crashes
number of timeouts
```

A tuning result is not usable unless:

```text
success rate = 100%
solver failure rate is close to 0
minimum tunnel margin is acceptable (>= 0)
mean lap time improves over manual baseline
```

---

## 26. Implementation Order

Follow this order. Do not skip ahead to TuRBO-8.

```text
[ ] Add solver status return and solve counters in drone_opt.py.
[ ] Add lap timestamp tracking in drone_mpcc_pp_controller.py.
[ ] Add reset_episode_stats().
[ ] Add runtime weight parameters to ACADOS stage params (NP = 21).
[ ] Remove tuning weights from the ACADOS build hash.
[ ] Add set_tuning_parameters(params).
[ ] Add _append_weight_params to the controller.
[ ] Verify that ACADOS does not recompile on weight change.
[ ] Implement SearchSpace.decode().
[ ] Implement compute_reward().
[ ] Implement process-based EpisodeRunner.
[ ] Implement JSONL/CSV trial storage.
[ ] Run 10-20 Sobol candidates end-to-end.
[ ] Implement Optuna TPE loop.
[ ] Run 50-100 Optuna trials.
[ ] Export best_parameters.yaml and validate with 10 episodes / 30 laps.
[ ] Implement TuRBO-1 with GP + UCB + trust region.
[ ] Run 50-100 TuRBO-1 candidates and compare to Optuna baseline.
[ ] Add checkpoint and resume for TuRBO state.
[ ] Only after all of the above: implement TuRBO-8.
```

After TuRBO is working:

```text
[ ] Add domain randomization (mass, inertia, thrust, drag, delay, noise).
[ ] Upgrade to paper-faithful Qc_h/Qc_v/Qw_xy/Qw_z split.
[ ] Add ROS service for live parameter injection (removes process restart).
```

---

## 27. Common Pitfalls

### ACADOS Rebuilds on Every Trial

Symptom: `c_generated_code` is regenerated for every BO candidate.

Cause: tunable weights are still embedded as compile-time constants, or they are still
included in the config hash.

Fix: move weights to runtime stage parameters and remove them from the build hash.
Verify by running two episodes with different `ql` values and checking whether ACADOS
rebuilds.

### TuRBO Learns Crashes

Symptom: the best reward comes from trajectories that crash through gates or violate
the tunnel.

Cause: the reward does not penalize crashes enough, or early termination on unsafe states
is not implemented.

Fix: assign a fixed low reward to all failed episodes. Add tunnel violation early
stopping. Consider adding a small tunnel safety penalty once the pipeline is stable.

### All Rewards Are Identical

Symptom: TuRBO or Optuna sees no useful variation and cannot learn.

Possible causes:
- every episode times out (lap detection broken),
- every episode crashes immediately (search space bounds too wide),
- parameter values are not reaching the controller (env var not set),
- solver always falls back to hover (warm-start state not reset).

Fix: run 5 manually chosen parameter sets at known good and known bad values. Inspect
`lap_count`, `lap_times`, `last_status`, and actual rotor commands.

### Solver Instability at Extreme Weights

Symptom: ACADOS returns frequent nonzero statuses for some candidates.

Fix:
- narrow search space bounds,
- increase `track_weight` lower bound if tunnel violations dominate,
- reduce `mu` upper bound if progress reward is too aggressive,
- increase `R_df` lower bound if rotor commands saturate.

### `qc_gate` Falls Below `qc_nom`

Symptom: drone tracks loosely near gates and crashes gate structures.

Fix: always derive `qc_gate = qc_nom * qc_gate_ratio` with `qc_gate_ratio >= 1`.

### Parallel Workers Interfere

Symptom: workers receive each other's ROS topics or MAVLink messages.

Fix: assign separate `ROS_DOMAIN_ID` and separate PX4/MAVLink port pairs to each
worker. Verify isolation before running parallel trials.

---

## 28. What to Improve Next

The pipeline is ready for automatic tuning. Natural next improvements:

- add a ROS evaluator node that publishes metrics JSON instead of relying on log parsing,
- add crash detection from geometry or sudden velocity change,
- measure maximum tunnel violation per episode,
- save reduced bag or telemetry for the top-N trials,
- upgrade from Optuna TPE to TuRBO/BoTorch when the episode loop is stable,
- implement horizontal/vertical contouring and angular-rate split for paper-faithful
  tuning space.

The chosen design is intentional: reliable pipeline first, ambitious search space second.
