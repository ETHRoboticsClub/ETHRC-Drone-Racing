# Guide: TuRBO Bayesian Optimization Tuning for MPCC++

This guide describes how to add automatic hyperparameter tuning to the
`mpcc_pp` controller using Trust Region Bayesian Optimization (TuRBO), following
the methodology used in the MPCC++ paper.

The goal is not only to run Bayesian Optimization, but to build a repeatable
experimental pipeline that can:

- evaluate one MPCC++ parameter vector in simulation,
- measure lap time, solver failures, and safety,
- compute the same kind of sparse episode reward used in the paper,
- feed the result into TuRBO,
- resume interrupted tuning runs,
- export the best parameters back into the controller configuration.

The current repository already contains most of the MPCC++ controller logic:

- `mpcc_pp/drone_model.py`: augmented quadrotor model with position, attitude,
  velocity, body rates, rotor thrust states, path progress, and progress speed.
- `mpcc_pp/drone_opt.py`: ACADOS OCP solver with contouring cost, lag cost,
  progress reward, rotor-rate penalties, body-rate penalties, and soft tunnel
  barrier.
- `mpcc_pp/drone_mpcc_pp_controller.py`: controller wrapper used by the ROS 2
  control node.
- `mpcc_pp/path_utils.py`: arc-length path, dynamic contouring weight, gated
  tunnel geometry, tunnel half-width/half-height computation.
- `drone_control/config/gates_config.yaml`: track definitions and current manual
  MPCC++ parameters.

The missing part is a tuning harness. This guide explains how to build it.

---

## 1. What the Paper Does

The MPCC++ paper treats the controller as a black-box function. For a given
parameter vector `phi`, it runs an episode, collects the resulting trajectory,
and computes one scalar reward.

In the paper, one tuning episode is:

```text
M = 3 consecutive laps around the track
```

The reward is:

```text
R(phi) = - mean(lap_times) - gamma * solver_failure_rate
```

with:

```text
gamma = 100
```

where:

- `lap_times` are the times for the completed laps in the episode,
- `solver_failure_rate` is the fraction of controller steps where the real-time
  solver did not return an acceptable status,
- failed/crashed episodes receive a very low reward.

The optimizer maximizes `R(phi)`. Since lap time is negated, faster laps produce
a higher reward. Since solver failures are penalized, TuRBO learns to avoid
aggressive parameter regions that may be fast but numerically unstable.

The paper uses:

- Gaussian Process surrogate models,
- UCB acquisition:

```text
alpha_UCB(phi) = mu(phi) + beta * sigma(phi)
```

- TuRBO, which runs local Bayesian Optimization inside trust regions,
- an eightfold setup called TuRBO-8 for parallel evaluations.

For this repository, start with TuRBO-1. Once the full single-worker pipeline is
correct, extend to TuRBO-8.

---

## 2. Why the Current Code Needs a Small Refactor First

The current `DroneOptimizerMpccPp` builds an ACADOS solver with cost weights
embedded directly into the symbolic cost expression:

```python
cost_lag      = w['ql']    * el**2
cost_contour  = qc         * ca.dot(ec, ec)
cost_omega    = w['Qw']    * ca.dot(w_k, w_k)
cost_dvt      = w['r_dvt'] * dvt_k**2
cost_df       = w['R_df']  * ca.dot(df_k, df_k)
cost_progress = -w['mu']   * vt_k
cost_track    = w['track_weight'] * (...)
```

This is fine for manual tuning, but bad for Bayesian Optimization.

The optimizer constructor also includes the sorted weights in the ACADOS build
hash:

```python
config_str = (... f"{sorted(w.items())}:" ...)
```

That means every time TuRBO proposes a new parameter vector, ACADOS may need to
regenerate and rebuild the solver. A tuning run with hundreds of evaluations
would become painfully slow.

The first required engineering step is:

```text
Move all tunable controller parameters from compile-time constants to runtime
ACADOS stage parameters.
```

This lets TuRBO evaluate many candidates using one compiled solver.

---

## 3. Recommended Tunable Parameter Set

There are two sensible tuning spaces.

### Option A: Fast Integration Tuning Space

This option matches the current code with minimal changes.

Tune these 8 parameters:

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

Meaning:

- `ql`: lag error weight. Higher values force the drone not to fall behind or
  overshoot along the path direction.
- `qc_nom`: contouring weight away from gates. Lower values allow wider
  shortcuts inside the tunnel.
- `qc_gate`: contouring weight near gates. Higher values make the drone pass
  closer to the reference path/gate centers.
- `Qw`: body-rate regularization. Higher values make angular motion smoother
  but may reduce agility.
- `r_dvt`: progress acceleration penalty. Higher values reduce noisy progress
  speed updates.
- `R_df`: rotor thrust-rate penalty. Higher values make rotor thrust commands
  smoother but less aggressive.
- `mu`: progress reward. Higher values push the controller to fly faster.
- `track_weight`: soft tunnel barrier weight. Higher values enforce the tunnel
  more strongly.

This is the recommended first version because it can be implemented quickly and
fits the current `path_utils.py` design, where `qc` is computed dynamically from
`qc_nom` and `qc_gate`.

### Option B: Paper-Faithful Tuning Space

The paper describes MPCC++ as having 8 tunable parameters:

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

In this repository, implementing this exactly requires splitting the current
scalar contouring and angular-rate penalties.

Instead of:

```python
cost_contour = qc * ca.dot(ec, ec)
cost_omega   = Qw * ca.dot(w_k, w_k)
```

use local path-frame contouring components:

```python
e_n = ca.dot(n_ref, e)
e_b = ca.dot(b_ref, e)

cost_contour = qc_h * e_n**2 + qc_v * e_b**2
```

and split angular-rate regularization:

```python
cost_omega = Qw_xy * (w_k[0]**2 + w_k[1]**2) + Qw_z * w_k[2]**2
```

This is more faithful to the paper, but it changes the OCP cost structure more
deeply. Build the minimal version first, then upgrade.

---

## 4. Search Space Design

Bayesian Optimization works better when every variable is normalized to the same
range. TuRBO normally operates on:

```text
[0, 1]^d
```

However, controller weights often vary over orders of magnitude. Therefore,
store each BO parameter as a normalized value `z` in `[0, 1]`, then map it to a
physical parameter with log scaling:

```python
physical = 10 ** (log10(low) + z * (log10(high) - log10(low)))
```

Recommended initial ranges for the fast integration tuning space:

```text
ql:            1e-2  .. 1e2
qc_nom:        1e-3  .. 1e1
qc_gate_ratio: 1e0   .. 5e1
Qw:            1e-7  .. 1e-2
r_dvt:         1e-7  .. 1e-1
R_df:          1e-7  .. 1e-1
mu:            1e-2  .. 1e2
track_weight:  1e0   .. 1e3
```

Use `qc_gate_ratio` instead of tuning `qc_gate` independently:

```python
qc_gate = qc_nom * qc_gate_ratio
```

This guarantees:

```text
qc_gate >= qc_nom
```

which is desirable because gate regions should not be tracked less strictly than
open track sections.

Recommended `search_space.py` API:

```python
from dataclasses import dataclass


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

    def decode(self, z):
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

## 5. Runtime Parameter Refactor in `drone_opt.py`

The current ACADOS stage parameter vector has 15 fields:

```text
[0:3]   p_ref
[3:6]   t_ref
[6]     qc
[7:10]  n_ref
[10:13] b_ref
[13]    W
[14]    H
```

Extend it to include the tunable weights:

```text
[15] ql
[16] Qw
[17] r_dvt
[18] R_df
[19] mu
[20] track_weight
```

If using the fast integration tuning space, `qc_nom` and `qc_gate` are not ACADOS
parameters directly. They are used in `path_utils.py` to compute stage-wise `qc`,
which remains field `[6]`.

So the new parameter count becomes:

```python
NP = 21
```

In `DroneOptimizerMpccPp.__init__`, add symbols:

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

Then replace:

```python
cost_lag      = w['ql']    * el**2
cost_omega    = w['Qw']    * ca.dot(w_k, w_k)
cost_dvt      = w['r_dvt'] * dvt_k**2
cost_df       = w['R_df']  * ca.dot(df_k, df_k)
cost_progress = -w['mu']   * vt_k
cost_track    = w['track_weight'] * (...)
```

with:

```python
cost_lag      = ql_p * el**2
cost_omega    = Qw_p * ca.dot(w_k, w_k)
cost_dvt      = r_dvt_p * dvt_k**2
cost_df       = R_df_p * ca.dot(df_k, df_k)
cost_progress = -mu_p * vt_k
cost_track    = track_weight_p * (...)
```

Also update the terminal cost:

```python
terminal_cost = (
    ql_p * el**2
    + qc * ca.dot(ec, ec)
    + Qw_p * ca.dot(w_k, w_k)
    - mu_p * vt_k
    + cost_track
)
```

Important: remove weights from the solver build hash. The build hash should only
include items that change the generated solver code, such as:

- horizon length,
- number of nodes,
- model hash,
- constraints,
- number of parameters,
- barrier slope,
- solver options.

It should not include the current tuning values.

---

## 6. Runtime Parameter Refactor in `path_utils.py`

`path_utils.py` returns one stage parameter row per shooting node.

Currently `get_tunnel_params(...)` returns shape:

```text
(N + 1, 15)
```

After adding runtime weights, return:

```text
(N + 1, 21)
```

The cleanest design is not to make `path_utils.py` know about tuning weights.
Keep `path_utils.py` responsible for geometry only, then append weights in the
controller.

For example, in `DroneMpccPpController._run_step(...)`:

```python
stage_params = self._path.get_tunnel_params(...)
stage_params = self._append_weight_params(stage_params)
```

Add:

```python
def _append_weight_params(self, stage_params: np.ndarray) -> np.ndarray:
    weights = self._runtime_weights
    extra = np.array([
        weights["ql"],
        weights["Qw"],
        weights["r_dvt"],
        weights["R_df"],
        weights["mu"],
        weights["track_weight"],
    ], dtype=np.float64)
    repeated = np.repeat(extra.reshape(1, -1), stage_params.shape[0], axis=0)
    return np.hstack([stage_params, repeated])
```

This keeps path geometry and tuning parameters separated.

---

## 7. Add a Runtime Tuning API to the Controller

The controller needs a way for the tuning harness to set a new parameter vector
without constructing a brand-new Python object or rebuilding ACADOS.

Add to `DroneMpccPpController`:

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

Initialize `_runtime_weights` in `__init__`:

```python
self._runtime_weights = {
    "ql": 7.0,
    "Qw": 0.0001,
    "r_dvt": 0.0002,
    "R_df": 0.0003,
    "mu": 7.0,
    "track_weight": tp.get("track_weight", 30.0),
}

if weights:
    self._runtime_weights.update(weights)
```

The older `weights` argument can remain for backwards compatibility, but after
the refactor it should no longer be used to generate compile-time cost weights.

---

## 8. Expose Solver Status and Timing

Bayesian Optimization needs solver failure rate. The current `solve()` method
prints the status but returns only:

```python
return u_pred[0].copy(), x_pred, u_pred
```

Change it to return an info dictionary:

```python
status = self.solver.solve()

info = {
    "status": int(status),
    "success": status in (0, 2),
}

try:
    info["time_tot"] = float(self.solver.get_stats("time_tot"))
except Exception:
    info["time_tot"] = None

return u_pred[0].copy(), x_pred, info
```

If other code still needs `u_pred`, include it inside `info`:

```python
info["u_pred"] = u_pred
```

Then in `DroneMpccPpController._run_step(...)`, accumulate:

```python
self._solve_count += 1
if not info["success"]:
    self._solver_fail_count += 1
```

Expose:

```python
def get_solver_stats(self) -> dict:
    if self._solve_count == 0:
        fail_rate = 0.0
    else:
        fail_rate = self._solver_fail_count / self._solve_count
    return {
        "solve_count": self._solve_count,
        "solver_fail_count": self._solver_fail_count,
        "solver_fail_rate": fail_rate,
    }
```

Also add a reset method:

```python
def reset_episode_stats(self) -> None:
    self._solve_count = 0
    self._solver_fail_count = 0
    self._lap = 0
    self._theta = 0.0
    self._theta_solve = 0.0
    self._vtheta = 0.0
    self._f_state = self._model.f_hover.copy()
    if self._opt is not None:
        self._opt._warm = False
```

---

## 9. Define Episode Success, Failure, and Reward

The tuning harness should classify every episode.

### Success

An episode succeeds if:

```text
lap_count >= laps_per_episode
```

Recommended default:

```text
laps_per_episode = 3
```

### Failure

An episode fails if any of these occur:

- timeout before completing the required laps,
- position or velocity becomes NaN or infinite,
- solver failure rate exceeds a hard threshold,
- vehicle leaves a safety bounding box,
- vehicle violates the tunnel by more than an allowed margin,
- simulator process dies,
- ROS topics stop updating.

Recommended initial thresholds:

```text
episode_timeout_s = 90
max_solver_fail_rate = 0.30
max_tunnel_violation_m = 0.25
max_position_norm_m = 200
max_speed_m_s = 40
```

Use lenient thresholds initially so that TuRBO receives useful reward variation.
Once tuning works, tighten the thresholds.

### Reward

Use the paper reward:

```python
reward = -np.mean(lap_times) - 100.0 * solver_fail_rate
```

For failed episodes:

```python
reward = failed_episode_reward
```

Recommended:

```text
failed_episode_reward = -1e3
```

For timeout episodes that still completed at least one lap, an alternative is:

```python
reward = -timeout_s - 100.0 * solver_fail_rate - 10.0 * missing_laps
```

But for the first version, keep it simple and assign a fixed bad reward to any
failed episode.

---

## 10. Tunnel Safety Metric

The paper emphasizes safety via the tunnel constraint. Therefore, log the actual
tunnel margin during each episode.

For current vehicle position `p`, path reference `p_ref`, normal `n_ref`,
binormal `b_ref`, half-width `W`, and half-height `H`:

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
margin >= 0: inside tunnel
margin < 0: outside tunnel by abs(margin) meters
```

Log:

```text
min_tunnel_margin
mean_tunnel_margin
max_tunnel_violation = max(0, -min_tunnel_margin)
```

Do not necessarily put this in the reward at first. The paper reward uses lap
time and solver failures. The tunnel is already in the controller cost. However,
using tunnel metrics for early termination and diagnostics is very useful.

Later, if TuRBO finds policies that cut too close to the tunnel boundary, add a
small safety penalty:

```python
reward -= safety_weight * max(0.0, -min_tunnel_margin)
```

Start with:

```text
safety_weight = 20
```

Only add this if needed.

---

## 11. Proposed File Layout

Add a tuning module inside `src/mpcc_pp/mpcc_pp`:

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

Add a console script in `src/mpcc_pp/setup.py`:

```python
entry_points={
    "console_scripts": [
        "mpcc_pp_turbo_tune = mpcc_pp.tuning.cli:main",
    ],
}
```

Then the target command becomes:

```bash
ros2 run mpcc_pp mpcc_pp_turbo_tune \
  --track src/drone_control/config/gates_config.yaml \
  --controller mpcc_pp \
  --episodes 600 \
  --laps-per-episode 3 \
  --out runs/mpcc_pp_tuning
```

---

## 12. `reward.py`

Keep reward computation isolated and testable.

Recommended API:

```python
from dataclasses import dataclass
from typing import Sequence
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
    if not result.success:
        return failed_episode_reward
    if not result.lap_times:
        return failed_episode_reward
    return -float(np.mean(result.lap_times)) - gamma * float(result.solver_fail_rate)
```

Unit tests should cover:

- successful episode with three lap times,
- failed episode,
- empty lap list,
- nonzero solver failure rate.

---

## 13. `storage.py`

Tuning runs are expensive. Every trial must be saved immediately.

Use a run directory:

```text
runs/mpcc_pp_tuning/2026-04-25_15-30-12/
  config.yaml
  trials.csv
  trials.jsonl
  best_params.yaml
  checkpoints/
    turbo_state.pt
  episodes/
    trial_000012_rep_00.npz
```

For each trial, save one JSON line:

```json
{
  "trial": 12,
  "seed": 1234,
  "z": [0.1, 0.7, 0.4],
  "params": {
    "ql": 3.1,
    "qc_nom": 0.2,
    "qc_gate": 12.5
  },
  "reward": -5.31,
  "success": true,
  "lap_times": [5.28, 5.35, 5.30],
  "solver_fail_rate": 0.0,
  "min_tunnel_margin": 0.11,
  "reason": "completed"
}
```

Also write `trials.csv` for quick plotting.

Update `best_params.yaml` whenever a new best reward appears:

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

This file should be directly portable back into
`drone_control/config/gates_config.yaml`.

---

## 14. `episode_runner.py`: Two Implementation Paths

There are two ways to evaluate an episode.

### Path A: ROS/SITL Process Runner

This is closest to the real stack.

The runner:

1. starts the simulator,
2. starts the controller,
3. waits until the drone is armed and receiving state,
4. applies the parameter vector,
5. monitors lap count and metrics,
6. stops everything,
7. returns an `EpisodeResult`.

This path is realistic but slower and more fragile.

Pseudo-code:

```python
class EpisodeRunner:
    def evaluate(self, params: dict, seed: int) -> EpisodeResult:
        self._write_trial_config(params, seed)
        self._start_simulation()
        self._wait_for_topics()
        self._wait_for_race_start()

        while not done:
            state = self._latest_state()
            self._update_metrics(state)

            if self._lap_count() >= self.laps_per_episode:
                return self._success_result()
            if self._timed_out():
                return self._failure_result("timeout")
            if self._unsafe_state(state):
                return self._failure_result("unsafe_state")

        return self._failure_result("unknown")
```

### Path B: In-Process Dynamics Runner

This bypasses ROS and uses `DroneMpccPpModel` plus the optimizer directly.

The runner:

1. creates `DroneMpccPpController`,
2. calls `initialize()`,
3. simulates dynamics in Python using the ACADOS/CasADi model or a simple
   integrator,
4. feeds the simulated state back into `controller_update`,
5. measures lap count.

This path is faster and easier to parallelize, but it may not reflect PX4,
MAVSDK, offboard command delay, or Gazebo dynamics.

Recommended strategy:

```text
Use Path A for final tuning and validation.
Use Path B only if simulation throughput becomes the bottleneck.
```

---

## 15. Passing Parameters to the ROS Controller

For process-based tuning, the tuning script needs to pass a different parameter
vector to the controller for every episode.

There are three good options.

### Option 1: Temporary YAML File

For each trial:

1. copy `gates_config.yaml` into the run directory,
2. overwrite the active track's `track` section with the candidate parameters,
3. launch:

```bash
ros2 launch drone_control sim.launch.py \
  controller_type:=mpcc_pp \
  waypoints_file:=/path/to/run/trial_000012.yaml \
  rviz:=false \
  plotjuggler:=false
```

This is simple and robust, but it restarts the controller process every trial.

### Option 2: ROS Service

Add a service to `drone_controller_node`:

```text
/mpcc_pp/set_tuning_parameters
```

The service receives parameter names and values, then calls:

```python
self.controller.set_tuning_parameters(params)
self.controller.reset_episode_stats()
```

This allows one long-lived controller process.

This is the best long-term architecture.

### Option 3: ROS Parameters

Declare ROS parameters:

```text
mpcc_pp.ql
mpcc_pp.qc_nom
...
```

Then use dynamic parameter updates.

This is idiomatic ROS, but nested dictionaries and type handling can become
annoying. A dedicated service is usually clearer.

Recommended sequence:

```text
Start with temporary YAML files.
Move to a ROS service once the evaluator and reward logic are correct.
```

---

## 16. Lap Time Measurement

The controller already tracks `lap_count` internally. For tuning, also record
timestamps when a lap completes.

Add to `DroneMpccPpController`:

```python
self._lap_timestamps = []
```

When lap count increments:

```python
now = time.monotonic()
self._lap_timestamps.append(now)
```

Expose:

```python
def get_lap_times(self) -> list[float]:
    if len(self._lap_timestamps) < 2:
        return []
    return [
        self._lap_timestamps[i] - self._lap_timestamps[i - 1]
        for i in range(1, len(self._lap_timestamps))
    ]
```

Be careful with the lead-in. The first segment from initial hover/start position
to gate 1 should not count as a lap. A good convention is:

- record `race_start_time` when the controller first reaches `circuit_start_theta`
  or completes the lead-in,
- record lap times only after that.

Simpler first version:

- ignore the lead-in,
- start episode timing after the first lap wrap,
- require `laps_per_episode + 1` wraps internally,
- compute lap times from the last `laps_per_episode` intervals.

---

## 17. TuRBO Implementation

Use BoTorch and GPyTorch.

Dependencies:

```text
torch
botorch
gpytorch
```

Add them as optional tuning dependencies:

```python
extras_require={
    "test": ["pytest"],
    "tuning": ["torch", "botorch", "gpytorch"],
}
```

The TuRBO loop structure:

```python
space = SearchSpace()
runner = EpisodeRunner(...)
turbo = TurboOptimizer(dim=space.dim, ...)

for trial_idx in range(total_episodes):
    z = turbo.ask()
    params = space.decode(z)
    result = runner.evaluate(params, seed=base_seed + trial_idx)
    reward = compute_reward(result)
    turbo.tell(z, reward)
    storage.append_trial(...)
```

### Initial Sobol Samples

Before fitting a GP, collect initial random low-discrepancy samples.

Recommended:

```text
n_init = 2 * dim
```

For an 8D search space:

```text
n_init = 16
```

Use:

```python
from torch.quasirandom import SobolEngine

sobol = SobolEngine(dimension=dim, scramble=True, seed=seed)
initial_points = sobol.draw(n_init)
```

### GP Model

For a simple first implementation:

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

Normalize rewards before fitting if needed:

```python
Y_norm = (Y - Y.mean()) / (Y.std() + 1e-8)
```

### UCB Acquisition

Use Upper Confidence Bound:

```python
from botorch.acquisition import UpperConfidenceBound

acq = UpperConfidenceBound(model, beta=2.0)
```

Optimize acquisition inside the current trust region.

---

## 18. Trust Region Logic

TuRBO keeps a local trust region around the best point found by a local BO
instance.

State:

```python
length = 0.8
length_min = 0.5 ** 7
length_max = 1.6
success_counter = 0
failure_counter = 0
success_tolerance = 3
failure_tolerance = ceil(max(4 / batch_size, dim / batch_size))
```

After each evaluation:

- if the new reward improves the best reward:
  - increment `success_counter`,
  - reset `failure_counter`.
- otherwise:
  - increment `failure_counter`,
  - reset `success_counter`.

If:

```text
success_counter == success_tolerance
```

expand:

```python
length = min(2.0 * length, length_max)
```

If:

```text
failure_counter == failure_tolerance
```

shrink:

```python
length = length / 2.0
```

If:

```text
length < length_min
```

restart the trust region from a new Sobol point or from a high-performing
previous candidate.

Trust region bounds around best point `x_center`:

```python
lower = torch.clamp(x_center - length / 2.0, 0.0, 1.0)
upper = torch.clamp(x_center + length / 2.0, 0.0, 1.0)
```

Candidate generation:

```python
candidate = optimize_acqf(
    acq_function=acq,
    bounds=torch.stack([lower, upper]),
    q=1,
    num_restarts=10,
    raw_samples=256,
)
```

---

## 19. From TuRBO-1 to TuRBO-8

Do not start with TuRBO-8. It multiplies infrastructure problems.

Build in this order:

1. single candidate evaluation works,
2. random/Sobol search over 5-10 episodes works,
3. TuRBO-1 works for 30-50 episodes,
4. logs and resume work,
5. final validation works,
6. only then add TuRBO-8.

For TuRBO-8, each worker needs isolated simulation resources:

```text
ROS_DOMAIN_ID
PX4 instance ID
MAVLink UDP ports
Gazebo world/model namespace
run directory
random seed
```

Example worker allocation:

```text
worker 0: ROS_DOMAIN_ID=30, PX4_SYS_AUTOSTART instance 0, MAVLink 14540
worker 1: ROS_DOMAIN_ID=31, PX4_SYS_AUTOSTART instance 1, MAVLink 14541
...
worker 7: ROS_DOMAIN_ID=37, PX4_SYS_AUTOSTART instance 7, MAVLink 14547
```

If PX4/Gazebo multi-instance is not reliable yet, run TuRBO-8 as eight logical
trust regions but evaluate candidates sequentially. This still gives better
global exploration than plain BO, just without wall-clock speedup.

---

## 20. Domain Randomization

The paper also tests MPCC++ with domain randomization.

For each BO iteration:

```text
run 10 different episodes with 10 different noise realizations
average the reward
update BO once with the average reward
```

This reduces the number of BO updates but improves robustness.

Recommended randomization variables:

```text
mass scale:          0.90 .. 1.10
inertia scale:       0.85 .. 1.15
max thrust scale:    0.90 .. 1.10
drag coefficient:    small positive range
state noise:         position/orientation/velocity measurement noise
command delay:       0 ms .. 40 ms
wind disturbance:    optional later
```

Do not start with domain randomization. First tune nominal simulation.

Suggested progression:

```text
Phase 1: nominal tuning
Phase 2: validate best policy with randomized rollouts
Phase 3: randomization-aware tuning
```

---

## 21. Final Evaluation Protocol

After training:

1. select the parameter vector with the highest training reward,
2. run 10 test episodes,
3. each test episode has 3 laps,
4. report:

```text
mean lap time
lap time standard deviation
success rate
solver failure rate
minimum tunnel margin
number of crashes
number of timeouts
```

This matches the paper's evaluation style:

```text
10 episodes = 30 test laps
```

A tuning result should not be considered usable unless:

```text
success rate = 100%
solver failure rate is close to 0
minimum tunnel margin is acceptable
lap time improves over manual baseline
```

---

## 22. Integration Back Into `gates_config.yaml`

Once tuning is complete, copy the best values into the active track:

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

Then update `DroneMpccPpController.__init__` to read these optional keys:

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

This keeps manual and tuned workflows unified.

---

## 23. Development Milestones

### Milestone 1: Documentation and Baseline Logs

Goal:

```text
Run current MPCC++ and record baseline lap times and solver failures.
```

Tasks:

- add solver status return,
- add lap time tracking,
- add episode metrics,
- run one manual baseline,
- save metrics to JSON/CSV.

Success criteria:

```text
One episode produces lap_times, solver_fail_rate, success, and reward.
```

### Milestone 2: Runtime Weight Parameters

Goal:

```text
Change MPCC++ weights without rebuilding ACADOS.
```

Tasks:

- extend ACADOS parameter vector,
- append weight parameters to stage params,
- remove weights from build hash,
- add `set_tuning_parameters`,
- verify two different parameter sets reuse the same solver.

Success criteria:

```text
Changing ql/mu/track_weight changes behavior but does not rebuild ACADOS.
```

### Milestone 3: Sobol Random Search

Goal:

```text
Evaluate random candidates end-to-end before adding TuRBO.
```

Tasks:

- implement `SearchSpace`,
- implement `EpisodeRunner`,
- implement storage,
- run 10-20 Sobol candidates.

Success criteria:

```text
trials.jsonl and best_params.yaml are produced.
```

### Milestone 4: TuRBO-1

Goal:

```text
Replace random search with trust-region BO.
```

Tasks:

- implement GP fitting,
- implement UCB acquisition,
- implement trust region updates,
- checkpoint optimizer state,
- resume interrupted runs.

Success criteria:

```text
TuRBO-1 improves reward over the initial Sobol candidates.
```

### Milestone 5: Final Test Protocol

Goal:

```text
Evaluate the best policy with paper-style 10 episodes / 30 laps.
```

Tasks:

- add `mpcc_pp_evaluate_best` CLI or `--eval-only`,
- run best params,
- produce summary table.

Success criteria:

```text
The final report gives lap time mean/std, success rate, and safety stats.
```

### Milestone 6: TuRBO-8

Goal:

```text
Parallelize evaluations.
```

Tasks:

- isolate ROS domains,
- isolate simulator instances,
- run multiple workers,
- coordinate TuRBO trust regions.

Success criteria:

```text
Eight candidate evaluations can run without topic, port, or simulator conflicts.
```

---

## 24. Common Pitfalls

### ACADOS Rebuilds Every Trial

Symptom:

```text
Every BO candidate regenerates c_generated_code.
```

Cause:

```text
Tunable weights are still compile-time constants or included in the config hash.
```

Fix:

```text
Move weights to runtime stage parameters and remove them from the hash.
```

### TuRBO Learns Crashes

Symptom:

```text
Best reward comes from unsafe trajectories.
```

Cause:

```text
Reward does not penalize crashes/tunnel violations enough, or failure detection
is too weak.
```

Fix:

```text
Assign fixed low reward to failed episodes. Add tunnel violation early stopping.
```

### All Rewards Are Identical

Symptom:

```text
TuRBO sees no useful variation.
```

Possible causes:

- every episode times out,
- every episode crashes immediately,
- lap detection is broken,
- parameter values are not actually reaching the controller,
- solver always falls back to hover.

Fix:

```text
Run 5 manually chosen parameter sets and inspect lap_count, lap_times, solver
status, and command outputs.
```

### Solver Instability at Extreme Weights

Symptom:

```text
ACADOS returns frequent nonzero statuses for some candidates.
```

Fix:

- narrow search bounds,
- increase `track_weight` lower bound if tunnel violations dominate,
- reduce `mu` upper bound if progress reward is too aggressive,
- increase `R_df` lower bound if rotor commands become too sharp,
- penalize solver failures as in the paper.

### `qc_gate` Becomes Lower Than `qc_nom`

Symptom:

```text
Drone tracks loosely at gates and crashes.
```

Fix:

```text
Tune qc_gate_ratio and compute qc_gate = qc_nom * qc_gate_ratio.
```

### Parallel Workers Interfere

Symptom:

```text
Workers receive each other's ROS topics or MAVLink messages.
```

Fix:

```text
Use separate ROS_DOMAIN_ID values and separate PX4/MAVLink ports.
```

---

## 25. Minimal First Implementation Checklist

Start with this exact checklist.

```text
[ ] Add solver status and solve counters.
[ ] Add lap timestamp tracking.
[ ] Add `reset_episode_stats()`.
[ ] Add runtime weight parameters to ACADOS stage params.
[ ] Remove weights from the ACADOS build hash.
[ ] Add `set_tuning_parameters(params)`.
[ ] Implement `SearchSpace.decode()`.
[ ] Implement `compute_reward()`.
[ ] Implement simple process-based `EpisodeRunner`.
[ ] Save every trial to JSONL.
[ ] Run 10 Sobol candidates.
[ ] Implement TuRBO-1.
[ ] Run 50 TuRBO-1 candidates.
[ ] Export `best_params.yaml`.
[ ] Run 10 evaluation episodes with the best params.
```

Only after that:

```text
[ ] Add domain randomization.
[ ] Add TuRBO-8.
[ ] Add service-based persistent controller tuning.
```

---

## 26. Target User Workflow

Build:

```bash
colcon build --packages-select mpcc_pp drone_control
source install/setup.bash
```

Run a short smoke test:

```bash
ros2 run mpcc_pp mpcc_pp_turbo_tune \
  --track src/drone_control/config/gates_config.yaml \
  --controller mpcc_pp \
  --episodes 10 \
  --laps-per-episode 1 \
  --out runs/mpcc_pp_tuning_smoke
```

Run the paper-like tuning budget:

```bash
ros2 run mpcc_pp mpcc_pp_turbo_tune \
  --track src/drone_control/config/gates_config.yaml \
  --controller mpcc_pp \
  --episodes 600 \
  --laps-per-episode 3 \
  --out runs/mpcc_pp_tuning
```

Evaluate the best result:

```bash
ros2 run mpcc_pp mpcc_pp_turbo_tune \
  --track src/drone_control/config/gates_config.yaml \
  --eval-only runs/mpcc_pp_tuning/latest/best_params.yaml \
  --eval-episodes 10 \
  --laps-per-episode 3
```

Expected final output:

```text
Best reward: -5.23
Mean lap time: 5.21 s
Lap time std: 0.06 s
Success rate: 100.0 %
Solver failure rate: 0.0 %
Minimum tunnel margin: 0.08 m
Best params written to: runs/mpcc_pp_tuning/.../best_params.yaml
```

---

## 27. Practical Recommendation

The safest implementation order is:

1. make the current MPCC++ controller measurable,
2. make weights runtime parameters,
3. run Sobol random search,
4. add TuRBO-1,
5. validate the best parameters,
6. add TuRBO-8 only after the single-worker pipeline is boringly reliable.

This mirrors the paper's method while staying compatible with the current
repository architecture.

