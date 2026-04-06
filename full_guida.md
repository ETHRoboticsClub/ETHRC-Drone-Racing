# Full Guide: Gate-Based MPCC with Track Constraint

## Table of Contents
1. [What Was Implemented and Why](#1-what-was-implemented-and-why)
2. [The Track Constraint — Theory](#2-the-track-constraint--theory)
3. [Architecture Overview](#3-architecture-overview)
4. [The Gates Configuration File](#4-the-gates-configuration-file)
5. [GatedRacingPath — Path and Tunnel Geometry](#5-gatedracingpath--path-and-tunnel-geometry)
6. [Track Constraint in the OCP (drone_opt.py)](#6-track-constraint-in-the-ocp-drone_optpy)
7. [Controller Integration (drone_mpcc_controller.py)](#7-controller-integration-drone_mpcc_controllerpy)
8. [RViz Gate Visualization](#8-rviz-gate-visualization)
9. [How to Run](#9-how-to-run)
10. [Tuning Parameters](#10-tuning-parameters)
11. [ACADOS Code Regeneration](#11-acados-code-regeneration)
12. [Backwards Compatibility](#12-backwards-compatibility)

---

## 1. What Was Implemented and Why

### Goal
Replace the MPCC controller's plain waypoint list with a list of **racing gates** that the drone must pass through. The gate geometry is used to enforce a **Track Constraint** inside the MPC optimiser, guaranteeing (as a soft constraint) that the drone stays within the gate aperture when passing through each gate.

### What changed at a glance

| File | Change |
|---|---|
| `config/gates_config.yaml` | **NEW** — competition gate definitions |
| `mpcc/path_utils.py` | **Added** `GatedRacingPath` class |
| `mpcc/drone_opt.py` | **Extended** OCP: NP 7→15, added track-constraint log-barrier cost |
| `mpcc/drone_mpcc_controller.py` | **Updated** to use `GatedRacingPath` and `get_tunnel_params()` |
| `drone_control/drone_controller_node.py` | **Updated** YAML loader to detect gates format |
| `drone_control/waypoint_visualizer.py` | **Updated** to render gate rectangles and arrows |
| `launch/sim.launch.py` | **Updated** default config to `gates_config.yaml` |

---

## 2. The Track Constraint — Theory

Source: Krinner et al., *"MPCC++: Model Predictive Contouring Control for Time-Optimal Flight with Safety Constraints"*, RSS 2024, Section IV-A.

### Why standard MPCC crashes into gates

Baseline MPCC (Romero et al. 2022) models gates only through a **dynamic contouring weight** `qc(θ)` — a Gaussian that increases the tracking penalty near each gate.
This approach has two weaknesses:
1. The drone is *penalised* for deviating from the centre, but never *forbidden* from hitting the gate post.
2. With high progress reward `μ`, the solver may find it better to cut corners even through a gate.

### The prismatic tunnel

MPCC++ introduces a **prismatic tunnel** around the centerline.
At each arc-length `θ`, the cross-section is a rectangle of:
- **total width** `2W(θ)` in the `n(θ)` direction (horizontal)
- **total height** `2H(θ)` in the `b(θ)` direction (vertical)

centred at the reference path position `p^d(θ)`.

The bottom-left corner of the cross-section is:
```
p_0(θ) = p^d(θ) − W(θ)·n(θ) − H(θ)·b(θ)
```

The four halfspace inequalities that keep the drone inside the tunnel:
```
h1 = (p − p_0)·n  ≥ 0           inside left  wall
h2 = 2W − (p − p_0)·n  ≥ 0     inside right wall
h3 = (p − p_0)·b  ≥ 0           inside bottom wall
h4 = 2H − (p − p_0)·b  ≥ 0     inside top   wall
```

Substituting `p_0` and using `e_n = (p − p^d)·n`, `e_b = (p − p^d)·b`:
```
h1 = W + e_n ≥ 0    →   e_n ≥ −W
h2 = W − e_n ≥ 0    →   e_n ≤  W
h3 = H + e_b ≥ 0    →   e_b ≥ −H
h4 = H − e_b ≥ 0    →   e_b ≤  H
```

Combined: the drone must satisfy `|e_n| ≤ W(θ)` and `|e_b| ≤ H(θ)`.

### Soft barrier formulation

Hard constraints break the real-time SQP solver near constraint boundaries.
Following the paper (eq. 14), each halfspace is enforced via a **log-barrier** added to the cost:

```
p(h) = log(1 + exp(−α·h))     α = 5
```

The paper uses α = 100 for sharp enforcement, but in practice α = 100 causes numerical overflow
during cold-start: if the drone starts at z ≈ 0 m and the path is at z = −3 m, then
h₃ = H + e_b = 1 + (−3) = −2, giving exp(−100·(−2)) = exp(200) ≈ 7 × 10⁸⁵. This makes
the QP Hessian ill-conditioned and HPIPM returns `ACADOS_NAN_DETECTED`. α = 5 keeps
gradients O(5), comparable to other cost terms (ql = 10, mu = 10).

The barrier is also implemented in numerically stable softplus form to prevent any exp overflow:

```
p(h) = softplus(−α·h)
     = max(−α·h, 0) + log(1 + exp(−|α·h|))
```

Properties:
- When `h ≫ 0` (inside tunnel): `p(h) ≈ 0` — no cost
- When `h = 0` (boundary): `p(h) = log(2) ≈ 0.69`
- When `h < 0` (violation): `p(h) ≈ −α·h` — linear penalty growing steeply

The total track constraint cost added to each stage:
```
cost_track = track_weight × [p(h1) + p(h2) + p(h3) + p(h4)]
```

### W(θ) and H(θ) — varying tunnel size

The tunnel is **wide between gates** (the drone can choose its own line) and **narrow at each gate** (the drone must pass through the aperture):

```
W(θ) = W_n + Σ_i (W_gate_i − W_n) · exp(−½(θ − θ_gate_i)² / σ²)
```

Since `W_gate_i < W_n`, each term is negative — `W(θ)` decreases near gates.
The result is clamped to `[min(W_gate_i), W_n]`.

### Local frame n(θ), b(θ)

For racing gates that are always vertical (standard competition setup):
```
n(θ) = normalize([−t_y, t_x, 0])   — horizontal right direction (NED)
b(θ) = [0, 0, −1]                  — NED up direction (vertical)
```

This avoids the Frenet-Serret frame singularities, because `n` and `b` are derived from the tangent by simple projection, not from curvature.

**Verification (NED: x=North, y=East, z=Down):**
- Flying North (`t=[1,0,0]`): `n=[0,1,0]` (East = right of North) ✓
- Flying East (`t=[0,1,0]`): `n=[−1,0,0]` (South = right of East) ✓

---

## 3. Architecture Overview

```
gates_config.yaml
      │
      ▼
drone_controller_node._load_track_config()
      │  returns {'gates': [...], 'track': {...}, 'waypoints': array}
      │
      ▼
DroneMpccController.from_config(config)
      │  detects 'gates' key → creates GatedRacingPath
      │
      ▼
GatedRacingPath                           RacingPath (base)
  ├─ arc-length spline (cubic, closed)  ← inherits
  ├─ gate_thetas[] — θ at each gate
  ├─ tunnel_width(θ)  W(θ)
  ├─ tunnel_height(θ) H(θ)
  ├─ normal_binormal(θ) → n, b
  └─ get_tunnel_params(θ, N, dt) → (N, 15)
      │
      ▼ stage_params (N+1, 15)
DroneOptimizerMpcc.solve(x0, stage_params)
      │  NP=15: [p_ref, t_ref, qc, n_ref, b_ref, W, H]
      │  cost += track_weight · Σ log(1+exp(−100·h_i))
      │
      ▼
AttitudeRate command → MAVSDK → PX4
```

---

## 4. The Gates Configuration File

**Path:** `src/drone_control/config/gates_config.yaml`

### Format

```yaml
track:                    # global tunnel parameters
  W_nominal:    3.0       # tunnel half-width between gates [m]
  H_nominal:    3.0       # tunnel half-height between gates [m]
  sigma_gate:   2.0       # Gaussian σ for W/H transitions [m]
  qc_nom:       0.5       # contouring weight on straights
  qc_gate:      10.0      # contouring weight near gates
  sigma_qc:     0.5       # Gaussian σ for qc transition [m]
  track_weight: 10.0      # log-barrier weight in OCP cost

gates:
  - name:     G1          # label shown in RViz
    x:  0.0               # NED position [m] — x=North
    y:  0.0               # NED position [m] — y=East
    z: -3.0               # NED position [m] — z=Down (negative = above ground)
    yaw_deg: -11.0        # flight direction through gate [deg]
                          # 0=North, 90=East, 180=South, -90=West
    width:  2.0           # inner aperture total width  [m] (horizontal)
    height: 2.0           # inner aperture total height [m] (vertical)
```

### How yaw_deg is computed

`yaw_deg` is the direction the drone should be flying when it passes through the gate.
For a smooth track, it should be the average of the incoming and outgoing directions.

For the provided 8-gate track, yaw angles were computed as the complex-number average of the bearing from the previous gate and the bearing to the next gate:

```
θ_gate_i = arg(exp(j·θ_in) + exp(j·θ_out))
```

You can recalculate these with:
```python
import numpy as np
positions = [(x0,y0), (x1,y1), ...]  # NED x,y for each gate
n = len(positions)
for i in range(n):
    prev = positions[(i-1) % n]
    curr = positions[i]
    next = positions[(i+1) % n]
    d_in  = np.arctan2(curr[1]-prev[1], curr[0]-prev[0])
    d_out = np.arctan2(next[1]-curr[1], next[0]-curr[0])
    v = np.exp(1j*d_in) + np.exp(1j*d_out)
    yaw = np.degrees(np.angle(v))
    print(f"G{i+1}: yaw_deg = {yaw:.1f}")
```

### Designing a custom track

1. Place gates in NED coordinates — draw your track on paper first.
2. Compute yaw angles using the formula above.
3. Set `width` and `height` to the inner gate aperture (e.g. 2.0 × 2.0 m).
4. Set `W_nominal` and `H_nominal` large enough that the drone has freedom between gates (3–5 m is typical).
5. `sigma_gate` controls how quickly the tunnel narrows before each gate. Larger values = softer, earlier narrowing.

---

## 5. GatedRacingPath — Path and Tunnel Geometry

**File:** `src/mpcc/path_utils.py`, class `GatedRacingPath(RacingPath)`

### Initialization

```python
path = GatedRacingPath(
    gates,           # list of gate dicts from YAML
    W_nominal=3.0,   # tunnel half-width between gates
    H_nominal=3.0,   # tunnel half-height between gates
    sigma_gate=2.0,  # Gaussian σ for W/H transitions [m]
    sigma_qc=0.5,    # Gaussian σ for qc transitions [m]
)
```

The constructor:
1. Extracts gate centres `positions[i] = (x, y, z)`.
2. Builds a **closed-loop path**: appends `positions[0]` to the end.
3. Computes arc-length knots `arc[i]` — the cumulative distance from gate 0 to gate i.
4. Stores `gate_thetas[i] = arc[i]` — the `θ` value at each gate.
5. Calls `RacingPath.__init__(positions_closed)` to fit cubic splines.

### tunnel_width(θ) and tunnel_height(θ)

```python
W(θ) = W_n + Σ_i (W_gate_i − W_n) · exp(−½·d_i(θ)² / σ²)
```

where `d_i(θ)` is the **shortest periodic distance** between `θ` and `θ_gate_i`:

```python
d = theta - theta_gate_i
d -= total_length * round(d / total_length)
```

This handles the lap wrap-around correctly — gate i is "close" to both `θ_gate_i` and `θ_gate_i ± total_length`.

The result is clamped to `[min_gate_half_width, W_nominal]`.

### normal_binormal(θ)

```python
n(θ) = normalize([−t_y(θ), t_x(θ), 0])   # horizontal right
b(θ) = [0, 0, −1]                          # NED up
```

Fallback for nearly-vertical segments (rare): `n = [1, 0, 0]`.

### get_tunnel_params(θ, n_stages, dt, ...)

Returns a `(n_stages, 15)` array. Each row:

| Indices | Name | Description |
|---|---|---|
| 0:3 | `p_ref` | reference path position `p^d(θ_k)` |
| 3:6 | `t_ref` | unit tangent `t(θ_k)` |
| 6 | `qc` | dynamic contouring weight `qc(θ_k)` |
| 7:10 | `n_ref` | horizontal tunnel normal `n(θ_k)` |
| 10:13 | `b_ref` | vertical tunnel normal `b(θ_k)` |
| 13 | `W` | tunnel half-width `W(θ_k)` |
| 14 | `H` | tunnel half-height `H(θ_k)` |

Predicted `θ_k` values are computed by advancing at `vtheta_guess` m/s, same as the base class.

---

## 6. Track Constraint in the OCP (drone_opt.py)

**File:** `src/mpcc/drone_opt.py`

### Parameter vector expansion

The per-stage parameter vector grew from **7 to 15 elements**:

```
Old: [p_ref(3), t_ref(3), qc(1)]                              NP = 7
New: [p_ref(3), t_ref(3), qc(1), n_ref(3), b_ref(3), W(1), H(1)]  NP = 15
```

In CasADi:
```python
p_ref = ca.MX.sym('p_ref', 3)   # reference position    [0:3]
t_ref = ca.MX.sym('t_ref', 3)   # unit tangent          [3:6]
qc    = ca.MX.sym('qc',    1)   # contouring weight     [6]
n_ref = ca.MX.sym('n_ref', 3)   # horiz tunnel normal   [7:10]
b_ref = ca.MX.sym('b_ref', 3)   # vert  tunnel normal   [10:13]
W     = ca.MX.sym('W',     1)   # tunnel half-width     [13]
H     = ca.MX.sym('H',     1)   # tunnel half-height    [14]
params = ca.vertcat(p_ref, t_ref, qc, n_ref, b_ref, W, H)
```

### Track constraint cost term

```python
# Position error projected onto gate frame
e   = p_k - p_ref          # total position error
e_n = ca.dot(n_ref, e)     # error in gate-width  direction
e_b = ca.dot(b_ref, e)     # error in gate-height direction

# 4 halfspace margins (≥ 0 inside tunnel)
h1 = W + e_n    # inside left  wall
h2 = W - e_n    # inside right wall
h3 = H + e_b    # inside bottom wall
h4 = H - e_b    # inside top   wall

# Log-barrier with α=5 (numerically stable softplus form)
alpha = 5.0

def _barrier(h):
    z = -alpha * h
    return ca.fmax(z, 0.0) + ca.log(1.0 + ca.exp(-ca.fabs(z)))

cost_track = track_weight * (
    _barrier(h1) + _barrier(h2) + _barrier(h3) + _barrier(h4)
)
```

This term is added to both **stage cost** and **terminal cost**.

### Full stage cost

```
L_k = ql·el²  +  qc·‖ec‖²  +  Qw·‖ω‖²  +  r_dvt·(Δvθ)²  +  R_df·‖Δf‖²  −  μ·vθ
      ╰──────────── existing MPCC cost ──────────────╯
    + track_weight · Σ_{i=1}^{4} log(1 + exp(−100·h_i))
      ╰─────────────── NEW track constraint cost ───────╯
```

### Default weights

| Parameter | Default | Description |
|---|---|---|
| `ql` | 10.0 | Lag error penalty |
| `Qw` | 0.1 | Body-rate regularisation |
| `r_dvt` | 0.1 | Progress acceleration cost |
| `R_df` | 0.01 | Rotor thrust-rate cost |
| `mu` | 10.0 | Progress reward (higher = faster) |
| `track_weight` | 10.0 | Track constraint log-barrier weight |

---

## 7. Controller Integration (drone_mpcc_controller.py)

### from_config()

Automatically detects the YAML format:

```python
# gates_config.yaml → GatedRacingPath + active Track Constraint
config = {'gates': [...], 'track': {...}}
ctrl = DroneMpccController.from_config(config)

# waypoints_config.yaml → RacingPath + inactive constraint (W=H=1000 m)
config = {'waypoints': np.array(...)}
ctrl = DroneMpccController.from_config(config)
```

### _run_step()

The key change: `get_horizon_params()` was replaced by `get_tunnel_params()`:

```python
# OLD (7-column, no tunnel constraint)
stage_params = self._path.get_horizon_params(theta, N+1, dt, vtheta_guess)

# NEW (15-column, with tunnel constraint geometry)
stage_params = self._path.get_tunnel_params(theta, N+1, dt, vtheta_guess,
                                             qc_nom=self._qc_nom,
                                             qc_gate=self._qc_gate)
```

`RacingPath.get_tunnel_params()` (base class, no gates) returns `W=H=1000` — effectively disabling the constraint. `GatedRacingPath.get_tunnel_params()` (override) returns the actual gate geometry.

---

## 8. RViz Gate Visualization

**File:** `src/drone_control/drone_control/waypoint_visualizer.py`

### Gate rectangles (LINE_LIST)

Each gate is rendered as a rectangle in RViz using `Marker.LINE_LIST`.
The 4 corner positions in NED are:

```python
n_ned = [−sin(yaw), cos(yaw), 0]   # horizontal right direction
b_ned = [0, 0, −1]                  # NED up direction

corners = [
    centre − half_w·n − half_h·b,   # bottom-left
    centre + half_w·n − half_h·b,   # bottom-right
    centre + half_w·n + half_h·b,   # top-right
    centre − half_w·n + half_h·b,   # top-left
]
```

The 4 edges BL→BR, BR→TR, TR→TL, TL→BL are each drawn as a pair of points in the LINE_LIST.
All corners are converted NED→ENU before publishing.

**Colour:** bright green (`r=0.2, g=1.0, b=0.2`), line width 0.10 m.

### Gate direction arrows (ARROW)

Each gate has a short arrow (0.8 m total) pointing in the `yaw_deg` direction:

```python
t_ned = [cos(yaw), sin(yaw), 0]   # flight direction through gate
start = centre − 0.4·t_ned
end   = centre + 0.4·t_ned
```

**Colour:** cyan (`r=0.0, g=0.8, b=1.0`).

### Published topics

| Topic | Type | Content |
|---|---|---|
| `/waypoints/markers` | `MarkerArray` | Gate frames, arrows, labels, drone, trail |
| `/mpcc/reference_path` | `MarkerArray` | Reference spline (purple) |
| `/mpcc/predicted_horizon` | `MarkerArray` | OCP predicted trajectory (green) |
| `/mpcc/progress_point` | `MarkerArray` | Current path projection point (yellow) |

---

## 9. How to Run

### Prerequisites

Same as before — PX4 SITL + Gazebo running.

### Launch

```bash
# Default: MPCC with gates_config.yaml
ros2 launch drone_control sim.launch.py

# Explicit gate file
ros2 launch drone_control sim.launch.py \
    controller_type:=mpcc \
    waypoints_file:=/full/path/to/my_gates.yaml

# MPC with old waypoints (unchanged)
ros2 launch drone_control sim.launch.py \
    controller_type:=mpc \
    waypoints_file:=$(ros2 pkg prefix drone_control)/share/drone_control/config/waypoints_config.yaml
```

### First run after this change

ACADOS will regenerate all C code automatically (the OCP structure changed: NP 7→15).
**This takes 30–90 seconds.** Subsequent runs use the cached compiled `.so` file and start in ~2 seconds.

You will see output like:
```
[INFO] Generating code for acados OCP in /…/src/mpcc/c_generated_code/.
[INFO] Compiling acados OCP solver …
DroneOptimizerMpcc: N=20, T=1.0 s, dt=0.050 s
  nx=19, nu=5, np=15
  Weights: ql=10.0, mu=10.0, Qw=0.1, r_dvt=0.1, R_df=0.01, track_weight=10.0
[DroneMpccController] Optimizer ready. Path: GatedRacingPath (Track Constraint active), length: XX.XX m
```

### Sending a goal

Same as before:
```bash
ros2 run drone_control send_trajectory_goal goal_only
```

The MPCC controller ignores the goal and follows the internal gated path.

---

## 10. Tuning Parameters

### track_weight

Controls how hard the track constraint is enforced:

| Value | Effect |
|---|---|
| 0.0 | Constraint disabled — same as old MPCC |
| 1.0 | Soft nudge — drone prefers to stay inside but may deviate |
| 10.0 | Default — strong penalty for gate violations |
| 50.0+ | Very hard — may degrade solver convergence near boundaries |

Set in `gates_config.yaml`:
```yaml
track:
  track_weight: 10.0
```

Or override per-launch:
```python
weights = {'track_weight': 20.0}
ctrl = DroneMpccController(gates=gates, track_params=tp, weights=weights)
```

### W_nominal / H_nominal

How much room the drone has between gates:
- Large values (5–10 m): drone has maximum freedom, follows pure progress optimisation
- Small values (1–2 m): tunnel is tight everywhere, drone tracks reference closely

### sigma_gate

Controls how far from each gate the tunnel starts narrowing:
- `sigma_gate = 1.0 m`: sharp narrowing, starts ~2 m before gate
- `sigma_gate = 2.0 m`: gradual narrowing, starts ~4 m before gate (default)
- `sigma_gate = 4.0 m`: very gradual — large section of the track is narrow

### mu (progress reward)

Higher `mu` = drone flies faster but takes more risks:
- `mu = 5.0`: conservative, may slow down at gates
- `mu = 10.0`: default balanced setting
- `mu = 20.0`: aggressive, high speed

### ql (lag weight)

Higher `ql` = drone tracks the reference path more closely:
- Low `ql`: drone deviates freely in the tangent direction (time-optimal)
- High `ql`: drone stays close to the reference arc, less time-optimal

### Gate width / height

The aperture dimensions in `gates_config.yaml`:
- `width: 2.0, height: 2.0` → 2 m × 2 m inner aperture (W_gate = 1.0 m, H_gate = 1.0 m)
- Smaller values = tighter constraint = safer but harder for the solver

The drone's body is about 0.5 m wide, so `width ≥ 1.0` is the absolute minimum.

---

## 11. ACADOS Code Regeneration

The parameter vector `NP` changed from **7 to 15**.
ACADOS will detect this via the updated JSON file and regenerate C code automatically.

**You do not need to do anything manually.**

The files in `src/mpcc/c_generated_code/` will be overwritten. The JSON file `src/mpcc/drone_mpcc_acados_ocp.json` will also be updated.

If regeneration fails, delete the cached files and retry:
```bash
rm -rf src/mpcc/c_generated_code/
rm -f  src/mpcc/drone_mpcc_acados_ocp.json
ros2 launch drone_control sim.launch.py   # will regenerate from scratch
```

---

## 12. Backwards Compatibility

### test.py

`src/mpcc/test.py` still works unchanged. It calls:
```python
mpcc = DroneMpccController(waypoints, t_horizon=1.0, n_nodes=20)
```

The constructor detects `waypoints` (not `gates`) and uses plain `RacingPath`.
`RacingPath.get_tunnel_params()` returns `W=H=1000 m` → track constraint is inactive.
The OCP uses NP=15 but the extra parameters are benign large values.

### MPC controller

`DroneMpcController.from_config(config)` still receives `config['waypoints']` (the gate centres extracted as positions), which it uses normally.

### Old waypoints_config.yaml

Still works if passed explicitly:
```bash
ros2 launch drone_control sim.launch.py \
    controller_type:=mpcc \
    waypoints_file:=…/waypoints_config.yaml
```

The `_load_track_config()` function detects the `waypoints:` key and falls back to the legacy format. MPCC will run without the track constraint (W=H=1000).
