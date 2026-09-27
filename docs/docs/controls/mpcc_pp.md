# Model Predictive Contouring Control++ (MPCC++)

**Owners:** [Max Hassdenteufel](https://github.com/Max1284), [Mattia De Martino](https://github.com/mattiademartino)

MPCC++ is the racing controller in `src/mpcc_pp`. Unlike point-to-point MPC, it optimizes progress along a continuous path while penalizing lag, contouring error, aggressive motion, and violations of a gate-aware safety tunnel. The package contains the augmented quadrotor model, ACADOS optimal-control problem, path and tunnel geometry, the controller adapter, a lightweight dynamics simulator, and two residual-learning approaches.

Automatic parameter search is documented separately on the [MPCC++ Optuna and tuning](mpcc_pp_optuna.md) page.

Status: integrated with `drone_control` for PX4/Gazebo and the package's own dynamics simulator. The controller uses NED/FRD internally and accepts the racing node's 13-element NED/FRD state. The checked-in parameters are simulation-oriented; this page does not claim hardware validation.

## Architecture

```text
gates_config.yaml + parameters.yaml
                 │
                 ▼
       RacingPath / GatedRacingPath
       19 geometry values per stage
                 │
                 ▼
13-state estimate ──► DroneMpccPpController ──► DroneOptimizerMpccPp
 [p,v,q,w]              │   ▲                     ACADOS SQP-RTI
                        │   └── previous thrust/progress and warm start
                        │
                        ├── AttitudeRate (body rates + collective command)
                        └── raw MPCC input [dvtheta, df0..df3]
                              for the lightweight simulator
```

The preferred path is gate-based. A `GatedRacingPath` activates the safety tunnel and uses each gate's center, yaw, width, and height. The legacy waypoint path is retained for compatibility, but passes `W = H = 1000 m`, effectively disabling the tunnel barrier.

## State, control, and frames

The OCP state has 19 elements:

| Indices | Symbol | Meaning | Frame / unit |
| :--- | :--- | :--- | :--- |
| `0:3` | `p` | position | NED, m |
| `3:7` | `q` | body-to-world quaternion, `wxyz` | FRD → NED |
| `7:10` | `v` | linear velocity | NED, m/s |
| `10:13` | `omega` | body rates | FRD, rad/s |
| `13:17` | `f` | four rotor thrust states | N |
| `17` | `theta` | path progress | m |
| `18` | `vtheta` | path-progress velocity | m/s |

The five controls are `[delta_vtheta, delta_f0, delta_f1, delta_f2, delta_f3]`: progress acceleration in m/s² followed by four rotor-thrust rates in N/s.

`DroneMpccPpController` converts the racing stack's external state `[p(3), v(3), q(4), omega(3)]` into this augmented layout. Rotor thrust, `theta`, and `vtheta` are internal estimates carried between solves. The standalone simulator accepts either layout and can convert its pose output from NED/FRD to ROS ENU/FLU.

## Quadrotor model (`drone_model.py`)

`DroneMpccPpModel` builds the CasADi expressions and exports an `AcadosModel`. Its continuous dynamics are:

| State | Dynamics |
| :--- | :--- |
| Position | `p_dot = v` |
| Velocity | NED gravity minus rotated collective thrust divided by mass, minus rotated body-frame linear drag |
| Quaternion | `q_dot = 0.5 q ⊗ [0, omega]`; the rotation matrix uses a normalized quaternion |
| Body rates | rigid-body rotational dynamics `J omega_dot = M - omega × J omega` |
| Rotor thrust | `f_dot = delta_f` |
| Progress | `theta_dot = vtheta`, `vtheta_dot = delta_vtheta` |

Rotor positions and yaw-moment coefficients map the four thrusts to collective force and body moments. Mass, gravity, diagonal inertia, rotor geometry, force limits, motor constants, drag, and all model constraints come from `parameters.yaml`.

The default constraints are applied to body-rate states, rotor-thrust states, progress velocity, rotor-thrust-rate controls, and progress acceleration. Position, linear velocity, quaternion, and `theta` have no direct box bounds.

### Command conversion

The controller exposes two outputs:

- `controller_update()` and `trajectory_controller_update()` use predicted next-step body rates and mean rotor force to create a MAVSDK `AttitudeRate`. The supplied goal is ignored because MPCC++ follows its internal path.
- `mpcc_pp_update()` converts all four predicted next-step rotor forces to normalized values for a direct-motor consumer.

The current `rotor_forces_to_motor_controls()` implementation linearly maps `[F_MIN, F_MAX]` to `[0, 1]`. Although motor-speed constants are configured and the docstring mentions `F = k omega²`, the function does not currently perform a square-root motor-speed inversion. Treat the resulting value as the package's normalized actuator command, not a physically inverted motor speed.

## Path and tunnel geometry (`path_utils.py`)

`RacingPath` parameterizes ordered 3D points by cumulative chord length, fits cubic splines, and builds a lookup table at `path.lookup_resolution_m`. It provides position, unit tangent, spline second derivative, closest-progress projection, and a gate-centered contouring-weight profile.

`GatedRacingPath` adds:

- a closed circuit through all gate centers;
- cubic Hermite splines whose derivative at each gate follows `yaw_deg`;
- an optional lead-in from the drone's initial position to gate 1;
- periodic circuit geometry after the first lap;
- a local orthonormal frame: tangent `t`, width axis `n`, and height axis `b`;
- gate-aware half-width `W(theta)` and half-height `H(theta)`.

Near a gate, Gaussian profiles shrink `W` and `H` toward half the gate aperture and raise the contouring weight from `qc_nom` toward `qc_gate`. Between gates, the tunnel widens to `W_nominal` and `H_nominal`. `tau` globally scales the resulting half-extents. Near a vertical path section, where a horizontal normal derived from the tangent is ill-defined, the nearest gate's yaw supplies the width direction.

On the first control step, the controller rebuilds the gated path with the measured drone position prepended. At lap completion it wraps progress to gate 1 rather than replaying that lead-in.

### Geometry stage parameters

Every horizon stage receives 19 geometry values:

```text
[p_ref(3), t_ref(3), qc,
 n_ref(3), b_ref(3), W, H,
 theta_ref, curvature_ref(3)]
```

On a warm solve, the controller shifts the previous predicted `theta` trajectory and uses it to sample the next geometry horizon. Otherwise it advances a constant-speed guess using at least `controller.min_prediction_speed`. The measured position is also projected into a local window around the previous progress estimate each step.

## Optimal-control problem (`drone_opt.py`)

`DroneOptimizerMpccPp` formulates an external-cost nonlinear OCP and solves it with ACADOS. The checked-in configuration uses `N = 25`, `T = 1.5 s`, ERK integration, Gauss–Newton Hessian approximation, `PARTIAL_CONDENSING_HPIPM`, and `SQP_RTI`.

For position error `e = p - p_ref`, the lag and contouring errors are:

```text
e_l = t_ref^T e
e_c = e - e_l t_ref
```

The stage cost is the discounted sum of:

```text
ql * e_l²
+ qc(theta) * ||e_c||²
+ Qw * ||omega||²
+ r_dvt * delta_vtheta²
+ R_df * ||delta_f||²
- mu * vtheta
+ track_weight * tunnel_barrier
+ r_theta_dev * (theta - theta_ref)²
```

The terminal cost excludes control increments and is multiplied by `terminal_cost_multiplier`. Per-stage discounting is `discount_factor^k`.

The optimized `theta` need not equal the sampled `theta_ref`. To keep the reference differentiable inside the OCP, position is approximated to second order:

```text
p_ref(theta) ~= p_ref(theta_ref)
             + t_ref * delta_theta
             + 0.5 * curvature_ref * delta_theta²
```

`r_theta_dev` is a soft trust-region penalty that keeps this Taylor approximation local.

### Safety tunnel

The position error is projected into the tunnel frame:

```text
e_n = n_ref^T e
e_b = b_ref^T e
```

The four inside margins are `W ± e_n` and `H ± e_b`. Each is penalized by the numerically stable softplus form of `log(1 + exp(-alpha h))`. This is a soft cost, not a hard nonlinear constraint: sufficiently large competing costs can still permit a violation.

### Full stage-parameter vector

The controller appends seven runtime weights and one discount to the 19 geometry values, producing 27 values per stage:

```text
[geometry(19), ql, Qw, r_dvt, R_df, mu,
 track_weight, r_theta_dev, discount]
```

The runtime weights can change without regenerating the symbolic OCP. Horizon, model, constraints, barrier slope, terminal multiplier, and solver settings are structural and may cause ACADOS to regenerate code. Generated artifacts live under the installed/source `mpcc_pp/c_generated_code` directory; ACADOS uses its JSON hash to decide whether reuse is possible.

The solver accepts ACADOS statuses `0` and `2`, normalizes predicted quaternions, records timing statistics, and shifts the previous state/control solution for the next warm start. A cold start places predicted states on the reference path with identity attitude and hover thrust.

## Controller behavior (`drone_mpcc_pp_controller.py`)

One update performs the following:

1. Build the lead-in on the first gated step.
2. Convert the external state to the augmented state.
3. Project the measured position onto the path and wrap completed laps.
4. Sample path and tunnel parameters over the horizon.
5. Append runtime weights and discounts, then solve the OCP.
6. Optionally correct the next predicted observable state with the MLP residual.
7. Optionally buffer a model-validation row.
8. Update the internal rotor-thrust and progress estimates.
9. Return an attitude-rate command or direct normalized rotor commands.

If the controller has not been initialized, or `controller_update()` catches an exception, it returns zero body rates with the package's normalized hover command. `mpcc_pp_update()` does not provide that exception-catching fallback.

The adapter also exposes the predicted horizon, progress point, tunnel samples, gate positions, lap count, last raw OCP input, and cumulative solver statistics to `racing_controller_node.py` for RViz and tuning metrics.

## Configuration (`parameters.py`, `parameters.yaml`)

Parameter-file selection, from highest to lowest priority, is:

1. an explicit `parameter_file` argument;
2. `MPCC_PP_PARAMETERS_FILE`;
3. bundled `src/mpcc_pp/mpcc_pp/parameters.yaml`.

The controller then deep-merges `track_profiles.default`, followed by the profile named by `active_track`. Legacy tuning keys in `gates_config.yaml` are ignored unless `compatibility.allow_legacy_track_overrides` is `true`; gate positions and aperture geometry always remain in the track file.

| Section | Controls |
| :--- | :--- |
| `controller` | horizon, nodes, prediction-speed floor, lap wrap, rate-command clip |
| `path.contouring` | nominal/gate contouring weights and Gaussian width |
| `path.tunnel` | visualization, nominal half-extents, gate transition width, scale |
| `cost` | OCP weights, discount, terminal multiplier, barrier slope |
| `model.physical` | mass, inertia, rotor model and drag |
| `model.residual` | embedded polynomial residual enable/path |
| `model.constraints` | body rate, thrust, thrust-rate, and progress bounds |
| `solver` | ACADOS QP, NLP, integrator, tolerances, and regularization |
| `visualization` | sample counts, publish throttles, marker styling |
| `track_profiles` | per-track overrides for the sections above |

To run with a separate parameter file:

```bash
export MPCC_PP_PARAMETERS_FILE=/absolute/path/to/parameters.yaml
ros2 launch drone_control sim.launch.py controller_type:=mpcc_pp
```

A track YAML can instead define `mpcc_pp_parameters_file` or `parameter_file`, globally or inside the active track.

## Launching the simulator

Run these commands inside the development container. In each new terminal, enter the workspace and source it first:

```bash
cd ~/ros_ws
source install/setup.bash
```

### Build the required packages

Run this command inside the development container to build the relevant packages for MPC:

```bash
colcon build --packages-select px4_msgs my_mavsdk mpc mpcc mpcc_pp drone_control gazebo_ros_bridges sitl_gazebo_plugins
```

### Lightweight MPCC++ simulator

This mode does not require PX4 or Gazebo. It starts `mpcc_pp_dynamics_simulator`, the racing controller, and the visualizer from one launch file:

```bash
ros2 launch drone_control sim.launch.py \
  controller_type:=mpcc_pp \
  waypoints_file:="$HOME/ros_ws/src/drone_control/config/gates_config.yaml" \
  acados_source_dir:=/opt/acados \
  acados_lib_dir:=/opt/acados/lib \
  own_simulator:=true \
  use_mavsdk:=false \
  rviz:=true
```

`mpcc_pp_dynamics_simulator` integrates the exact symbolic 19-state model with RK4 and zero-order-held inputs. In this mode the controller reads `/mpcc_pp/sim_state`, publishes the raw five-element OCP input on `/mpcc_pp/mpc_action`, skips the MPC stabilization phase, and does not start MAVSDK.

### PX4 SITL and Gazebo

Start PX4 SITL and headless Gazebo in the first terminal from the PX4 tree:

```bash
cd ~/ros_ws/px4/PX4-Autopilot
make px4_sitl gz_zephyros HEADLESS=1
```

Wait for PX4 to report that it is ready for takeoff. In a second sourced workspace terminal, start the ROS controller against the PX4/Gazebo state bridge:

```bash
ros2 launch drone_control sim.launch.py \
  controller_type:=mpcc_pp \
  waypoints_file:="$HOME/ros_ws/src/drone_control/config/gates_config.yaml" \
  acados_source_dir:=/opt/acados \
  acados_lib_dir:=/opt/acados/lib \
  own_simulator:=false \
  use_mavsdk:=true \
  mpcc_stabilization_sec:=3.0 \
  rviz:=true
```

This mode reads the external state from `drone_state_sim`, starts MAVSDK by default, and uses the MPC hover controller for `mpcc_stabilization_sec` before handing control to MPCC++.

### `sim.launch.py` arguments

Append launch arguments as `name:=value`. The launch file exposes all of the following:

| Argument | Default | Description |
| :--- | :--- | :--- |
| `controller_type` | empty | Controller to run: `mpc`, `mpcc`, `mpcc_pp`, `mpc_rotor`, `position_ned`, or `rl`. An empty value defers to the controller configuration. Set `mpcc_pp` for this package. |
| `waypoints_file` | empty | Path to a YAML track file. Gate-format YAML activates the MPCC++ gated path and tunnel; an empty value disables the launch-time trajectory selection. |
| `plotjuggler` | `false` | Start PlotJuggler with the controller launch. |
| `own_simulator` | `false` | `true` selects the lightweight 19-state MPCC++ simulator; `false` selects the external PX4/Gazebo pipeline. |
| `use_mavsdk` | `true` | Start the MAVSDK client in the external-simulator path. The lightweight path forces this off regardless of the supplied value. |
| `mpcc_stabilization_sec` | `3.0` | Seconds of MPC hover before MPCC/MPCC++ takes over in the PX4/Gazebo path. The lightweight path forces this to `0.0`. |
| `rviz` | `true` | Start RViz through the visualizer launch. |
| `model_logging` | `false` | Record `(x, u, x_pred, x_real)` model-validation samples under `src/mpcc_pp/data`. |
| `residual_model_path` | empty | Path to a trained `residual_model.pt`; an empty value disables the post-solve MLP correction. |
| `verbose_input` | `false` | Print periodic MPCC thrust/body-rate input and command-publication diagnostics. |
| `simulator_debug` | `false` | Print the lightweight simulator's current augmented state, input, and integrated next state. It has no effect when `own_simulator:=false`. |
| `simulator_debug_every` | `1` | Print one lightweight-simulator dynamics diagnostic every N integrated actions when `simulator_debug:=true`. |

Boolean values use `true` or `false`. `simulator_debug_every` must be a positive integer in practice; the node clamps it to at least one.

### Lightweight simulator topics

| Direction | Default topic | Type | Content |
| :--- | :--- | :--- | :--- |
| Input | `/mpcc_pp/mpc_state` | `std_msgs/Float64MultiArray` | optional 19-state or external 13-state reset/update |
| Input | `/mpcc_pp/mpc_action` | `std_msgs/Float64MultiArray` | five OCP controls |
| Output | `/mpcc_pp/sim_state` | `std_msgs/Float64MultiArray` | augmented 19-state |
| Output | `drone_state_sim` | `std_msgs/Float64MultiArray` | external 13-state |
| Output | `/mpcc_pp/sim_pose` | `geometry_msgs/PoseStamped` | visualization pose, ENU/FLU by default |

Important node parameters include `dt`, `integration_steps`, initial state fields, `periodic_publish`, `hold_initial_until_first_action`, output toggles, `pose_ned_to_enu`, and dynamics debug throttling. With `integration_steps: 0`, it reuses `solver.sim_method_num_steps`.

## Model validation and residual learning

### CSV logging (`model_logger.py`)

Enable logging through the simulation launch:

```bash
ros2 launch drone_control sim.launch.py \
  own_simulator:=false \
  use_mavsdk:=true \
  controller_type:=mpcc_pp \
  waypoints_file:="$HOME/ros_ws/src/drone_control/config/gates_config.yaml" \
  acados_source_dir:=/opt/acados \
  acados_lib_dir:=/opt/acados/lib \
  mpcc_stabilization_sec:=3.0 \
  rviz:=true \
  model_logging:=true
```

`ModelValidationLogger` writes `model_validation_<timestamp>.csv` under `src/mpcc_pp/data` by default. Each row aligns the previous step's augmented state, applied OCP control, and predicted next state with the next observed 13-state. The first call only fills the buffer; reset discards an unmatched pending sample.

`data/compute_residuals.py` computes channel residuals, norms, summary statistics, and Pearson correlations. Its `CSV_PATH` is currently edited in the script rather than supplied on the command line.

### Post-solve MLP residual (`residual_model.py`)

`data/train_residual_model.py` trains a normalized PyTorch MLP with 24 inputs (`x_aug + u`) and 13 output corrections (`p, q, v, omega`). Enable a checkpoint with:

```bash
ros2 launch drone_control sim.launch.py \
  controller_type:=mpcc_pp \
  residual_model_path:=/absolute/path/to/residual_model.pt
```

This correction is applied to `x_pred[1, 0:13]` after the OCP solve. It does not alter the OCP dynamics or the optimized trajectory, although the corrected next body rates affect the attitude-rate adapter. The training script currently uses source-edited `CSV_PATHS`.

### Embedded paper-style residual (`paper_residual_model.py`)

`data/fit_paper_residual_model.py` fits separate linear models for body linear- and angular-acceleration residuals using small polynomial feature sets of body velocity and a mean-thrust/rotor-speed proxy. It writes `paper_residual_model.json`.

Set `model.residual.enabled: true` and its absolute `coefficients_path` to inject those corrections directly into `v_dot` and `omega_dot` when the ACADOS model is built. This changes the OCP dynamics and therefore requires solver regeneration. It is the residual approach that most closely matches the package's MPCC++ formulation.

## Package entry points and files

| Path / executable | Purpose |
| :--- | :--- |
| `mpcc_pp/drone_model.py` | 19-state CasADi/ACADOS dynamics and actuator mapping |
| `mpcc_pp/drone_opt.py` | external-cost ACADOS OCP and warm-started solve |
| `mpcc_pp/drone_mpcc_pp_controller.py` | `ControllerInterface` adapter and episode statistics |
| `mpcc_pp/path_utils.py` | spline path, progress projection, gate tunnel geometry |
| `mpcc_pp/parameters.py` | YAML loading, profiles, and legacy merge helpers |
| `mpcc_pp/parameters.yaml` | authoritative controller/model/solver tuning |
| `mpcc_pp/dynamics_simulator_node.py` / `mpcc_pp_dynamics_simulator` | exact-model ROS 2 simulator |
| `mpcc_pp/model_logger.py` | delayed model-prediction CSV logger |
| `mpcc_pp/residual_model.py` | post-solve PyTorch residual MLP |
| `mpcc_pp/paper_residual_model.py` | embedded polynomial residual format and inference |
| `data/` | datasets and residual analysis/training scripts |
| `mpcc_pp/tuning/`, `mpcc_pp_optuna_tune`, `mpcc_pp_gazebo_trial` | [Optuna pipeline](mpcc_pp_optuna.md) |

Build and expose the ROS entry points with:

```bash
colcon build --symlink-install --packages-select mpcc_pp drone_control
source install/setup.bash
```

## Known limitations

- Hardware flight has not been established by the package or docs; validate in the lightweight simulator and PX4/Gazebo before any physical test.
- Tunnel safety is a soft penalty, not a hard feasibility guarantee.
- The actuator conversion is a linear force normalization despite the configured quadratic motor model.
- Hard ACADOS failures can be under-counted in tuning statistics: with solver logging enabled, `solve()` raises before the controller calls its statistics recorder, and the per-step solver-status print is commented out.
- The post-solve MLP does not participate in optimization and does not renormalize its corrected quaternion.
- The data scripts use constants edited in source rather than command-line input paths, and their analysis/training dependencies (`pandas`, `torch`) are not fully declared in `setup.py`.
- `package.xml` and `setup.py` still contain placeholder maintainer/license metadata.
- The older `src/mpcc_pp/guide.md`, `codeguide.md`, and `guida_breve.md` mix design proposals with obsolete interfaces; use these docs and the current source for operational behavior.
