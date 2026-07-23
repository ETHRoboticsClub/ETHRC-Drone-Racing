# Boreas 4S SITL Dynamics and Calibration

**Page Owner:** [Leads of the Aeronautics Division](https://github.com/orgs/ETHRoboticsClub/teams/aeronautics/)

---

## Model Status

The Betaflight SITL plant targets **Boreas 4S**. Its rigid-body properties are
CAD-derived and authoritative. Static maximum thrust is measured at **916 gf
per propeller**; the intermediate thrust curve, motor transient, reaction
torque, and aerodynamic coefficients remain provisional. The profile status is
therefore `partially_measured`, not a fully validated digital twin.

The single source of truth is:

```text
config/airframes/boreas_4s.yaml
```

Every parameter records a provenance and calibration status. The profile is
loaded by both the Gazebo plant and the MPC.

## Rigid-Body Properties

The CAD-derived URDF describes the complete flight-ready Boreas vehicle. The
visual propeller links therefore have negligible inertial values and must not
add mass a second time.

| Property | Value |
| --- | ---: |
| Mass | `0.8351086449771665 kg` |
| CoG | `[0.00483918, 0.00340842, 0.00927557] m` |
| `Ixx` | `0.002158 kg m²` |
| `Iyy` | `0.002919 kg m²` |
| `Izz` | `0.002743 kg m²` |

Validate that the profile, URDF, SDF, rotor order, and rotor geometry agree:

```bash
python3 scripts/simulation/validate_boreas_model.py
```

This check should run before accepting changes to any Boreas model file.

## Propulsion Model

`EthrcBetaflightSITLPlugin` is the only source of propulsion. Do not add
`LiftDrag` or `MulticopterMotorModel` alongside it, because that would count
rotor forces twice.

For each rotor, the plugin applies:

```text
n_target = clamp(Betaflight motor output, 0, 1)
dn/dt    = (n_target - n) / tau_up_or_down
T        = c0 + c1*n + c2*n²
Q        = direction * moment_ratio * T
```

`T` acts along body `+Z` at the CAD rotor position. `Q` is the propeller
reaction torque. Translational and angular drag are component-wise quadratic
in the body frame.

The measured endpoint gives:

```text
maximum thrust/prop  =  8.9828914 N (916 gf)
maximum total thrust = 35.9315656 N
vehicle weight       =  8.1896182 N
thrust-to-weight      =  4.38595 (using the training gravity convention)
hover rotor state    =  0.477412 (quadratic prior)
```

Only the maximum is measured. Until a multi-point sweep is available, the
curve is quadratic through zero and that endpoint. Motor time constants are
currently `0`: no step response has been measured, so SITL does not invent a
lag. Replace both values when identified data is available.

## Diagnostics

The plant publishes Gazebo Transport diagnostics:

| Topic | Meaning |
| --- | --- |
| `/drone/sim/model_profile` | Airframe and calibration status |
| `/drone/sim/motor_command` | Raw Betaflight normalized outputs |
| `/drone/sim/motor_rpm` | Filtered mechanical rotor RPM |
| `/drone/sim/motor_thrust` | Per-rotor thrust in newtons |
| `/drone/sim/applied_wrench` | Total simulated world-frame wrench |

Inspect one message with:

```bash
gz topic -e -n 1 -t /drone/sim/motor_thrust
```

## Automated Plant Validation

The airborne validation world runs at a 1 ms physics step and bypasses
Betaflight by injecting deterministic servo packets. It checks profile loading,
hover thrust, actuator cutoff, applied wrench, odometry, and real-time factor:

```bash
source scripts/source_sitl_env.sh
python3 scripts/simulation/validate_sitl_plant.py \
  --report /tmp/boreas_plant_report.json
```

The normal racing worlds use a 2 ms step. The 1 ms validation world is retained
as a performance benchmark; on the reference WSL development machine it does
not consistently sustain `0.9` real-time factor.

The 2026-07-13 measured-endpoint reference run passed with `8.1896180 N`
applied hover force against `8.1896182 N` expected weight, a `0.477412` hover
command, less than `6e-8 N` per-motor hover error, and less than `6e-7 N` per
motor after cutoff. Use `/drone/odometry` for
world-position checks; Betaflight's console position fields are firmware debug
values and are not the plant ground truth.

## Hardware-Faithful RL Contract

SITL and the real Boreas share ACTUAL rates, zero expo, and `500 deg/s` at full
stick. SITL uses lock-step gyro/PID scheduling and a conservative simulation
rate tune (`P5/I20/D0`) because the motor transient has not been identified;
do not claim hardware-PID parity yet. The shared deployment adapter converts
the policy's desired force through the inverse Boreas propeller curve before
sending Betaflight throttle.

The nominal residual-policy baseline passed armed hover and delayed `+/-1 m`
X/Y goals. The hover endpoint was approximately `(-0.016, 0.013, 0.479) m` for
a `(0, 0, 0.5) m` goal; all four cardinal goals finished within about `3 cm`.
The deployed default keeps learned residual authority at zero. The current
Isaac residual candidates were rejected by paired nominal and armed Gazebo
tests; do not enable one for real flight until the Betaflight inner-loop model
is included in training or identified to equivalent fidelity.

Validate either controller through ROS, Betaflight, and Gazebo without arming:

```bash
python3 scripts/simulation/validate_sitl_controller.py rl
python3 scripts/simulation/validate_sitl_controller.py mpc
```

Pass `--arm` only in an isolated test session where vehicle motion is intended.

## Physical Identification Campaign

When Boreas is available, record the exact motor, propeller, ESC firmware, and
battery used. A thrust-stand CSV must contain at least:

```text
rpm,thrust_n
```

Recommended additional columns are:

```text
timestamp_s,motor_command,voltage_v,current_a,torque_nm,battery_temperature_c
```

Collect steady-state points across the usable RPM range at several 4S voltage
levels. Record separate rising and falling step responses for actuator time
constants. Fit the static curve with:

```bash
python3 scripts/simulation/fit_boreas_propulsion.py measurements.csv \
  --maximum-rpm 30000 \
  --report propulsion_fit.json
```

Do not copy coefficients into the profile unless:

- the data includes at least three repeated sweeps;
- held-out residuals are reported;
- the fitted maximum thrust is physically plausible;
- motor and propeller identifiers are recorded; and
- the profile provenance changes from `provisional` to a dataset identifier.

## Flight Identification

Use Vicon and record synchronized:

- Vicon pose;
- `/imu0`;
- `/motor_rpm`;
- `/battery`;
- `/drone_attitude_cmd`;
- Betaflight motor outputs or Blackbox motor fields; and
- the exact Betaflight configuration hash.

Perform safe hover, axis steps, multisine rate excitation, straight
acceleration, and coast-down runs. Fit drag and transient parameters on one set
of flights and validate them on different flights.

Acceptance targets are:

- static thrust error below 5% over the usable range;
- RPM and actuator transient error below 10%;
- hover command within `0.03` normalized throttle;
- attitude-rate rise-time and steady-state error below 15%; and
- documented residuals on held-out Vicon runs.

Battery sag, ground effect, propwash, frame flexibility, and thermal behavior
remain deferred until the simpler model fails those held-out tests.
