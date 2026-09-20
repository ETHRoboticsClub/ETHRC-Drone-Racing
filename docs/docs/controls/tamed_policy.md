# Tamed RL Policy Configuration for State Estimation

**Page Owner:** Aeronautics Division

---

## 🎯 Overview & Rationale

Standard high-speed drone racing RL policies optimize strictly for minimal lap times, resulting in extreme angular velocities, sharp attitude step changes, and velocity spikes. While effective in pure simulation, these aggressive maneuvers cause severe challenges for physical state estimation:

1. **Camera Motion Blur & Feature Loss**: High roll/pitch/yaw rates break visual gate tracking (`gate_detector_node`) and VIO feature tracking.
2. **IMU Integration Drift**: Extreme accelerations and high-frequency control chattering amplify IMU integration drift during dead-reckoning (`aigp_state_estimator_node`).
3. **Out-of-FOV Gates**: Large tilt angles swing the front camera away from upcoming gates, preventing visual pose resets (`/aigp_state_estimator/set_pose`).

To address this, we created a **Tamed Policy Environment Configuration** inside `ext/DroneRacing_ManagerBased_Enviroment` designed specifically to train smooth, state-estimator-friendly racing policies.

> [!WARNING]
> The completed tamed run is useful reward-tuning evidence, but it is not a
> deployable AIGP result: it used the old default Crazyflie/moments/50 Hz task.
> Its checkpoint is 105D, while the former ROS/export path was 25D. The strict
> exporter now reproduces the 105D actor numerically, and new AIGP training must
> use `configs/sweeps/single/aigp_sensor_policy.yaml` before deployment.

---

## 🛠️ Key Hyperparameter Changes

The tamed policy environment tunes key reward penalties and physical action constraints:

| Parameter / Reward Term | Standard Policy | Tamed Policy | Purpose for State Estimator |
|---|---|---|---|
| `ANGULAR_VELOCITY` penalty weight | `-0.001` | **`-0.02`** | Suppresses rapid spins and flips; keeps rotation rates low for clean camera frames |
| `ORIENTATION` penalty weight | `-0.005` | **`-0.03`** | Restricts steep roll/pitch tilt angles; keeps gates within camera field of view |
| `ACTION_SMOOTHNESS` penalty weight | `-0.0001` | **`-0.003`** | Penalizes high-frequency step changes in torque/thrust actions to prevent chattering |
| `VELOCITY_LIMIT` penalty weight | `-0.05` | **`-0.25`** | Strictly enforces velocity bounds to prevent unmanageable speed build-up |
| `velocity_limit` cap | `15.0 m/s` | **`10.0 m/s`** | Bounds total linear speed for steady visual feature tracking |
| `action_noise.lpf_alpha` | `1.0` | **`0.85`** | Applies low-pass filtering on actions during training to simulate realistic actuator lag |
| `action_noise.gaussian_std` | `0.0` | **`0.015`** | Adds additive action noise to build policy robustness against control perturbations |

---

## 🚀 AWS Batch Training Job

A training run for the tamed policy was submitted to AWS Batch GPU infrastructure.

- **Config Path**: `ext/DroneRacing_ManagerBased_Enviroment/configs/sweeps/single/tamed_state_estimator_policy.yaml`
- **S3 Input URI**: `s3://ethrc-aeronautics/droneracing/isaac-rl-runs/sweeps/inputs/runs/2026-07-22/tamed_policy/sweep.yaml`
- **Git Branch Ref**: `feature/tamed-policy` (Pushed to `origin`)
- **AWS Batch Job ID**: `c9dcc19d-59fb-49f3-af46-040f52dd23b0`
- **Run ID**: `run-20260722T144103Z-8eaeaf`
- **S3 Outputs URI**: `s3://ethrc-aeronautics/droneracing/isaac-rl-runs/sweeps/outputs/runs/run-20260722T144103Z-8eaeaf/`

### Tracking Run Status & Downloading Artifacts

To inspect the live training status:
```bash
cd ext/DroneRacing_ManagerBased_Enviroment
make status JOB_ID=c9dcc19d-59fb-49f3-af46-040f52dd23b0
```

To tail CloudWatch container logs:
```bash
make logs JOB_ID=c9dcc19d-59fb-49f3-af46-040f52dd23b0
```

Once the job finishes (`SUCCEEDED`), download the trained policy checkpoint and logs:
```bash
make download_results RUN_ID=run-20260722T144103Z-8eaeaf DEST=ext/tamed_policy/
```

> [!NOTE]
> The existing Nicola baseline model (`ext/nicola/policy.pt` and `ext/nicola/policy.onnx`) remains untouched.

## AIGP-Compatible Retraining

The replacement sweep uses the competition track and gate geometry, 120 Hz
physics with a 60 Hz command policy, Anemos rates control, a measured
thrust-to-weight ratio of 2.33, randomized command delay/PID gains, observation
corruption, a 4 rad/s (approximately 229 deg/s) rate scale, and the exact 105D
sensor observation contract:

```bash
cd ext/DroneRacing_ManagerBased_Enviroment
make submit \
  CONFIG=configs/sweeps/single/aigp_sensor_policy.yaml \
  S3_KEY=runs/aigp_sensor_policy/sweep.yaml \
  BRANCH=<branch-containing-these-changes>
```

An exported artifact is accepted only when checkpoint-to-actor and
actor-to-TorchScript parity are both within `1e-5`.
