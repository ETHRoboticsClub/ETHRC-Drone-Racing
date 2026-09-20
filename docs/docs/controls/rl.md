# Reinforcement Learning (RL)

**Page Owner:** [Maté Tirpák](https://github.com/matetirpak), [Giorgos Evangelou](https://github.com/evangeloug)

---

## Current Boreas Deployment Contract

The active checkpoint consumes 12 values in this order: projected gravity,
body angular velocity, one body-frame waypoint divided by 10, and body linear
velocity. Its four actions are training collective force followed by FLU roll,
pitch, and yaw rates.

Deployment maps full-scale rate actions to the real linear Boreas Betaflight
profile (`500 deg/s`). Yaw changes sign at the Betaflight boundary; roll and
pitch retain the Isaac/FLU stick sign. The training collective force
(`35.9315656 N` maximum) is converted through the inverse measured Boreas thrust
curve rather than sent directly as throttle.

The deployed controller is now a residual architecture: a conservative
position/velocity outer loop supplies collective thrust and body-rate targets,
and PPO learns only a bounded correction. The nominal path passed armed
Betaflight SITL hover and delayed `+/-1 m` X/Y waypoint tests with about `3 cm`
final 3-D error. The older direct-action checkpoint is not accepted for flight.

## Boreas transfer retraining

The hardware-transfer job is
`ext/DroneRacing_ManagerBased_Enviroment/configs/sweeps/single/boreas_transfer.yaml`.
It uses the Boreas-accurate URDF, measured `916 gf` maximum thrust per propeller,
linear `500 deg/s` ACTUAL rates, the ROS throttle slew limit, and a bounded
residual action. Motor lag is zero until a step response is measured; training
against an invented transient is prohibited. Its cardinal evaluation writes
`boreas_contract_trace.jsonl` beside the checkpoint.

The safe default is the qualified nominal controller
(`RL_RESIDUAL_POLICY=true`, `RL_RESIDUAL_SCALE=0`). A learned residual is
enabled only for an accepted checkpoint with `RL_RESIDUAL_SCALE=0.15`,
`RL_TRAINING_MAX_THRUST_N=35.9315656`, and `RL_POLICY_OUTPUT_TANH=true`.
The default `RL_RESIDUAL_MASK=0,1,1,0` limits learned correction to roll and
pitch; collective thrust and yaw remain under the Gazebo-qualified nominal
loop until their real transient models are identified.
The checkpoint must beat its paired zero-residual baseline, pass
trajectory-based cardinal acceptance, and pass the armed Gazebo gates before
promotion. Verify the interface before judging dynamics:

```bash
python3 scripts/simulation/validate_sitl_controller.py rl --arm \
  --trace /tmp/boreas_sitl_trace.jsonl
python3 scripts/simulation/compare_policy_traces.py \
  /path/to/boreas_contract_trace.jsonl /tmp/boreas_sitl_trace.jsonl
```

The comparator defaults to the first synchronized identical-state row. Use
`--rows N` only when both traces were driven from the same recorded state
sequence; independently simulated trajectories will diverge.

### Qualification status — 2026-07-14

No learned residual checkpoint is currently promoted. The paired-baseline
experiments reject the learned correction: the most faithful lateral-only run
(`df1469e`) had `0.5982 m` mean error versus `0.5351 m` for nominal and
regressed west/south progress. Direct Betaflight/Gazebo testing also rejects
the earlier full-axis checkpoints. This is a simulator-transfer finding, not
an acceptance-threshold adjustment: the Isaac inner rate/actuator model must
be aligned with lockstep Betaflight before a learned residual is enabled.

The robustness fine-tune (`21fe967`, v13) resumed the exact v12 checkpoint at
`0.02` lateral authority with bounded model, sensor and latency randomization.
It remained unqualified: learned mean error was `0.5448 m` versus `0.5454 m`
nominal, but east/north progress regressed by `0.0085/0.0044 m`, and the
absolute `0.5 m` mean / `1.0 m` P95 gates were not met. Terminations and
saturation were zero. The checkpoint is not promoted.

---

## 1. Compute Requirements (Optional)

If your local machine does not have enough GPU compute, you can request a cloud VM to run training.

Follow the instructions here:  
**https://www.notion.so/Getting-started-with-AWS-and-the-Nvidia-IsaacSim-Workstation-AMI-29cca831cdd380eb9a72c2de794d1ef3**

Once the VM is ready (or if you use your own machine), continue with the steps below.

---

## 2. Clone the Repository

```bash
git clone git@github.com:ETHRoboticsClub/IsaacLab_aeronautics.git
cd IsaacLab_aeronautics
```

## 3. Start IsaacLab (Docker-Based Setup)

IsaacLab runs entirely inside Docker. Start the container with:

```bash
python docker/container.py start
```
Then enter the running container:
```bash
python docker/container.py enter
```
Inside the container, your terminal prompt will look like:
```bash
root@ip-xxx-xx-xx-xxx:/workspace/isaaclab#
```
This means you are now working inside the IsaacLab environment.

## 4. Location of the Custom Quadrotor Environment

Our modified quadrotor environment (a copy of the default quadcopter task) is located at:
```bash
source/
    Isaaclab_tasks/
        Isaaclab_tasks/
            direct/
                quadcopter_copy/
```
Modify files in this folder to change:

- the policy  
- dynamics  
- task logic  
- observations  
- rewards  
- or any other environment behavior

## 5. Training the Quadrotor Environment
Inside the container, run:
```bash
python scripts/reinforcement_learning/rsl_rl/train.py \
    --task=Isaac-Quadcopter-Direct-v0-Copy \
    --headless \
    --video \
    --video_length 500 \
    --video_interval 1000
```

### Explanation of Flags

| Flag | Description |
|------|-------------|
| `--task=Isaac-Quadcopter-Direct-v0-Copy` | Selects the custom quadrotor task you created. |
| `--headless` | Runs training without rendering (faster, required on VMs). |
| `--video` | Enables video recording during training. |
| `--video_length 500` | Each recorded video clip is 500 simulation steps long. |
| `--video_interval 1000` | Records a new video every 1000 training iterations. |

Videos are saved automatically inside the `runs/` directory.

---

## 6. Additional Resources

For more details on:

- environment creation  
- task registration  
- RL integrations  
- logging, debugging, or visualization  

refer to the official IsaacLab documentation:

👉 **https://isaac-sim.github.io/IsaacLab/**

## 7. Overview of the Current Environment Setup

The provided `QuadcopterEnvCopy` environment is a modified version of the default IsaacLab quadcopter task.  
It introduces a **time-based circular trajectory** that the drone must follow, along with small vertical oscillations.

### What the Drone Is Trained to Do

The quadrotor is trained using reinforcement learning to:

- Track a **moving goal position** that follows a circular path around each environment's origin.
- Maintain stable flight while adjusting thrust and rotational moments.
- Minimize:
  - linear velocity error  
  - angular velocity error  
  - distance to the moving goal  

### How the Trajectory Works

Each environment receives:
- A unique random **phase offset** (so not all drones fly in sync),
- A circular XY path with adjustable radius and period,
- A small vertical oscillation added to the Z position.

The desired trajectory is computed every step and used to generate:
- **Observations** (target position in the quadrotor’s body frame),
- **Rewards** (distance-to-goal mapping, velocity penalties),
- **Done conditions** (crashing or flying too high).

### Summary

In its current form, the drone learns to **fly smoothly along a dynamic circular trajectory**, staying close to a moving 3D point while maintaining stable and efficient control.  
This setup is ideal for testing trajectory tracking, stability control, and low-level motor policy behavior.
