# Reinforcement Learning (RL)

**Page Owner:** [Maté Tirpák](https://github.com/matetirpak), [Giorgos Evangelou](https://github.com/evangeloug)

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

