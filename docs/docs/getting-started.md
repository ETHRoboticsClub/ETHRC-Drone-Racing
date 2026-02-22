# Getting Started

**Page Owner:** [Leads of the Aeronautics Division](https://github.com/orgs/ETHRoboticsClub/teams/aeronautics/)

---

This repository provides a ready-to-use drone racing environment using Docker. By containerizing the development environment, we ensure consistency across different host systems and simplify the setup of complex robotics software.

## Environment Overview

The environment bundles the following core components:

*   **ROS 2 Humble**: Our primary robotics framework.
*   **Gazebo (gz)**: A high-fidelity 3D simulator for drones.
*   **PX4 SITL**: The autopilot software running in simulation.
*   **MAVROS / MAVSDK**: Communication bridges between PX4 and ROS.
*   **Workspace (`/ws`)**: The location for custom ROS 2 packages.

> [!IMPORTANT]
> **Host OS**: Ubuntu/Linux is highly recommended for GUI compatibility.  
> **Requirements**: Docker Engine + Compose v2.

---

## Quick Start

Follow these steps to get the environment running.

### Step 0: SSH Setup
We use SSH for Git operations to avoid repeated password prompts.  
[Follow GitHub's SSH guide here](https://docs.github.com/en/authentication/connecting-to-github-with-ssh).

### Step 1: Clone the Repository
Ensure you clone with submodules to include the PX4 Autopilot.
```bash
git clone --recurse-submodules git@github.com:ETHRoboticsClub/ETHRC-Drone-Racing.git
cd ETHRC-Drone-Racing
git submodule update --init --recursive
```

### Step 2: Build and Prepare
Build the Docker image and prepare the host volumes for ROS 2.
```bash
docker compose build
./scripts/prepare_colcon_volumes.sh
```

### Step 3: Start Services
Allow GUI applications and start the container.
```bash
xhost +local:root
docker compose up -d
docker exec -it erc-aeronautics bash
```

### Step 4: Build Workspace
Inside the container terminal, build the entire ROS 2 pipeline.
```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --executor sequential
```

---

## Running Simulation

To launch the default simulation environment:

1.  **Source the workspace**:
    ```bash
    source install/setup.bash
    ```
2.  **Launch PX4 SITL with Gazebo**:
    ```bash
    cd /ws/px4/PX4-Autopilot
    make px4_sitl gz_x500_cam 
    ```

---

## Visualization and Monitoring

Visualizing the internal state of the drone is crucial for debugging perception and control algorithms.

### RViz Visualization
We use RViz2 to visualize sensor data, ArUco detections, and EKF state. To launch RViz with the ArUco monitor:
```bash
source install/setup.bash
ros2 launch aruco_pose_estimation aruco_pose_estimation.launch.py rviz:=true
```

### Full Perception Launch
You can launch the integrated perception stack (VIO + ArUco + EKF) using:
```bash
source install/setup.bash
ros2 launch launch/perception.launch.py
```

---

## Troubleshooting & Common Issues

### Permission Errors
If you see permission denied errors during build or when accessing volumes, run the preparation script on the **host**:
```bash
sudo ./scripts/prepare_colcon_volumes.sh
```

### Submodule Issues
If the build fails due to missing source files in `src/perception/VIO` or `src/ros2-vicon-receiver`, ensure your submodules are up to date:
```bash
git submodule update --init --recursive
```

---

## Next Steps

Now that your environment is set up, explore the following resources:

*   [**Perception Modules**](perception/index.md): Learn about VIO, ArUco tracking, and state estimation.
*   [**Control Algorithms**](controls/index.md): Understand MPC and RL-based drone control.
*   [**Development Guidelines**](guidelines.md): Review our coding standards and Git workflow.
