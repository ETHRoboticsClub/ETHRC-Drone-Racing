# Flying the Drone (Simulation)

**Page Owner:** [Mattia De Martino](https://github.com/mattiademartino)

---

Once the simulation and controller are running, you can command the drone using our standard goal-sending utilities. This section is primarily relevant for the **MPC Team** and those testing high-level control logic in simulation.

## Prerequisites

Ensure you have the Docker environment running and the workspace built.
See [Getting Started](../getting-started.md) for setup instructions.

## Step 1: Launch the Controller

In a container terminal, start the MPC controller:
```bash
source install/setup.bash
ros2 launch drone_control drone_controller.launch.py
```

## Step 2: Send a Pose Goal (Takeoff)

In a **separate** container terminal, use the `send_pose_goal` script to command a takeoff to 2 meters:
```bash
source install/setup.bash
ros2 run drone_control send_pose_goal goal_only -x 0.0 -y 0.0 -z 2.0 -yaw 90
```

### Argument Breakdown

| Argument | Description |
| :--- | :--- |
| `goal_only` | Sends the goal and waits for completion. Use `cancel` to stop an active mission. |
| `-x`, `-y`, `-z` | Target coordinates in meters. |
| `-yaw` | Target orientation in **degrees**. |

## Step 3: Monitor in PlotJuggler

The controller launch automatically opens **PlotJuggler**. Look for the `/drone_state_sim` and `/drone_current_target` topics to verify the drone is tracking the reference accurately.

---

## Technical Details

For more information on how the controller works, see:
- [MPC Controller](mpc.md)
- [Drone Control Module](drone_control.md)
