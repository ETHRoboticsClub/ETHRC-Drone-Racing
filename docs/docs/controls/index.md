# Controls

**Page Owner:** [Saad Lahlou](https://github.com/saadl27)

---

Welcome to the Controls team documentation. This section describes the control algorithms, flight stack configuration, and how components interact during flight.

## Overview

The architecture is designed to be **modular**, **controller-agnostic**, and tightly integrated with **ROS 2**, while relying on MAVSDK for low-level communication with the vehicle.

## Codebase Architecture

At a high level, the control stack is organized around three main layers:

1. **ROS 2 control interface (`drone_control`)**
2. **Controller implementations (MPC, RL)**
3. **Low-level vehicle interface (MAVSDK / PX4)**

Each layer has a clearly defined responsibility, allowing controllers to be swapped or extended with minimal changes to the rest of the system.

---

## `drone_control`: ROS 2 Control Interface

The `drone_control` module is the **central ROS 2 interface for flight control**.

It acts as the glue between:
- The **ground station**
- The **perception and state estimation stack**
- The **controller (MPC or RL)**
- The **low-level vehicle interface (MAVSDK)**

### Responsibilities

`drone_control` is responsible for:

- Subscribing to ROS 2 topics providing:
  - State estimation (pose, velocity, attitude, etc.)
  - Setpoints and mission-level commands
  - System and control-related signals from the ground station
- Managing the **control logic and execution flow**
- Selecting and running the active controller
- Publishing **low-level control commands** to ROS 2 topics

These command topics are then consumed by the MAVSDK client, which forwards them to the drone.

---

## Command Path: ROS 2 → MAVSDK → Drone

The control flow is as follows:

1. **State estimation and setpoints** are produced by the perception stack and/or ground station and published on ROS 2 topics.
2. `drone_control`:
   - Collects the current state
   - Passes it to the active controller
   - Receives actuator-level or rate-level commands
3. `drone_control` publishes these commands on a dedicated ROS 2 topic.
4. **MAVSDK subscribes to this topic** and:
   - Converts the commands to MAVSDK offboard messages
   - Sends them to the drone via MAVLink

This design ensures a clean separation between **control logic** and **vehicle communication**.

---

## Controller Implementations: MPC and RL

The actual control laws are implemented in separate modules:

- **MPC**: Model Predictive Control–based controller
- **RL**: Reinforcement Learning–based controller

Each controller:
- Consumes the current estimated state
- Outputs control commands (e.g. body rates + thrust or per-motor thrust) 

Currently, the MPC outputs body rates + collective thrust, (we just realize that MAVSDK can send per-motor thrust), it is planned to change the model to output per-motor thrust.

### Modularity Goal

A key design goal is to make `drone_control` **fully controller-agnostic**.

Any controller that follows the expected interface can be plugged in, without modifying:
- MAVSDK integration
- ROS 2 communication
- Ground station or perception interfaces

At the moment:
- The system has been **validated and used primarily with MPC**
- The RL pipeline still requires:
  - Policy extraction
  - Runtime inference integration
  - ROS 2–level interfacing

---

## Current Status and Future Work

- **MPC** is fully integrated and operational
- **RL** support is under development
  - Policy loading and inference need to be finalized
  - Interface alignment with `drone_control` is still required

The existing architecture already supports this extension, and only the controller-specific logic needs to be added.

For detailed information on each controller and their interfaces, please refer to:
- [MPC](mpc.md)
- [RL](rl.md)
- [DRONE_CONTROL](drone_control.md)
- [MAVSDK](mavsdk.md)


**Troubleshooting**: 
For troubleshooting of common problems, check [General Control Troubleshooting](troubleshooting.md)
