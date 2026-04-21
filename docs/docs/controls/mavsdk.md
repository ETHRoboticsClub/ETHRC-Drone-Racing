# MAVSDK

**Page Owner:** [Saad Lahlou](https://github.com/saadl27)

---

# MAVSDK Overview

## What is MAVSDK?

**MAVSDK (Micro Air Vehicle Software Development Kit)** is a high-level, language-agnostic API designed to control and interact with MAVLink-compatible vehicles such as drones and ground robots. It provides an abstraction layer on top of MAVLink, allowing developers to communicate with an autopilot without dealing directly with low-level message handling.

MAVSDK supports multiple programming languages (C++, Python, Swift, Java) and exposes a plugin-based API for telemetry, offboard control, mission handling, parameter access, and system health monitoring.

In practice, **MAVSDK is the bridge between PX4 and our code**, running on the companion computer.

---

## PX4 Autopilot

**PX4** is an open-source autopilot system running on the flight controller hardware. It is responsible for estimation, control, safety logic, and actuator outputs, and exposes its interface via MAVLink.

In our stack, **we do not rely on PX4’s EKF nor on its internal cascaded PID controllers for nominal operation**. State estimation and control are handled externally by our own pipelines (e.g. MPC, RL).  
However, **PX4 remains available as a fallback**: its estimator and controllers can be re-enabled if needed for safety, debugging, or recovery.

We chose PX4 because it is:

- Highly **modular** and well-structured
- Supported by a rich ecosystem of tools:
  - Software-In-The-Loop (SITL) with Gazebo
  - Hardware-In-The-Loop (HITL)
- Backed by a large and active community
- Fully compatible with MAVSDK

While our long-term goal is to minimize dependence on PX4 for control and estimation, we keep it in the stack to **manage low-level flight infrastructure and safety-critical components**.

---

## MAVLink

**MAVLink** is a lightweight communication protocol used to exchange messages between:

- PX4 (flight controller)
- Companion computers
- Ground control stations (e.g. QGroundControl)

It defines standardized messages for telemetry, odometry, vehicle state, commands, parameters, and system status.  
MAVSDK abstracts this protocol and exposes a clean, robust API.

---

## MAVSDK in Our Architecture

In our architecture, **MAVSDK acts as the bridge between PX4 and our ROS 2-based codebase**.

A dedicated **MAVSDK client is embedded inside ROS 2**, allowing our autonomy, estimation, and control modules to communicate with the flight controller through a unified interface. MAVSDK handles all low-level communication, while our code focuses on high-level logic.

---

## Connecting to the Drone

The connection mechanism depends on the execution context:

- **PX4 SITL**  
  MAVSDK typically connects over **UDP** to a simulated PX4 instance (e.g. Gazebo).

- **Real hardware**  
  MAVSDK connects over **serial (USB/UART)** or **UDP**, depending on the companion computer and flight controller setup.

Once connected, MAVSDK manages system discovery, heartbeats, and link monitoring.

---

## Data Flow: PX4 → MAVSDK → ROS 2

Using the MAVSDK API, we retrieve:

- **Estimator outputs and telemetry**
  - PX4 odometry (position, velocity, orientation)
- **Vehicle status**
  - Arming state
  - Flight mode
  - System health and failsafe status

This data is forwarded to ROS 2 and consumed by higher-level modules.

---

## Command Flow and Control Philosophy

### Standard PX4 Control Path

By default, MAVSDK offboard commands rely on **PX4’s cascaded PID control architecture**. Commands such as body rates and collective thrust are processed by PX4 controllers before being converted into actuator outputs.

This mode is useful for safety and rapid prototyping, but it constrains full control authority.

---

### Direct Per-Motor Thrust Control

Our primary objective is to **bypass PX4’s internal control loops** and directly command the actuators.  
We therefore support **per-motor thrust commands**, enabling:

- Full control authority for MPC or RL policies
- Independence from PX4’s cascaded PID controllers
- Clear separation between control logic and hardware interfacing

In this setup, **MAVSDK is used purely as a helper for low-level communication and system management**, not as a control algorithm.

---

## Why We Keep MAVSDK and PX4

Even when bypassing PX4 controllers and estimation, we retain both components because they provide:

- Robust communication and system management (via MAVSDK)
- Access to telemetry, status, and safety mechanisms
- A reliable **plan B** using PX4’s native estimator and controllers if needed

In the long term, PX4 and MAVSDK may no longer be required, but for now we keep them to manage the interface layer handling low-level interaction with the vehicle.

---

In summary, **MAVSDK is the communication backbone between PX4 and our ROS 2 autonomy stack**, while PX4 serves as a modular, tool-rich, and safety-capable platform that we can fall back on when necessary.
