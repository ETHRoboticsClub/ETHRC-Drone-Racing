# Model Predictive Control (MPC)

**Page Owner:** [Mattia De Martino](), [Michalis Papadakis]()

---
This section documents the **Model Predictive Control (MPC)** implementation contained in the `mpc` folder.  
The MPC is implemented using **CasADi** for symbolic modeling and **ACADOS** for solving the nonlinear optimal control problem (OCP).

The controller computes **total thrust and body-rate commands**, which are later converted into MAVSDK offboard commands.

---

## Overview

The MPC implementation is structured into three main components:

1. **`drone_model.py`** – Drone dynamics and constraints
2. **`drone_opt.py`** – ACADOS optimal control problem and solver
3. **`drone_mpc_controller.py`** – High-level MPC controller interface

The execution flow is:

- Current state and reference are provided to the MPC controller
- The optimizer solves a finite-horizon OCP
- The first control input is extracted
- The command is converted to a MAVSDK-compatible `AttitudeRate`

---

## Drone Dynamics Model (`drone_model.py`)

This file defines the drone model used by the MPC and exports it in a format compatible with ACADOS.

### State Definition

The state vector has **13 dimensions**:

- Position `p` (3): NED frame  
- Velocity `v` (3): NED frame  
- Quaternion `q` (4): body → NED  
- Body angular rates `w` (3)

### Control Inputs

The control vector has **4 inputs**:

- Total thrust (N)
- Body rate commands: roll, pitch, yaw (rad/s)

---

### Dynamics

The model includes:

- **Position dynamics**  
  Position derivative equals velocity.

- **Translational dynamics**  
  Thrust is applied in the body frame and rotated to the NED frame using the quaternion.  
  Gravity is applied in the NED frame.  
  An optional drag term is present but set to zero.

- **Quaternion kinematics**  
  Quaternion derivative is computed from body angular rates.  
  The quaternion is normalized to avoid numerical drift.

- **Angular rate dynamics**  
  Angular rates follow a first-order tracking model:
    w_dot = (w_cmd - w) / tau_w


---

### Physical Parameters and Constraints

The model includes:
- Mass and inertia
- Maximum thrust derived from motor parameters
- Thrust limits
- Body rate limits
- Bounds on:
- Angular rates
- Velocity
- Position

All constraints are stored and later used by the optimizer.

---

### ACADOS Model Export

The model is exported with:
- Explicit and implicit dynamics
- State and control definitions
- Constraint information

This allows direct integration into an ACADOS OCP.

---

## MPC Optimizer (`drone_opt.py`)

This file defines and solves the nonlinear OCP using **ACADOS**.

### Horizon Setup

- Prediction horizon: `T`
- Number of nodes: `N`

---

### Cost Function

An **external nonlinear cost** is used.

#### Stage Cost

The stage cost penalizes:
- Position error
- Velocity error
- Quaternion error (Euclidean difference)
- Angular rate error
- Control effort (thrust deviation from hover and body rates)

#### Terminal Cost

The terminal cost penalizes:
- Position error
- Velocity error
- Quaternion error
- Angular rate error

---

### Parameters and References

The reference state is passed as a parameter to the solver.  
The optimizer supports:
- Point-to-point control
- Reference trajectory tracking

---

### Constraints

The optimizer enforces:
- Control bounds (thrust and body rates)
- State bounds on angular rates
- Fixed initial state at the beginning of each solve

---

### MPC Solve Step

At each control step:
1. The current state is normalized (quaternion)
2. References are set for all nodes
3. The solver is initialized (with warm start if available)
4. The OCP is solved
5. The first control input is extracted

If the solver fails, an exception is raised and logged.

---

## MPC Controller Interface (`drone_mpc_controller.py`)

This file provides the interface between the MPC and the rest of the system.

### Responsibilities

The controller:
- Initializes the drone model and optimizer
- Receives the current state estimate
- Receives a goal position or reference trajectory
- Calls the optimizer

---

### Control Output

The MPC outputs:
- Thrust in Newtons
- Body rates in rad/s

These are converted to:
- Normalized thrust in `[0, 1]`
- Body rates in degrees per second

The final output is a MAVSDK `AttitudeRate` command.

---

### Error Handling

If the optimizer fails:
- A safe hover command is returned
- Debug information can be logged for analysis

---

## Summary

The MPC module provides:
- A physically consistent drone model
- A constrained nonlinear MPC formulation
- A clear interface for integration with the flight stack

It is currently the main controller implemented in the codebase and serves as the reference control implementation.
