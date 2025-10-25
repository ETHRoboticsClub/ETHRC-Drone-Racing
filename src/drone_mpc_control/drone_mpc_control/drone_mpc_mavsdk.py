#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Drone MPC Controller using MAVSDK and ACADOS

This node implements Model Predictive Control (MPC) for a quadrotor drone.
It uses MAVSDK to communicate with PX4 autopilot.

Key Components:
1. MAVSDK: Direct Python interface to PX4 (simpler than MAVROS)
2. ACADOS: Fast optimization library for MPC
3. ROS 2: Used only for parameter management and logging


"""

PARAMS_TO_DISABLE = {
# RC & Offboard communication failsafes
"COM_RC_LOSS_T": 0,        # RC loss timeout
"COM_OBL_ACT": 0,          # Offboard loss action (0 = Disabled)
"COM_OF_LOSS_T": 0,        # Offboard loss timeout

# Navigation / Position failsafes
"NAV_DLL_ACT": 0,          # GPS failure
"NAV_RCL_ACT": 0,          # RC lost action
"NAV_ACC_RAD": 10.0,       # Looser acceptance radius (optional)

# Geofence
"GF_ACTION": 0,            # Disable geofence action
"GF_MAX_HOR_DIST": 0,      # No horizontal fence
"GF_MAX_VER_DIST": 0,      # No vertical fence

# Battery failsafes
"COM_LOW_BAT_ACT": 0,      # Disable low battery failsafe
"BAT_CRIT_THR": 0.0,       # No critical threshold
"BAT_EMERGEN_THR": 0.0,    # No emergency threshold

# Data link loss
"COM_DL_LOSS_T": 0,        # Data link loss timeout
"COM_DL_LOSS_EN": 0,       # Disable data link loss failsafe

# Mission failure actions
"MIS_FAIL_ACT": 0,         # Mission failure action disabled

# RTL & Return-to-home triggers
"RTL_RETURN_ALT": 0,       # Minimum return altitude
"RTL_DESCEND_ALT": 0,      # No descend safety
"RTL_LAND_DELAY": -1,      # Disable auto-land after RTL
}

# ============================================================================
# IMPORTS
# ============================================================================

# Standard Python libraries
import os
import sys
import asyncio
import numpy as np
import timeit
from threading import Lock, Thread
from dataclasses import dataclass
from typing import Optional, Tuple

# ROS 2 imports (only for node structure and parameters)
import rclpy
from rclpy.node import Node

# MAVSDK imports - this is our PX4 interface
from mavsdk import System
from mavsdk.offboard import (
    OffboardError,
    AttitudeRate,
    PositionNedYaw
)
from mavsdk.telemetry import LandedState

# ACADOS imports - for MPC optimization
from .drone_model import DroneModel
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
import scipy.linalg


# ============================================================================
# DATA STRUCTURES
# ============================================================================

@dataclass
class DroneState:
    """
    Represents the complete state of the drone.

    This matches the 13-state vector used by ACADOS:
    - position: [x, y, z] in NED frame (North-East-Down)
    - velocity: [vx, vy, vz] in NED frame
    - quaternion: [qw, qx, qy, qz] representing orientation
    - angular_velocity: [wx, wy, wz] in body frame (rad/s)
    """
    position: np.ndarray = np.zeros(3)          # [x, y, z] meters
    velocity: np.ndarray = np.zeros(3)          # [vx, vy, vz] m/s
    quaternion: np.ndarray = np.array([1, 0, 0, 0])  # [qw, qx, qy, qz]
    angular_velocity: np.ndarray = np.zeros(3)  # [wx, wy, wz] rad/s

    def to_vector(self) -> np.ndarray:
        """Convert to 13-element state vector for ACADOS."""
        return np.concatenate([
            self.position,
            self.velocity,
            self.quaternion,
            self.angular_velocity
        ])

    def is_valid(self) -> bool:
        """Check if state contains valid (non-NaN) values."""
        vec = self.to_vector()
        return not np.any(np.isnan(vec)) and not np.any(np.isinf(vec))


# ============================================================================
# MAIN CONTROLLER CLASS
# ============================================================================

class DroneMPCController(Node):
    """
    Model Predictive Controller for quadrotor drone using MAVSDK.

    This class:
    1. Connects to PX4 via MAVSDK
    2. Gets drone state (position, velocity, orientation)
    3. Solves MPC optimization problem using ACADOS
    4. Sends control commands (thrust + body rates) to PX4

    The MPC runs at 50 Hz by default and predicts 1 second into the future.
    """

    def __init__(self):
        """
        Initialize the MPC controller.

        Steps:
        1. Initialize ROS 2 node (for parameters and logging)
        2. Load parameters from YAML files
        3. Setup ACADOS MPC solver
        4. Initialize MAVSDK system
        5. Start async event loop in separate thread
        """
        # Initialize ROS 2 node - we use this mainly for parameters and logging
        super().__init__('drone_mpc_controller')

        # ====================================================================
        # DECLARE ROS 2 PARAMETERS
        # ====================================================================
        # These can be set via YAML files or launch arguments

        # MPC Parameters
        self.declare_parameter('control_rate', 50)      # How fast to run MPC (Hz)
        self.declare_parameter('t_horizon', 1.0)        # How far to predict (seconds)
        self.declare_parameter('n_nodes', 20)           # Prediction steps

        # Cost function weights - how much to penalize tracking errors
        self.declare_parameter('weights.position', [200.0, 200.0, 500.0])  # [x, y, z]
        self.declare_parameter('weights.velocity', [1.0, 1.0, 1.0])
        self.declare_parameter('weights.quaternion', [5.0, 5.0, 200.0, 1.0])
        self.declare_parameter('weights.angular_velocity', [1.0, 1.0, 1.0])
        self.declare_parameter('weights.control', [6.0, 6.0, 6.0, 6.0])  # Control effort

        # Target position (where we want the drone to go)
        self.declare_parameter('target.x', 0.0)
        self.declare_parameter('target.y', 0.0)
        self.declare_parameter('target.z', -2.0)  # Negative because NED frame (down is positive)

        # Drone physical parameters (defaults match drone_params.yaml)
        self.declare_parameter('mass', 0.5)  # kg - must match drone_model.py
        self.declare_parameter('gravity', 9.8)  # m/s^2
        self.declare_parameter('inertia.I_xx', 0.00232)  # kg*m^2 - must match drone_model.py
        self.declare_parameter('inertia.I_yy', 0.00232)
        self.declare_parameter('inertia.I_zz', 0.00400)

        # MAVSDK connection
        self.declare_parameter('mavsdk_port', 14540)  # PX4 SITL default port
        self.declare_parameter('connection_string', 'udpin://0.0.0.0:14540')  # Connection type

        # GET parameters
        self.control_rate = self.get_parameter('control_rate').value
        self.t_horizon = self.get_parameter('t_horizon').value
        self.n_nodes = self.get_parameter('n_nodes').value

        # ====================================================================
        # STATE VARIABLES
        # ====================================================================

        # Current drone state (updated by MAVSDK telemetry)
        self.current_state = DroneState()
        self.state_lock = Lock()  # Thread-safe access to state

        # Target state (where we want to be)
        target_x = self.get_parameter('target.x').value
        target_y = self.get_parameter('target.y').value
        target_z = self.get_parameter('target.z').value
        self.target_state = DroneState()
        self.target_state.position = np.array([target_x, target_y, target_z])
        self.target_state.quaternion = np.array([1, 0, 0, 0])  # Level orientation

        # Connection status
        self.is_connected = False
        self.is_armed = False
        self.is_offboard = False
        self.health_ok = False  # Track if position estimate is ready
        self.arm_attempted = False  # Track if we've tried to arm

        # ====================================================================
        # SETUP ACADOS MPC SOLVER
        # ====================================================================

        self.get_logger().info("=" * 70)
        self.get_logger().info("Initializing ACADOS MPC Solver...")
        self.get_logger().info("This will compile C code - first time takes 5-10 seconds")
        self.get_logger().info("=" * 70)

        try:
            self.setup_acados()
            self.get_logger().info("✓ ACADOS solver initialized successfully!")
        except Exception as e:
            self.get_logger().error(f"✗ ACADOS initialization failed: {e}")
            raise

        # ====================================================================
        # SETUP MAVSDK
        # ====================================================================

        self.get_logger().info("Initializing MAVSDK connection...")

        # Create MAVSDK System object
        self.drone = System()


        # Connection string for MAVSDK (PX4 SITL uses udpin://0.0.0.0:14540)
        # IMPORTANT: Use 'udpin' (UDP input) to receive data from PX4 SITL
        self.connection_string = self.get_parameter('connection_string').value

        # Event loop for async operations (MAVSDK is async-based)
        self.loop = asyncio.new_event_loop()

        # Start event loop in separate thread so it doesn't block ROS
        self.async_thread = Thread(target=self._run_async_loop, daemon=True)
        self.async_thread.start()

        # Start MAVSDK connection and telemetry (async)
        asyncio.run_coroutine_threadsafe(
            self._initialize_mavsdk(),
            self.loop
        )

        # ====================================================================
        # ROS 2 TIMER FOR CONTROL LOOP
        # ====================================================================

        # Main control loop runs at specified rate
        self.control_timer = self.create_timer(
            1.0 / self.control_rate,
            self.control_loop_callback
        )

        # Statistics tracking
        self.solve_times = []
        self.iteration_count = 0

        self.get_logger().info("=" * 70)
        self.get_logger().info("✓ Drone MPC Controller Initialized!")
        self.get_logger().info(f"  Control rate: {self.control_rate} Hz")
        self.get_logger().info(f"  MPC horizon: {self.t_horizon} seconds")
        self.get_logger().info(f"  Target position: [{target_x:.2f}, {target_y:.2f}, {target_z:.2f}]")
        self.get_logger().info("=" * 70)




    async def set_parameters(self):
        for param, value in PARAMS_TO_DISABLE.items():
            try:
                if isinstance(value, int):
                    await self.drone.param.set_param_int(param, value)
                else:
                    await self.drone.param.set_param_float(param, value)
                print(f"✅ Set {param} = {value}")
            except Exception as e:
                print(f"⚠️ Could not set {param}: {e}")

    # ========================================================================
    # ACADOS SETUP
    # ========================================================================

    def setup_acados(self):
        """
        Setup the ACADOS MPC solver.

        ACADOS is a fast optimization library that solves the MPC problem:
        - Predict drone motion over time horizon
        - Find optimal control inputs (thrust + body rates)
        - Minimize tracking error while respecting constraints

        This function:
        1. Creates the drone dynamics model
        2. Defines cost function (what to minimize)
        3. Sets constraints (max thrust, max angular velocity, etc.)
        4. Compiles the solver (generates C code for speed)
        """

        # Create drone dynamics model (defines how drone moves)
        drone_model_obj = DroneModel()
        model = drone_model_obj.model
        constraint = drone_model_obj.constraint

        self.T = self.t_horizon  # Time horizon
        self.N = self.n_nodes    # Number of prediction nodes

        # Working directory for ACADOS generated code
        package_dir = os.path.join(
            os.path.expanduser('~'),
            'drone',
            'drone_mpc_ros2',
            'src',
            'drone_mpc_control'
        )
        self.acados_models_dir = os.path.join(package_dir, 'c_generated_code')
        if not os.path.exists(self.acados_models_dir):
            os.makedirs(self.acados_models_dir)
        os.chdir(package_dir)

        # ACADOS installation path
        acados_source_path = os.environ.get(
            'ACADOS_SOURCE_DIR',
            os.path.expanduser('~/acados')
        )

        # State and control dimensions
        nx = model.x.size()[0]  # 13 states (position, velocity, quaternion, angular_velocity)
        nu = model.u.size()[0]  # 4 controls (thrust T, moments Mx, My, Mz)
        ny = nx + nu            # Output dimension for cost function
        n_params = len(model.p)  # Model parameters

        self.nx = nx
        self.nu = nu

        # Get weight parameters from ROS parameters
        weights_pos = self.get_parameter('weights.position').value
        weights_vel = self.get_parameter('weights.velocity').value
        weights_quat = self.get_parameter('weights.quaternion').value
        weights_ang_vel = self.get_parameter('weights.angular_velocity').value
        weights_control = self.get_parameter('weights.control').value

        # ====================================================================
        # CREATE OPTIMAL CONTROL PROBLEM (OCP)
        # ====================================================================

        ocp = AcadosOcp()
        ocp.acados_include_path = acados_source_path + '/include'
        ocp.acados_lib_path = acados_source_path + '/lib'
        ocp.model = model
        ocp.dims.N = self.N
        ocp.solver_options.tf = self.T

        # Model parameters
        ocp.dims.np = n_params
        ocp.parameter_values = np.zeros(n_params)

        # ====================================================================
        # COST FUNCTION
        # ====================================================================
        # We want to minimize: state_error + control_effort
        # Q matrix: weights for state errors
        # R matrix: weights for control effort

        Q = np.diag(weights_pos + weights_vel + weights_quat + weights_ang_vel)
        R = np.diag(weights_control)

        # Linear least squares cost: minimize ||Vx*x + Vu*u - y_ref||^2
        ocp.cost.cost_type = 'LINEAR_LS'
        ocp.cost.cost_type_e = 'LINEAR_LS'  # Terminal cost
        ocp.cost.W = scipy.linalg.block_diag(Q, R)  # Stage cost weight
        ocp.cost.W_e = Q  # Terminal cost weight

        # Vx: maps states to outputs
        ocp.cost.Vx = np.zeros((ny, nx))
        ocp.cost.Vx[:nx, :nx] = np.eye(nx)
        ocp.cost.Vx_e = np.eye(nx)

        # Vu: maps controls to outputs
        ocp.cost.Vu = np.zeros((ny, nu))
        ocp.cost.Vu[nx:, :] = np.eye(nu)

        # ====================================================================
        # CONSTRAINTS
        # ====================================================================

        # Control input constraints (thrust and moments)
        ocp.constraints.lbu = np.concatenate((
            np.array([constraint.T_min]),  # Min thrust
            constraint.M_min                # Min moments
        ))
        ocp.constraints.ubu = np.concatenate((
            np.array([constraint.T_max]),  # Max thrust
            constraint.M_max                # Max moments
        ))
        ocp.constraints.idxbu = np.array(range(nu))

        # State constraints (angular velocity limits)
        ocp.constraints.lbx = constraint.w_min
        ocp.constraints.ubx = constraint.w_max
        ocp.constraints.idxbx = np.array([10, 11, 12])  # Indices of angular velocity

        # Initial state constraint (will be updated each iteration)
        x_init = np.zeros(nx)
        x_init[6] = 1.0  # Quaternion w component
        ocp.constraints.x0 = x_init

        # Reference trajectory
        x_ref = np.zeros(nx)
        x_ref[6] = 1.0
        u_ref = np.zeros(nu)
        ocp.cost.yref = np.concatenate((x_ref, u_ref))
        ocp.cost.yref_e = x_ref

        # ====================================================================
        # SOLVER OPTIONS
        # ====================================================================

        ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        ocp.solver_options.integrator_type = 'ERK'  # Explicit Runge-Kutta
        ocp.solver_options.print_level = 0
        ocp.solver_options.nlp_solver_type = 'SQP_RTI'  # Real-Time Iteration

        # Generate and compile solver
        json_file = os.path.join(package_dir, model.name + '_acados_ocp.json')
        self.solver = AcadosOcpSolver(ocp, json_file=json_file)
        self.integrator = AcadosSimSolver(ocp, json_file=json_file)

    # ========================================================================
    # MAVSDK ASYNC METHODS
    # ========================================================================

    def _run_async_loop(self):
        """
        Run the asyncio event loop in a separate thread.

        MAVSDK uses async/await, so we need an event loop.
        This runs in a separate thread so it doesn't block ROS.
        """
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    async def _initialize_mavsdk(self):
        """
        Initialize MAVSDK connection to PX4.

        Steps:
        1. Connect to PX4 (via UDP)
        2. Wait for drone to be discovered
        3. Wait for health check (position estimate ready)
        4. Start telemetry streams (position, velocity, attitude, etc.)
        """
        try:
            # Connect to PX4
            # IMPORTANT: Use 'udpin' (UDP input) for receiving from PX4 SITL!
            self.get_logger().info(f"Connecting to PX4 at {self.connection_string}...")
            self.get_logger().info("Make sure PX4 SITL is running first!")
            await self.drone.connect(system_address=self.connection_string)

            # Wait for drone to be discovered
            self.get_logger().info("Waiting for drone to connect...")
            async for state in self.drone.core.connection_state():
                if state.is_connected:
                    self.get_logger().info("✓ Drone connected!")
                    self.is_connected = True
                    break

            # Wait for position estimate to be ready (CRITICAL!)
            # Without this, PX4 will reject arming and offboard commands
            self.get_logger().info("Waiting for position estimate...")
            async for health in self.drone.telemetry.health():
                if health.is_global_position_ok and health.is_home_position_ok:
                    self.get_logger().info("✓ Position estimate OK!")
                    self.health_ok = True
                    break

            # Start telemetry streams
            self.get_logger().info("Starting telemetry streams...")
            await self._start_telemetry()
            self.get_logger().info("✓ Telemetry streams started")

            # Start monitoring tasks
            asyncio.create_task(self._monitor_connection())
            asyncio.create_task(self._monitor_armed_state())
            asyncio.create_task(self._monitor_health())

        except Exception as e:
            self.get_logger().error(f"MAVSDK initialization failed: {e}")

    async def _start_telemetry(self):
        """
        Start all telemetry streams from PX4.

        We subscribe to:
        - Position (NED frame)
        - Velocity (NED frame)
        - Attitude (quaternion)
        - Angular velocity (body rates)

        IMPORTANT: Each telemetry stream must run as a separate task!
        If we use sequential 'async for' loops, only the first one runs.
        """
        # Start each telemetry stream as a separate task
        asyncio.create_task(self._subscribe_position_velocity())
        asyncio.create_task(self._subscribe_attitude())
        asyncio.create_task(self._subscribe_angular_velocity())

    async def _subscribe_position_velocity(self):
        """Subscribe to position and velocity telemetry."""
        async for position_velocity in self.drone.telemetry.position_velocity_ned():
            with self.state_lock:
                # Position in NED frame
                self.current_state.position = np.array([
                    position_velocity.position.north_m,
                    position_velocity.position.east_m,
                    position_velocity.position.down_m
                ])
                # Velocity in NED frame
                self.current_state.velocity = np.array([
                    position_velocity.velocity.north_m_s,
                    position_velocity.velocity.east_m_s,
                    position_velocity.velocity.down_m_s
                ])

    async def _subscribe_attitude(self):
        """Subscribe to attitude (quaternion) telemetry."""
        async for attitude_quaternion in self.drone.telemetry.attitude_quaternion():
            with self.state_lock:
                self.current_state.quaternion = np.array([
                    attitude_quaternion.w,
                    attitude_quaternion.x,
                    attitude_quaternion.y,
                    attitude_quaternion.z
                ])

    async def _subscribe_angular_velocity(self):
        """Subscribe to angular velocity telemetry."""
        async for attitude_angular_velocity in self.drone.telemetry.attitude_angular_velocity_body():
            with self.state_lock:
                self.current_state.angular_velocity = np.array([
                    attitude_angular_velocity.roll_rad_s,
                    attitude_angular_velocity.pitch_rad_s,
                    attitude_angular_velocity.yaw_rad_s
                ])

    async def _monitor_connection(self):
        """Monitor connection status."""
        async for state in self.drone.core.connection_state():
            self.is_connected = state.is_connected
            if not state.is_connected:
                self.get_logger().warn("Drone disconnected!")

    async def _monitor_armed_state(self):
        """Monitor armed status."""
        async for is_armed in self.drone.telemetry.armed():
            self.is_armed = is_armed

    async def _monitor_health(self):
        """Monitor health status (position estimate, etc.)."""
        async for health in self.drone.telemetry.health():
            self.health_ok = health.is_global_position_ok and health.is_home_position_ok

    async def _arm_and_takeoff(self):
        """
        Arm the drone and start offboard mode.

        CRITICAL: Correct sequence according to MAVSDK documentation:
        1. ARM FIRST (before offboard mode!)
        2. Send initial setpoint (required before offboard)
        3. Start offboard mode

        The old sequence (offboard then arm) causes the drone not to respond!
        """
        try:
            # Step 1: ARM FIRST! (This is critical!)
            self.get_logger().info("Arming drone...")
            await self.drone.action.arm()
            self.get_logger().info("✓ Drone armed")

            # Step 2: Set initial setpoint (required before starting offboard)
            # Start with current position to avoid sudden jumps
            initial_setpoint = AttitudeRate(
                0.0,  # Start from origin
                0.0,
                0.0,
                0.0   # yaw
            )

            self.get_logger().info("Setting initial setpoint...")
            for i in range(10):  # Send multiple times to ensure PX4 receives it
                await self.drone.offboard.set_attitude_rate(initial_setpoint)
                await asyncio.sleep(0.1)

            #await asyncio.sleep(1)  # Wait a moment

            # Step 3: Start offboard mode
            self.get_logger().info("Starting offboard mode...")
            await self.drone.offboard.start()
            self.is_offboard = True
            self.get_logger().info("✓ Offboard mode started - Ready to fly!")

        except OffboardError as e:
            self.get_logger().error(f"Offboard mode failed: {e._result.result}")
            self.get_logger().error("Trying to disarm...")
            try:
                await self.drone.action.disarm()
            except:
                pass
        except Exception as e:
            self.get_logger().error(f"Arming/Offboard failed: {e}")
            self.get_logger().error("Trying to disarm...")
            try:
                await self.drone.action.disarm()
            except:
                pass

    async def _send_attitude_rate_command(self, thrust: float, body_rates: np.ndarray):
        """
        Send attitude rate command to PX4.

        Args:
            thrust: Normalized thrust [0-1]
            body_rates: [roll_rate, pitch_rate, yaw_rate] in rad/s
        """
        try:
            attitude_rate = AttitudeRate(
                roll_deg_s=float(np.degrees(body_rates[0])),
                pitch_deg_s=float(np.degrees(body_rates[1])),
                yaw_deg_s=float(np.degrees(body_rates[2])),
                thrust_value=float(thrust)
            )
            await self.drone.offboard.set_attitude_rate(attitude_rate)
        except Exception as e:
            self.get_logger().error(f"Failed to send attitude rate: {e}")

    # ========================================================================
    # MPC SOLVING
    # ========================================================================

    def solve_mpc(self) -> Tuple[Optional[np.ndarray], float]:
        """
        Solve the MPC optimization problem.

        Returns:
            (control_input, solve_time) where:
            - control_input: [thrust, Mx, My, Mz] or None if failed
            - solve_time: time taken to solve (seconds)
        """
        with self.state_lock:
            x_current = self.current_state.to_vector()

        # Check for valid state
        if not self.current_state.is_valid():
            self.get_logger().warn("Invalid state (NaN/Inf), skipping MPC solve")
            return None, 0.0

        # Set current state as initial constraint
        self.solver.set(0, 'lbx', x_current)
        self.solver.set(0, 'ubx', x_current)

        # Set reference trajectory (constant setpoint)
        target_ref = self.target_state.to_vector()
        u_ref = np.zeros(self.nu)
        y_ref = np.concatenate((target_ref, u_ref))

        for i in range(self.N):
            self.solver.set(i, 'yref', y_ref)
        self.solver.set(self.N, 'yref', target_ref)

        # Solve
        start_time = timeit.default_timer()
        status = self.solver.solve()
        solve_time = timeit.default_timer() - start_time

        if status != 0:
            self.get_logger().warn(f"MPC solver returned status {status}")
            return None, solve_time

        # Get optimal control
        u_optimal = self.solver.get(0, 'u')

        return u_optimal, solve_time

    # ========================================================================
    # CONTROL LOOP
    # ========================================================================

    def control_loop_callback(self):
        """
        Main control loop (called by ROS timer).

        This runs at control_rate Hz and:
        1. Checks connection status
        2. Checks health status (position estimate)
        3. Arms and starts offboard if needed
        4. Solves MPC
        5. Sends control commands to PX4
        6. Logs statistics
        """
        # Wait for connection
        if not self.is_connected:
            if self.iteration_count % 50 == 0:  # Log every second at 50Hz
                self.get_logger().info("Waiting for PX4 connection...")
            self.iteration_count += 1
            return

        # Wait for health check (position estimate)
        if not self.health_ok:
            if self.iteration_count % 50 == 0:
                self.get_logger().info("Waiting for position estimate to be ready...")
            self.iteration_count += 1
            return

        # Check if we have valid state data
        with self.state_lock:
            has_valid_position = not np.all(self.current_state.position == 0)
            position_str = f"[{self.current_state.position[0]:.2f}, {self.current_state.position[1]:.2f}, {self.current_state.position[2]:.2f}]"
            velocity_str = f"[{self.current_state.velocity[0]:.2f}, {self.current_state.velocity[1]:.2f}, {self.current_state.velocity[2]:.2f}]"
            quaternion_str = f"[{self.current_state.quaternion[0]:.2f}, {self.current_state.quaternion[1]:.2f}, {self.current_state.quaternion[2]:.2f}, {self.current_state.quaternion[3]:.2f}]"

        if not has_valid_position:
            if self.iteration_count % 50 == 0:
                self.get_logger().info("Waiting for valid telemetry data...")
            self.iteration_count += 1
            return

        # Start offboard mode and arm if not already
        if not self.is_offboard:
            if self.iteration_count % 50 == 0:
                self.get_logger().info("=" * 70)
                self.get_logger().info("TELEMETRY RECEIVED:")
                self.get_logger().info(f"  Position (NED): {position_str} m")
                self.get_logger().info(f"  Velocity (NED): {velocity_str} m/s")
                self.get_logger().info(f"  Attitude (quat): {quaternion_str}")
                self.get_logger().info("=" * 70)

                if not self.arm_attempted:
                    self.get_logger().info("Initiating arming and offboard mode...")

            # Only try to arm once (use flag instead of iteration count)
            if not self.arm_attempted:
                self.arm_attempted = True
                asyncio.run_coroutine_threadsafe(
                    self._arm_and_takeoff(),
                    self.loop
                )
            self.iteration_count += 1
            return

        # Solve MPC
        u_optimal, solve_time = self.solve_mpc()
        self.solve_times.append(solve_time)

        if u_optimal is None:
            return

        # Convert MPC output to PX4 commands
        # u_optimal = [thrust, Mx, My, Mz]
        thrust_raw = u_optimal[0]
        moments = u_optimal[1:4]

        # Normalize thrust to [0, 1]
        mass = self.get_parameter('mass').value
        g = self.get_parameter('gravity').value
        hover_thrust = mass * g
        thrust_normalized = np.clip((thrust_raw + hover_thrust) / (2 * hover_thrust), 0.0, 1.0)

        # Convert moments to body rates (simplified)
        I_xx = self.get_parameter('inertia.I_xx').value
        I_yy = self.get_parameter('inertia.I_yy').value
        I_zz = self.get_parameter('inertia.I_zz').value
        I_inv = np.diag([1/I_xx, 1/I_yy, 1/I_zz])

        with self.state_lock:
            w_current = self.current_state.angular_velocity

        # Desired angular acceleration
        w_dot = I_inv @ moments
        dt = 1.0 / self.control_rate
        w_desired = w_current + w_dot * dt
        w_desired = np.zeros_like(w_desired) 
        # Send command to PX4
        asyncio.run_coroutine_threadsafe(
            self._send_attitude_rate_command(0.5 , w_desired),
            self.loop
        )

        # Print statistics every 50 iterations (~1 second at 50Hz) for better visibility
        self.iteration_count += 1
        if self.iteration_count >= 50:
            avg_time = np.mean(self.solve_times) if self.solve_times else 0
            max_time = np.max(self.solve_times) if self.solve_times else 0

            with self.state_lock:
                pos_error = np.linalg.norm(
                    self.current_state.position - self.target_state.position
                )
                current_pos = self.current_state.position
                current_vel = self.current_state.velocity
                target_pos = self.target_state.position

            self.get_logger().info("=" * 70)
            self.get_logger().info(f"Current pos: [{current_pos[0]:6.2f}, {current_pos[1]:6.2f}, {current_pos[2]:6.2f}]m (NED)")
            self.get_logger().info(f"Current vel: [{current_vel[0]:6.2f}, {current_vel[1]:6.2f}, {current_vel[2]:6.2f}]m/s")
            self.get_logger().info(f"Target pos:  [{target_pos[0]:6.2f}, {target_pos[1]:6.2f}, {target_pos[2]:6.2f}]m (NED)")
            self.get_logger().info(f"ERROR: {pos_error:.3f}m")
            self.get_logger().info(f"MPC: Avg={avg_time*1000:.2f}ms, Max={max_time*1000:.2f}ms")
            self.get_logger().info(f"CMD: Thrust={thrust_normalized:.3f}, Rates=[{np.degrees(w_desired[0]):5.1f}, {np.degrees(w_desired[1]):5.1f}, {np.degrees(w_desired[2]):5.1f}]°/s")
            self.get_logger().info("=" * 70)

            self.solve_times = []
            self.iteration_count = 0

    def destroy_node(self):
        """Cleanup when node is destroyed."""
        # Stop offboard mode
        if self.is_offboard:
            asyncio.run_coroutine_threadsafe(
                self.drone.offboard.stop(),
                self.loop
            )

        # Stop event loop
        self.loop.call_soon_threadsafe(self.loop.stop)

        super().destroy_node()


# ============================================================================
# MAIN FUNCTION
# ============================================================================

def main(args=None):
    """
    Main entry point for the MPC controller node.

    This is called when you run: ros2 run drone_mpc_control drone_mpc_mavsdk
    """
    # Initialize ROS 2
    rclpy.init(args=args)

    try:
        # Create controller
        controller = DroneMPCController()
        # await controller.set_parameters()

        # Spin (keep node alive)
        rclpy.spin(controller)

    except KeyboardInterrupt:
        print("\nShutting down...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Cleanup
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
