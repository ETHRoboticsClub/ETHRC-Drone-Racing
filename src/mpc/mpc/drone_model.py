#!/usr/bin/env python
"""
Drone dynamics model for MPC using CasADi and ACADOS.
State: position(3), velocity(3), quaternion(4), angular_rates(3) = 13 states
Control: thrust(1), body_rates(3) = 4 controls
Uses full quadrotor dynamics with thrust vector and body rates.
"""
import numpy as np
import casadi as ca
from acados_template import AcadosModel
from .airframe import load_boreas_airframe

class DroneModel(object):
    def __init__(self):
        model = AcadosModel()
        constraint = ca.types.SimpleNamespace()
        airframe = load_boreas_airframe()
        
        # Boreas parameters shared with the Betaflight SITL plant.
        self.I_xx, self.I_yy, self.I_zz = airframe.inertia_kg_m2
        self.m = airframe.mass_kg
        self.g = 9.81  # m/s^2
        self.I = np.diag([self.I_xx, self.I_yy, self.I_zz])

        self.maxThrust = airframe.maximum_thrust_n
        self.thrust_curve_per_motor_n = airframe.thrust_curve_per_motor_n
        self.calibration_status = airframe.calibration_status
        
        # Aerodynamic drag coefficient
        self.k_drag = float(sum(airframe.translational_drag[:2]) / 2.0)
        
        # Time constant for body rate tracking (first-order dynamics)
        # Use more conservative value for numerical stability
        self.tau_w = 0.1  # s (increased for better conditioning)
        
        # ============================================
        # Control inputs: thrust and body rate commands
        # ============================================
        thrust_cmd = ca.MX.sym('thrust', 1)  # N (total thrust magnitude)
        wx_cmd = ca.MX.sym('wx_cmd', 1)      # rad/s (roll rate command)
        wy_cmd = ca.MX.sym('wy_cmd', 1)      # rad/s (pitch rate command)
        wz_cmd = ca.MX.sym('wz_cmd', 1)      # rad/s (yaw rate command)
        
        controls = ca.vertcat(thrust_cmd, wx_cmd, wy_cmd, wz_cmd)
        
        # ============================================
        # State vector (13 states total)
        # ============================================
        p = ca.MX.sym('p', 3)    # position NED [north, east, down]
        v = ca.MX.sym('v', 3)    # velocity NED [vn, ve, vd]
        q = ca.MX.sym('q', 4)    # quaternion [w,x,y,z] body->NED
        w = ca.MX.sym('w', 3)    # angular rates body [wx, wy, wz]
        
        states = ca.vertcat(p, v, q, w)
        
        # ============================================
        # System dynamics
        # ============================================
        
        # 1. Position dynamics: p_dot = v
        p_dot = v
        
        # 2. Velocity dynamics: m*v_dot = R*F_thrust - m*g - F_drag
        # Rotation matrix from body to NED frame using quaternion
        qw, qx, qy, qz = q[0], q[1], q[2], q[3]
        
        # Normalize quaternion to prevent numerical drift
        q_norm = ca.sqrt(qw**2 + qx**2 + qy**2 + qz**2 + 1e-8)  # Add small epsilon
        qw, qx, qy, qz = qw/q_norm, qx/q_norm, qy/q_norm, qz/q_norm
        
        # Thrust vector in body frame (pointing up = negative z_body in NED convention)
        # La thrust punta lungo +z_body, che corrisponde alla direzione "up" del drone
        F_body = ca.vertcat(0, 0, thrust_cmd)
        
        # Rotation matrix body -> NED (from quaternion)
        R_bn = ca.vertcat(
            ca.horzcat(
                1 - 2*(qy**2 + qz**2),
                2*(qx*qy - qw*qz),
                2*(qx*qz + qw*qy)
            ),
            ca.horzcat(
                2*(qx*qy + qw*qz),
                1 - 2*(qx**2 + qz**2),
                2*(qy*qz - qw*qx)
            ),
            ca.horzcat(
                2*(qx*qz - qw*qy),
                2*(qy*qz + qw*qx),
                1 - 2*(qx**2 + qy**2)
            )
        )
        
        # Thrust force in NED frame
        F_thrust_ned = R_bn @ F_body
        
        # Gravity in NED frame (positive down)
        g_ned = ca.vertcat(0, 0, self.g)
        
        # Aerodynamic drag (opposing velocity)
        v_norm = ca.sqrt(ca.dot(v, v) + 1e-6)  # Add small epsilon for smoothness
        F_drag = self.k_drag * v_norm * v
        
        # Acceleration
        a = (-F_thrust_ned + self.m * g_ned - F_drag) / self.m
        v_dot = a
        
        # 3. Quaternion kinematics: q_dot = 0.5 * q ⊗ [0, w]
        q_dot = 0.5 * ca.vertcat(
            -w[0]*qx - w[1]*qy - w[2]*qz,
             w[0]*qw + w[2]*qy - w[1]*qz,
             w[1]*qw - w[2]*qx + w[0]*qz,
             w[2]*qw + w[1]*qx - w[0]*qy
        )
        
        # 4. Angular rate dynamics (first-order tracking)
        # Per body rates istantanei: w_dot = (w_cmd - w) / tau_w con tau_w molto piccolo
        # Oppure per dinamica più realistica basata su inerzia:
        # I*w_dot = M - w × (I*w), dove M sono i momenti comandati
        
        w_cmd = ca.vertcat(wx_cmd, wy_cmd, wz_cmd)
        w_dot = (w_cmd - w) / self.tau_w
        
        # Concatenate all derivatives
        f_expression = ca.vertcat(p_dot, v_dot, q_dot, w_dot)
        
        # ============================================
        # ACADOS model setup
        # ============================================
        f = ca.Function('f', [states, controls], [f_expression], 
                       ['state', 'control_input'], ['xdot'])
        
        x_dot = ca.MX.sym('x_dot', states.size1())
        f_impl = x_dot - f_expression
        
        model.f_expl_expr = f_expression
        model.f_impl_expr = f_impl
        model.x = states
        model.xdot = x_dot
        model.u = controls
        model.p = []
        model.name = 'drone_thrust'
        
        # ============================================
        # Physical constraints
        # ============================================
        # Thrust limits 
        constraint.thrust_min = 0 
        constraint.thrust_max = self.maxThrust  # N
        
        # Body rate limits (rad/s)
        constraint.w_max_xy = 50.0   # roll/pitch rates
        constraint.w_max_yaw = 50.0  # yaw rate
        
        # Control input bounds
        constraint.u_min = np.array([constraint.thrust_min, 
                                     -constraint.w_max_xy,
                                     -constraint.w_max_xy,
                                     -constraint.w_max_yaw])
        constraint.u_max = np.array([constraint.thrust_max,
                                     constraint.w_max_xy,
                                     constraint.w_max_xy,
                                     constraint.w_max_yaw])
        
        # State bounds (for angular rates in state)
        constraint.w_state_max = np.array([np.pi, np.pi, np.pi])  # rad/s
        constraint.w_state_min = -constraint.w_state_max
        
        # Velocity bounds (safety limits)
        constraint.v_max = np.array([10.0, 10.0, 10.0])  # m/s 
        constraint.v_min = -constraint.v_max
        constraint.p_min = np.array([-50.0, -50.0, -100.0])  # [north, east, down] 
        constraint.p_max = np.array([ 50.0,  50.0,   0.0])
        
        self.model = model
        self.constraint = constraint
        self.f = f
        
        print(f"DroneModel initialized: nx={states.size1()}, nu={controls.size1()}")
        print(f"  Mass: {self.m} kg")
        print(f"  Thrust limits: [{constraint.thrust_min:.1f}, {constraint.thrust_max:.1f}] N")
        print(f"  Body rate limits: ±{constraint.w_max_xy:.1f} rad/s (roll/pitch), "
              f"±{constraint.w_max_yaw:.1f} rad/s (yaw)")
