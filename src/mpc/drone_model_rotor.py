#!/usr/bin/env python
"""
Drone dynamics model with per-rotor thrust inputs and full rigid body dynamics.
State: position(3), velocity(3), quaternion(4), angular_rates(3) = 13 states
Control: per-rotor normalized thrust(4) = 4 controls, u_i in [0, 1]
Uses Euler's equations for angular dynamics instead of first-order rate tracking.

Rotor geometry from PX4 X500 airframe (4001_gz_x500), FRD body frame:
  Rotor 0: PX=+0.13  PY=+0.22  KM=+0.05  rear-right,  CCW
  Rotor 1: PX=-0.13  PY=-0.20  KM=+0.05  front-left,  CCW
  Rotor 2: PX=+0.13  PY=-0.22  KM=-0.05  rear-left,   CW
  Rotor 3: PX=-0.13  PY=+0.20  KM=-0.05  front-right, CW

Sign conventions (FRD/NED):
  Roll+  = right wing down (rotation about +X body)
  Pitch+ = nose up         (rotation about +Y body)
  Yaw+   = clockwise from above (rotation about +Z body, which points down)
"""
import numpy as np
import casadi as ca
from acados_template import AcadosModel


class DroneModelRotor(object):
    def __init__(self):
        model = AcadosModel()
        constraint = ca.types.SimpleNamespace()

        # Physical parameters (X500 from SDF)
        self.I_xx = 0.02166666666666667  # kg*m^2
        self.I_yy = 0.02166666666666667  # kg*m^2
        self.I_zz = 0.0400               # kg*m^2
        self.m = 2.5372                  # kg
        self.g = 9.81                    # m/s^2
        self.I = np.diag([self.I_xx, self.I_yy, self.I_zz])
        self.I_inv = np.diag([1.0/self.I_xx, 1.0/self.I_yy, 1.0/self.I_zz])

        self.motorConstant = 8.54858e-6   # N/(rad/s)^2  (thrust = kf * omega^2)
        self.maxRotVelocity = 1000.0      # rad/s
        self.maxSingleRotorThrust = self.motorConstant * self.maxRotVelocity**2  # ~8.55 N
        self.maxThrust = 4 * self.maxSingleRotorThrust  # ~34.19 N total

        # Aerodynamic drag coefficient
        self.k_drag = 0.0  # N/(m/s)^2

        # X500 rotor geometry (PX4 4001_gz_x500 airframe, FRD body frame)
        # [px, py] in meters, km = yaw drag-to-thrust moment coefficient
        self.rotor_px = np.array([+0.13, -0.13, +0.13, -0.13])  # forward(-)/ rear(+)
        self.rotor_py = np.array([+0.22, -0.20, -0.22, +0.20])  # right(+) / left(-)
        self.rotor_km = np.array([+0.05, +0.05, -0.05, -0.05])  # CCW(+) / CW(-)

        # Hover command (normalized [0,1] per rotor for level hover)
        self.hover_thrust_per_rotor_N = self.m * self.g / 4.0  # ~6.22 N
        self.hover_normalized = self.hover_thrust_per_rotor_N / self.maxSingleRotorThrust  # ~0.727

        # ============================================
        # Control inputs: per-rotor normalized thrust [0, 1]
        # ============================================
        u0 = ca.MX.sym('u0', 1)  # rotor 0 (rear-right, CCW)
        u1 = ca.MX.sym('u1', 1)  # rotor 1 (front-left, CCW)
        u2 = ca.MX.sym('u2', 1)  # rotor 2 (rear-left, CW)
        u3 = ca.MX.sym('u3', 1)  # rotor 3 (front-right, CW)

        controls = ca.vertcat(u0, u1, u2, u3)

        # ============================================
        # State vector (13 states, same as DroneModel)
        # ============================================
        p = ca.MX.sym('p', 3)    # position NED [north, east, down]
        v = ca.MX.sym('v', 3)    # velocity NED [vn, ve, vd]
        q = ca.MX.sym('q', 4)    # quaternion [w,x,y,z] body->NED
        w = ca.MX.sym('w', 3)    # angular rates body [wx, wy, wz] (rad/s)

        states = ca.vertcat(p, v, q, w)

        # ============================================
        # Per-rotor forces (Newtons)
        # ============================================
        kf = self.maxSingleRotorThrust  # max thrust per rotor
        F0 = u0 * kf
        F1 = u1 * kf
        F2 = u2 * kf
        F3 = u3 * kf

        # Total thrust along body +Z axis
        T_total = F0 + F1 + F2 + F3

        # ============================================
        # Moments from rotor geometry (FRD body frame)
        # ============================================
        # Roll moment (Mx): torque about body X-axis from rotor forces along Z
        # M_roll = sum(py_i * Fi) — force along +Z at position py creates roll
        Mx = self.rotor_py[0]*F0 + self.rotor_py[1]*F1 + self.rotor_py[2]*F2 + self.rotor_py[3]*F3

        # Pitch moment (My): torque about body Y-axis
        # M_pitch = -sum(px_i * Fi) — negative because +px is rear, and
        # increasing rear thrust produces nose-up (+pitch)
        My = -(self.rotor_px[0]*F0 + self.rotor_px[1]*F1 + self.rotor_px[2]*F2 + self.rotor_px[3]*F3)

        # Yaw moment (Mz): reaction torque from rotor drag
        # CCW rotors (km>0) produce +Mz reaction, CW rotors (km<0) produce -Mz
        Mz = self.rotor_km[0]*F0 + self.rotor_km[1]*F1 + self.rotor_km[2]*F2 + self.rotor_km[3]*F3

        # ============================================
        # System dynamics
        # ============================================

        # 1. Position dynamics: p_dot = v
        p_dot = v

        # 2. Velocity dynamics: m*v_dot = R*F_thrust - m*g - F_drag
        qw, qx, qy, qz = q[0], q[1], q[2], q[3]

        # Normalize quaternion to prevent numerical drift
        q_norm = ca.sqrt(qw**2 + qx**2 + qy**2 + qz**2 + 1e-8)
        qw, qx, qy, qz = qw/q_norm, qx/q_norm, qy/q_norm, qz/q_norm

        # Thrust vector in body frame (total thrust along body +Z)
        F_body = ca.vertcat(0, 0, T_total)

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
        v_norm = ca.sqrt(ca.dot(v, v) + 1e-6)
        F_drag = self.k_drag * v_norm * v

        # Acceleration (same sign convention as original model)
        a = (-F_thrust_ned + self.m * g_ned - F_drag) / self.m
        v_dot = a

        # 3. Quaternion kinematics: q_dot = 0.5 * q ⊗ [0, w]
        q_dot = 0.5 * ca.vertcat(
            -w[0]*qx - w[1]*qy - w[2]*qz,
             w[0]*qw + w[2]*qy - w[1]*qz,
             w[1]*qw - w[2]*qx + w[0]*qz,
             w[2]*qw + w[1]*qx - w[0]*qy
        )

        # 4. Angular rate dynamics: full Euler's equations
        # I * w_dot = M - w x (I * w)
        M_vec = ca.vertcat(Mx, My, Mz)
        I_diag = ca.vertcat(self.I_xx, self.I_yy, self.I_zz)
        Iw = I_diag * w  # element-wise since I is diagonal
        w_cross_Iw = ca.vertcat(
            w[1]*Iw[2] - w[2]*Iw[1],
            w[2]*Iw[0] - w[0]*Iw[2],
            w[0]*Iw[1] - w[1]*Iw[0]
        )
        I_inv_diag = ca.vertcat(1.0/self.I_xx, 1.0/self.I_yy, 1.0/self.I_zz)
        w_dot = I_inv_diag * (M_vec - w_cross_Iw)

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
        model.name = 'drone_rotor'

        # ============================================
        # Physical constraints
        # ============================================
        # Per-rotor thrust limits (normalized [0, 1])
        constraint.u_min = np.array([0.0, 0.0, 0.0, 0.0])
        constraint.u_max = np.array([1.0, 1.0, 1.0, 1.0])

        # State bounds (angular rates in state)
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

        print(f"DroneModelRotor initialized: nx={states.size1()}, nu={controls.size1()}")
        print(f"  Mass: {self.m} kg, Max single rotor thrust: {self.maxSingleRotorThrust:.2f} N")
        print(f"  Hover per-rotor (normalized): {self.hover_normalized:.4f}")
        print(f"  Rotor layout (FRD): PX={self.rotor_px}, PY={self.rotor_py}")
        print(f"  Rotor directions: KM={self.rotor_km} (CCW=+, CW=-)")
