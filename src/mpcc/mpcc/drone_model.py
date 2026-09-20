#!/usr/bin/env python
"""
Augmented quadrotor dynamics model for MPCC.

Implements the state/control space and ODE from Section IV-A and
eqs. (12)–(16) of:
  Romero et al., "Model Predictive Contouring Control for Time-Optimal
  Quadrotor Flight", IEEE T-RO 2022.

State (19 dims):  x = [p(3), q(4), v(3), ω(3), f(4), θ(1), vθ(1)]
Control (5 dims): u = [Δvθ(1), Δf(4)]

State indices
─────────────
  0:3   p   — position in NED inertial frame [m]
  3:7   q   — quaternion [w, x, y, z], body → NED
  7:10  v   — velocity in NED inertial frame [m/s]
 10:13  ω   — body rates in FRD body frame [rad/s]
 13:17  f   — per-rotor thrust [N]  (state, first-order rotor model)
   17   θ   — arc-length progress [m]
   18   vθ  — progress velocity [m/s]

Control indices
───────────────
   0    Δvθ — progress acceleration [m/s²]
  1:5   Δf  — per-rotor thrust rate [N/s]

Physical parameters match the Zephyros 5-inch racing quadrotor.
Frame convention: FRD body, NED inertial.
"""

import numpy as np
import casadi as ca
from acados_template import AcadosModel


class DroneMpccModel:
    """CasADi / ACADOS augmented model for MPCC."""

    # ── Physical constants (Zephyros 5-inch racing quadrotor) ─────────────
    # Total mass: 969 g (measured), max total thrust: 30.8 N → T/W = 3.24
    # Inertia: SolidWorks frame-only values (ixx=0.001021, iyy=0.001544,
    #   izz=0.001377) scaled by 955.1/441.8 = 2.162 to account for full
    #   drone mass (motors, ESC, battery, FC). Refine with real measurements.
    MASS = 0.969               # kg
    G    = 9.81                # m/s²
    I_XX = 0.002207            # kg·m²
    I_YY = 0.003338            # kg·m²
    I_ZZ = 0.002977            # kg·m²

    # Per-rotor thrust limits [N]
    # max = 30.8 N / 4 motors = 7.7 N per motor
    F_MIN = 0.0
    F_MAX = 7.7                # N

    # Rotor geometry in FRD body frame (from Zephyros airframe 4023_gz_zephyros)
    # Positions from SolidWorks URDF (x-forward, y-right).
    # KM sign: + = CCW (looking from above), - = CW.
    #   Rotor 0: front-right  CCW  PX=+0.0884  PY=+0.0720  KM=+0.016
    #   Rotor 1: back-left    CCW  PX=-0.0822  PY=-0.0680  KM=+0.016
    #   Rotor 2: front-left   CW   PX=+0.0886  PY=-0.0680  KM=-0.016
    #   Rotor 3: back-right   CW   PX=-0.0818  PY=+0.0718  KM=-0.016
    ROTOR_PX = np.array([+0.0884, -0.0822, +0.0886, -0.0818])
    ROTOR_PY = np.array([+0.0720, -0.0680, -0.0680, +0.0718])
    # Match Zephyros Gazebo SDF momentConstant=0.016, preserving rotor directions.
    ROTOR_KM = np.array([+0.016,  +0.016,  -0.016,  -0.016 ])

    # Drag coefficients — linear drag in body frame (eq. 14), set to 0
    DRAG = np.zeros(3)

    def __init__(self):
        model = AcadosModel()
        constraint = ca.types.SimpleNamespace()

        # ── State symbols ──────────────────────────────────────────────
        p  = ca.MX.sym('p',  3)   # position (NED)
        q  = ca.MX.sym('q',  4)   # quaternion [w, x, y, z]
        v  = ca.MX.sym('v',  3)   # velocity (NED)
        w  = ca.MX.sym('w',  3)   # body rates (FRD)
        f  = ca.MX.sym('f',  4)   # per-rotor thrust [N]
        th = ca.MX.sym('th', 1)   # arc-length θ
        vt = ca.MX.sym('vt', 1)   # progress velocity vθ

        states = ca.vertcat(p, q, v, w, f, th, vt)   # 19 × 1

        # ── Control symbols ────────────────────────────────────────────
        dvt = ca.MX.sym('dvt', 1)   # Δvθ — progress acceleration [m/s²]
        df  = ca.MX.sym('df',  4)   # Δf  — rotor thrust rates [N/s]

        controls = ca.vertcat(dvt, df)   # 5 × 1

        # ── Rotation matrix R(q): body frame → NED frame ───────────────
        qw, qx, qy, qz = q[0], q[1], q[2], q[3]
        q_norm = ca.sqrt(qw**2 + qx**2 + qy**2 + qz**2 + 1e-8)
        qw = qw / q_norm
        qx = qx / q_norm
        qy = qy / q_norm
        qz = qz / q_norm

        # Standard rotation from quaternion [w,x,y,z]
        R_bn = ca.vertcat(
            ca.horzcat(1 - 2*(qy**2 + qz**2),   2*(qx*qy - qw*qz),    2*(qx*qz + qw*qy)),
            ca.horzcat(2*(qx*qy + qw*qz),        1 - 2*(qx**2 + qz**2), 2*(qy*qz - qw*qx)),
            ca.horzcat(2*(qx*qz - qw*qy),        2*(qy*qz + qw*qx),    1 - 2*(qx**2 + qy**2))
        )

        # ── Rotor forces → collective thrust + moments (eq. 13) ────────
        f0, f1, f2, f3 = f[0], f[1], f[2], f[3]

        T_total = f0 + f1 + f2 + f3
        Mx = (self.ROTOR_PY[0]*f0 + self.ROTOR_PY[1]*f1
              + self.ROTOR_PY[2]*f2 + self.ROTOR_PY[3]*f3)
        My = -(self.ROTOR_PX[0]*f0 + self.ROTOR_PX[1]*f1
               + self.ROTOR_PX[2]*f2 + self.ROTOR_PX[3]*f3)
        Mz = (self.ROTOR_KM[0]*f0 + self.ROTOR_KM[1]*f1
              + self.ROTOR_KM[2]*f2 + self.ROTOR_KM[3]*f3)

        # ── Dynamics (eqs. 12–16) ───────────────────────────────────────

        # 1) Position: ṗ = v
        p_dot = v

        # 2) Velocity: v̇ = g + (1/m) R(q) f_T − R(q) D Rᵀ(q) v  (eq. 14)
        #    In FRD/NED: thrust is upward = body −z; we use the same
        #    convention as mpc/drone_model.py where F_body = [0,0,T] and
        #    the sign is accounted for in v_dot below.
        F_body = ca.vertcat(0.0, 0.0, T_total)
        F_thrust_ned = R_bn @ F_body
        g_ned = ca.vertcat(0.0, 0.0, self.G)

        # Linear drag: R D Rᵀ v  (body-frame drag applied to NED velocity)
        D_body = ca.diag(ca.vertcat(self.DRAG[0], self.DRAG[1], self.DRAG[2]))
        drag_ned = R_bn @ (D_body @ (R_bn.T @ v))

        v_dot = g_ned - F_thrust_ned / self.MASS - drag_ned

        # 3) Quaternion kinematics: q̇ = ½ q ⊙ [0; ω]  (eq. 12)
        q_dot = 0.5 * ca.vertcat(
            -w[0]*qx - w[1]*qy - w[2]*qz,
             w[0]*qw + w[2]*qy - w[1]*qz,
             w[1]*qw - w[2]*qx + w[0]*qz,
             w[2]*qw + w[1]*qx - w[0]*qy
        )

        # 4) Angular rates: J ω̇ = τ − ω × Jω  (eq. 12)
        M_vec = ca.vertcat(Mx, My, Mz)
        Iw = ca.vertcat(self.I_XX * w[0], self.I_YY * w[1], self.I_ZZ * w[2])
        w_cross_Iw = ca.vertcat(
            w[1]*Iw[2] - w[2]*Iw[1],
            w[2]*Iw[0] - w[0]*Iw[2],
            w[0]*Iw[1] - w[1]*Iw[0]
        )
        w_dot = ca.vertcat(
            (M_vec[0] - w_cross_Iw[0]) / self.I_XX,
            (M_vec[1] - w_cross_Iw[1]) / self.I_YY,
            (M_vec[2] - w_cross_Iw[2]) / self.I_ZZ,
        )

        # 5) Rotor dynamics: ḟ = Δf  (first-order model, eq. 16)
        f_dot = df

        # 6) Progress dynamics (eq. 16)
        th_dot = vt
        vt_dot = dvt

        # ── Assemble ODE ────────────────────────────────────────────────
        f_expl = ca.vertcat(p_dot, q_dot, v_dot, w_dot, f_dot, th_dot, vt_dot)

        # ── ACADOS model fields ─────────────────────────────────────────
        x_dot_sym = ca.MX.sym('x_dot', states.size1())
        model.f_expl_expr = f_expl
        model.f_impl_expr = x_dot_sym - f_expl
        model.x    = states
        model.xdot = x_dot_sym
        model.u    = controls
        model.p    = []
        model.name = 'drone_mpcc'

        # ── Constraint bounds ───────────────────────────────────────────
        # Body rates [rad/s] — indices 10, 11, 12
        w_max = np.pi
        constraint.w_min = np.full(3, -w_max)
        constraint.w_max = np.full(3,  w_max)

        # Per-rotor thrust [N] — indices 13, 14, 15, 16
        constraint.f_min = np.full(4, self.F_MIN)
        constraint.f_max = np.full(4, self.F_MAX)

        # Per-rotor thrust rate [N/s] — controls 1:5
        df_rate = 20.0
        constraint.df_min = np.full(4, -df_rate)
        constraint.df_max = np.full(4,  df_rate)

        # Progress velocity [m/s] — index 18
        constraint.vt_min = 0.0
        constraint.vt_max = 20.0

        # Progress acceleration [m/s²] — control 0
        dvt_rate = 10.0
        constraint.dvt_min = -dvt_rate
        constraint.dvt_max =  dvt_rate

        self.model      = model
        self.constraint = constraint
        self.nx = int(states.size1())    # 19
        self.nu = int(controls.size1())  # 5

        self.f_hover = np.full(4, self.MASS * self.G / 4.0)   # ≈ 2.38 N each

        print(f"DroneMpccModel: nx={self.nx}, nu={self.nu}")
        print(f"  f_hover per rotor: {self.f_hover[0]:.3f} N  "
              f"(total {self.f_hover.sum():.2f} N ≈ {self.MASS}×{self.G:.2f} N)")
