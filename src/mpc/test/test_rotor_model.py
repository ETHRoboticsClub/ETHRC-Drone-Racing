#!/usr/bin/env python3
"""
Unit tests for the per-rotor drone dynamics model (DroneModelRotor).
Tests mixing correctness, saturation, sign conventions, and hover equilibrium.

Run: python test_rotor_model.py
"""
import numpy as np
import sys
import os

from mpc.drone_model_rotor import DroneModelRotor


def test_hover_equilibrium():
    """All 4 rotors at hover thrust should produce ~mg upward and zero moments."""
    model = DroneModelRotor()
    f = model.f

    # State: hovering at origin, identity quaternion, zero rates
    x = np.zeros(13)
    x[6] = 1.0  # qw = 1 (identity quaternion)

    # Control: all rotors at hover normalized
    u = np.full(4, model.hover_normalized)

    x_dot = f(x, u).full().flatten()

    # Position derivative = velocity = 0
    assert np.allclose(x_dot[0:3], 0.0, atol=1e-6), f"p_dot should be zero, got {x_dot[0:3]}"

    # Velocity derivative should be ~0 (thrust balances gravity)
    # v_dot = (-F_thrust_ned + m*g*[0,0,1]) / m
    # At hover with identity quaternion: F_thrust_ned = [0, 0, T_total]
    # T_total = 4 * hover_normalized * maxSingleRotorThrust ≈ mg
    assert np.allclose(x_dot[3:6], 0.0, atol=0.1), \
        f"v_dot should be ~zero at hover, got {x_dot[3:6]}"

    # Quaternion derivative should be ~0 (zero angular rates)
    assert np.allclose(x_dot[6:10], 0.0, atol=1e-6), \
        f"q_dot should be zero, got {x_dot[6:10]}"

    # Angular acceleration should be ~0 (symmetric thrust, zero rates)
    assert np.allclose(x_dot[10:13], 0.0, atol=1e-3), \
        f"w_dot should be ~zero at hover, got {x_dot[10:13]}"

    print("[PASS] test_hover_equilibrium")


def test_total_thrust_conservation():
    """Sum of per-rotor thrusts at hover should equal mg."""
    model = DroneModelRotor()
    total_thrust = 4 * model.hover_normalized * model.maxSingleRotorThrust
    expected = model.m * model.g

    assert abs(total_thrust - expected) < 0.01, \
        f"Total thrust {total_thrust:.4f} N != mg {expected:.4f} N"

    print("[PASS] test_total_thrust_conservation")


def test_rotor0_increases_produce_correct_moments():
    """
    Rotor 0 is rear-right (PX=+0.13, PY=+0.22, KM=+0.05, CCW).
    Increasing only rotor 0 should produce:
      - Negative roll moment? No: Mx = py0 * F0, py0 = +0.22 → Mx > 0 → positive roll (right wing down)
      - Negative pitch moment? My = -(px0 * F0), px0 = +0.13 → My < 0 → negative pitch (nose down)
      - Positive yaw moment? Mz = km0 * F0, km0 = +0.05 → Mz > 0 → positive yaw (CW from above)
    """
    model = DroneModelRotor()
    f = model.f

    x = np.zeros(13)
    x[6] = 1.0

    # Baseline: hover on all rotors
    u_hover = np.full(4, model.hover_normalized)

    # Increase rotor 0 by 10%
    u_test = u_hover.copy()
    u_test[0] += 0.1

    x_dot = f(x, u_test).full().flatten()

    # Angular acceleration (w_dot) should show the moment effects
    w_dot = x_dot[10:13]

    # Roll (w_dot[0]): Rotor 0 at PY=+0.22, more force → positive roll moment → positive w_dot_x
    assert w_dot[0] > 0, f"Increasing rotor 0 should produce positive roll, got w_dot_x={w_dot[0]:.4f}"

    # Pitch (w_dot[1]): Rotor 0 at PX=+0.13 (rear), My = -(px*F) → negative pitch
    assert w_dot[1] < 0, f"Increasing rotor 0 should produce negative pitch, got w_dot_y={w_dot[1]:.4f}"

    # Yaw (w_dot[2]): Rotor 0 CCW (KM=+0.05), Mz = km*F → positive yaw
    assert w_dot[2] > 0, f"Increasing rotor 0 should produce positive yaw, got w_dot_z={w_dot[2]:.4f}"

    print("[PASS] test_rotor0_increases_produce_correct_moments")


def test_rotor3_increases_produce_correct_moments():
    """
    Rotor 3 is front-right (PX=-0.13, PY=+0.20, KM=-0.05, CW).
    Increasing only rotor 3 should produce:
      - Mx = py3 * F3, py3 = +0.20 → positive roll
      - My = -(px3 * F3), px3 = -0.13 → My = +0.13*F3 → positive pitch (nose up)
      - Mz = km3 * F3, km3 = -0.05 → negative yaw
    """
    model = DroneModelRotor()
    f = model.f

    x = np.zeros(13)
    x[6] = 1.0

    u_hover = np.full(4, model.hover_normalized)
    u_test = u_hover.copy()
    u_test[3] += 0.1

    x_dot = f(x, u_test).full().flatten()
    w_dot = x_dot[10:13]

    assert w_dot[0] > 0, f"Increasing rotor 3 should produce positive roll, got w_dot_x={w_dot[0]:.4f}"
    assert w_dot[1] > 0, f"Increasing rotor 3 should produce positive pitch, got w_dot_y={w_dot[1]:.4f}"
    assert w_dot[2] < 0, f"Increasing rotor 3 should produce negative yaw, got w_dot_z={w_dot[2]:.4f}"

    print("[PASS] test_rotor3_increases_produce_correct_moments")


def test_symmetric_rotors_zero_moments():
    """All rotors at the same thrust should produce zero net moments."""
    model = DroneModelRotor()
    f = model.f

    x = np.zeros(13)
    x[6] = 1.0

    # Any uniform thrust
    for thrust_level in [0.3, 0.5, 0.7, 0.9]:
        u = np.full(4, thrust_level)
        x_dot = f(x, u).full().flatten()
        w_dot = x_dot[10:13]

        # Note: X500 is not perfectly symmetric (PY values differ slightly: 0.22 vs 0.20)
        # So roll/pitch moments won't be exactly zero. Use relaxed tolerance.
        assert np.allclose(w_dot, 0.0, atol=0.5), \
            f"Uniform thrust {thrust_level:.1f} should produce ~zero moments, got w_dot={w_dot}"

    print("[PASS] test_symmetric_rotors_zero_moments")


def test_roll_command_convention():
    """
    Positive roll (right wing down): increase LEFT rotors (1, 2), decrease RIGHT rotors (0, 3).
    Rotor 1: PY=-0.20 (left), Rotor 2: PY=-0.22 (left)
    Rotor 0: PY=+0.22 (right), Rotor 3: PY=+0.20 (right)

    For positive roll (Mx > 0), we need net positive Mx = sum(py_i * Fi).
    Since right rotors have +PY, increasing them increases Mx.
    This is correct: +roll = right wing down = right side thrust increases.
    """
    model = DroneModelRotor()
    f = model.f

    x = np.zeros(13)
    x[6] = 1.0

    u_hover = np.full(4, model.hover_normalized)

    # Increase right-side rotors (0, 3) and decrease left-side (1, 2) for positive roll
    u_roll = u_hover.copy()
    delta = 0.05
    u_roll[0] += delta  # rear-right
    u_roll[3] += delta  # front-right
    u_roll[1] -= delta  # front-left
    u_roll[2] -= delta  # rear-left

    x_dot = f(x, u_roll).full().flatten()
    w_dot = x_dot[10:13]

    assert w_dot[0] > 0, f"Right-side thrust increase should produce positive roll, got {w_dot[0]:.4f}"

    print("[PASS] test_roll_command_convention")


def test_pitch_command_convention():
    """
    Positive pitch (nose up): increase REAR rotors (0, 2), decrease FRONT rotors (1, 3).
    My = -sum(px_i * Fi). Rear rotors have +PX, so increasing them makes sum(px*F) more positive,
    which makes My more negative. Wait — that's nose DOWN.

    Actually: My = -(px0*F0 + px1*F1 + px2*F2 + px3*F3)
    Rear rotors (0, 2) have px > 0. Increasing them → sum increases → My decreases (more negative).
    Front rotors (1, 3) have px < 0. Increasing them → sum decreases → My increases (more positive).

    So for positive pitch (nose up, My > 0): increase FRONT rotors.
    """
    model = DroneModelRotor()
    f = model.f

    x = np.zeros(13)
    x[6] = 1.0

    u_hover = np.full(4, model.hover_normalized)

    # Increase front rotors for positive pitch (nose up)
    u_pitch = u_hover.copy()
    delta = 0.05
    u_pitch[1] += delta  # front-left
    u_pitch[3] += delta  # front-right
    u_pitch[0] -= delta  # rear-right
    u_pitch[2] -= delta  # rear-left

    x_dot = f(x, u_pitch).full().flatten()
    w_dot = x_dot[10:13]

    assert w_dot[1] > 0, f"Front thrust increase should produce positive pitch, got {w_dot[1]:.4f}"

    print("[PASS] test_pitch_command_convention")


def test_yaw_command_convention():
    """
    Positive yaw (CW from above, nose right, Mz > 0): increase CCW rotors (0, 1).
    CCW rotors have KM > 0, so Mz = sum(km_i * Fi), increasing CCW motor thrust increases Mz.
    """
    model = DroneModelRotor()
    f = model.f

    x = np.zeros(13)
    x[6] = 1.0

    u_hover = np.full(4, model.hover_normalized)

    # Increase CCW rotors (0, 1), decrease CW rotors (2, 3) for positive yaw
    u_yaw = u_hover.copy()
    delta = 0.05
    u_yaw[0] += delta  # CCW
    u_yaw[1] += delta  # CCW
    u_yaw[2] -= delta  # CW
    u_yaw[3] -= delta  # CW

    x_dot = f(x, u_yaw).full().flatten()
    w_dot = x_dot[10:13]

    assert w_dot[2] > 0, f"CCW thrust increase should produce positive yaw, got {w_dot[2]:.4f}"

    print("[PASS] test_yaw_command_convention")


def test_saturation_clamping():
    """Verify that the model handles extreme inputs without NaN/Inf."""
    model = DroneModelRotor()
    f = model.f

    x = np.zeros(13)
    x[6] = 1.0

    # Test with clamped extreme values
    for u_vals in [[0, 0, 0, 0], [1, 1, 1, 1], [0, 1, 0, 1], [1, 0, 1, 0]]:
        u = np.array(u_vals, dtype=float)
        x_dot = f(x, u).full().flatten()
        assert np.all(np.isfinite(x_dot)), f"x_dot has NaN/Inf for u={u}: {x_dot}"

    print("[PASS] test_saturation_clamping")


def test_constraint_bounds():
    """Verify constraint arrays are correctly set."""
    model = DroneModelRotor()
    c = model.constraint

    assert np.allclose(c.u_min, [0, 0, 0, 0]), f"u_min should be [0,0,0,0], got {c.u_min}"
    assert np.allclose(c.u_max, [1, 1, 1, 1]), f"u_max should be [1,1,1,1], got {c.u_max}"
    assert len(c.u_min) == 4
    assert len(c.u_max) == 4

    print("[PASS] test_constraint_bounds")


def test_model_dimensions():
    """Verify state and control dimensions."""
    model = DroneModelRotor()

    nx = model.model.x.size1()
    nu = model.model.u.size1()

    assert nx == 13, f"State dimension should be 13, got {nx}"
    assert nu == 4, f"Control dimension should be 4, got {nu}"

    print("[PASS] test_model_dimensions")


def test_gyroscopic_coupling():
    """With nonzero angular rates, w_dot should include gyroscopic terms (w x Iw)."""
    model = DroneModelRotor()
    f = model.f

    x = np.zeros(13)
    x[6] = 1.0
    x[10] = 2.0  # wx = 2 rad/s (roll rate)
    x[11] = 1.0  # wy = 1 rad/s (pitch rate)

    u = np.full(4, model.hover_normalized)
    x_dot = f(x, u).full().flatten()

    # With wx=2, wy=1, wz=0 and symmetric hover thrust (zero moments):
    # w_dot = I_inv * (M - w x Iw)
    # w x Iw = [wx, wy, 0] x [Ixx*wx, Iyy*wy, 0]
    #        = [wy*0 - 0*Iyy*wy, 0*Ixx*wx - wx*0, wx*Iyy*wy - wy*Ixx*wx]
    #        = [0, 0, wx*wy*(Iyy - Ixx)]
    # Since Ixx ≈ Iyy, the coupling is ~zero for yaw. But for non-equal inertias
    # there should be coupling.
    # The key point is that w_dot is NOT zero when rates are nonzero.
    w_dot = x_dot[10:13]

    # At minimum, the yaw coupling should be nonzero if Ixx != Izz
    # Actually with Ixx=Iyy, the cross term in yaw is zero.
    # But roll/pitch will have cross terms due to Izz != Ixx:
    # w_dot_x = (1/Ixx) * (Mx - wy*wz*(Izz - Iyy)) ← wz=0 so no coupling
    # With wz=0, no gyroscopic coupling in roll/pitch either.

    # Let's test with all three rates nonzero
    x[12] = 1.5  # wz = 1.5 rad/s
    x_dot2 = f(x, u).full().flatten()
    w_dot2 = x_dot2[10:13]

    # Now gyroscopic terms should be nonzero
    # w_dot_x = (1/Ixx) * (Mx - wy*wz*(Izz - Iyy))
    # (Izz - Iyy) = 0.04 - 0.02167 ≈ 0.01833
    # wy*wz*(Izz-Iyy) = 1.0 * 1.5 * 0.01833 ≈ 0.0275
    # This should affect roll acceleration
    assert not np.allclose(w_dot2, 0.0, atol=1e-4), \
        f"Gyroscopic coupling should produce nonzero w_dot with all rates nonzero"

    print("[PASS] test_gyroscopic_coupling")


if __name__ == '__main__':
    print("=" * 60)
    print("DroneModelRotor Unit Tests")
    print("=" * 60)

    tests = [
        test_model_dimensions,
        test_constraint_bounds,
        test_hover_equilibrium,
        test_total_thrust_conservation,
        test_saturation_clamping,
        test_symmetric_rotors_zero_moments,
        test_rotor0_increases_produce_correct_moments,
        test_rotor3_increases_produce_correct_moments,
        test_roll_command_convention,
        test_pitch_command_convention,
        test_yaw_command_convention,
        test_gyroscopic_coupling,
    ]

    passed = 0
    failed = 0
    for test in tests:
        try:
            test()
            passed += 1
        except AssertionError as e:
            print(f"[FAIL] {test.__name__}: {e}")
            failed += 1
        except Exception as e:
            print(f"[ERROR] {test.__name__}: {e}")
            failed += 1

    print(f"\n{'=' * 60}")
    print(f"Results: {passed} passed, {failed} failed out of {len(tests)}")
    print(f"{'=' * 60}")
    sys.exit(0 if failed == 0 else 1)
