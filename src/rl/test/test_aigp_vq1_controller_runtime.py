from pathlib import Path

import numpy as np

from rl.aigp_policy_runtime import FRAME_FLIP, load_track
from rl.aigp_vq1_controller_runtime import (
    RateControllerConfig,
    VelocityFieldConfig,
    rate_thrust_command_frd,
    vehicle_tilt_rad,
    velocity_field_command_ned,
)


TRACK = Path(__file__).resolve().parents[1] / "config/aigp_competition_track.yaml"


def _state_from_nwu(position_nwu, velocity_nwu=(0.0, 0.0, 0.0)):
    # Identity NWU/FLU attitude maps to identity NED/FRD under F R F.
    return np.array(
        [
            *(FRAME_FLIP * np.asarray(position_nwu, dtype=float)),
            *(FRAME_FLIP * np.asarray(velocity_nwu, dtype=float)),
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ]
    )


def test_origin_command_moves_toward_gate_zero_in_local_ned():
    gates = load_track(TRACK)
    command, telemetry = velocity_field_command_ned(
        _state_from_nwu([0.0, 0.0, 0.0]), gates[0], VelocityFieldConfig()
    )

    assert command[0] < -0.95
    assert command[1] < 0.0  # NWU +Y centering becomes NED -Y.
    assert command[2] < 0.0  # Gate 0 is slightly above the origin.
    assert telemetry.forward_distance_m > 23.0
    assert telemetry.centering_error_m < 0.5


def test_steep_gate_leg_keeps_descending_at_the_gate_plane():
    gates = load_track(TRACK)
    gate = gates[2]
    position_nwu = gate.position + np.array([2.0, 0.0, 0.62])
    command, telemetry = velocity_field_command_ned(
        _state_from_nwu(position_nwu), gate, VelocityFieldConfig()
    )

    assert command[0] < 0.0
    assert command[2] > 0.0  # Positive NED Z commands descent toward gate centre.
    assert abs(telemetry.centering_error_m - 0.62) < 1.0e-9


def test_command_is_bounded_and_level_tilt_is_zero():
    gates = load_track(TRACK)
    state = _state_from_nwu([0.0, -20.0, 20.0])
    command, _ = velocity_field_command_ned(
        state,
        gates[0],
        VelocityFieldConfig(max_command_speed_m_s=2.7),
    )

    assert np.linalg.norm(command) <= 2.7 + 1.0e-9
    assert abs(vehicle_tilt_rad(state)) < 1.0e-12


def test_live_start_rate_command_uses_fresh_vq1_collective_calibration():
    gates = load_track(TRACK)
    # Representative build-3390 reset state measured on 2026-07-24.
    state = np.array(
        [
            -0.000742,
            -0.000010,
            0.020605,
            0.0,
            0.0,
            0.0,
            0.000336,
            -0.154726,
            0.0000305,
            -0.987957,
            0.0,
            0.0,
            0.0,
        ]
    )
    command, telemetry = rate_thrust_command_frd(
        state, gates[0], RateControllerConfig()
    )

    assert command.shape == (4,)
    assert 0.25 < command[3] < 0.35
    assert command[1] < 0.0  # Build-3390 pitch-command sign backs off forward pitch.
    assert command[0] < 0.0  # Build-3390 roll sign centers toward gate-0 NED -Y.
    assert np.linalg.norm(telemetry.desired_body_rate_frd) <= 1.5 + 1e-9


def test_collective_calibration_rejects_nonphysical_slope():
    gates = load_track(TRACK)
    state = _state_from_nwu([0.0, 0.0, 0.0])
    try:
        rate_thrust_command_frd(
            state,
            gates[0],
            RateControllerConfig(thrust_fit_slope_ned_m_s2=1.0),
        )
    except ValueError as exc:
        assert "slope must be negative" in str(exc)
    else:
        raise AssertionError("nonphysical thrust slope was accepted")


def test_leveling_override_commands_zero_acceleration_attitude():
    gates = load_track(TRACK)
    state = np.array(
        [
            -0.000742,
            -0.000010,
            0.020605,
            0.0,
            0.0,
            0.0,
            0.000336,
            -0.154726,
            0.0000305,
            -0.987957,
            0.0,
            0.0,
            0.0,
        ]
    )

    command, telemetry = rate_thrust_command_frd(
        state,
        gates[0],
        RateControllerConfig(),
        acceleration_override_nwu=np.zeros(3),
    )

    assert command[1] < 0.0
    assert abs(command[0]) < 1.0
    np.testing.assert_allclose(telemetry.desired_acceleration_nwu, np.zeros(3))


def test_actuator_backend_uses_normal_frd_rate_signs():
    gates = load_track(TRACK)
    state = np.array(
        [
            -0.000742,
            -0.000010,
            0.020605,
            0.0,
            0.0,
            0.0,
            0.000336,
            -0.154726,
            0.0000305,
            -0.987957,
            0.0,
            0.0,
            0.0,
        ]
    )
    command, _ = rate_thrust_command_frd(
        state,
        gates[0],
        RateControllerConfig(roll_rate_sign=1.0, pitch_rate_sign=1.0),
        acceleration_override_nwu=np.zeros(3),
    )

    assert command[1] > 0.0
