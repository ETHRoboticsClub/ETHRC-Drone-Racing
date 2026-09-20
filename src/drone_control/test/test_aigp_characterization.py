import numpy as np

from drone_control.aigp_characterization import AigpCharacterization, G, Sample


def _imu_sample(t, phase, thrust, accel_z, gyro=None):
    return Sample(
        t=t,
        phase=phase,
        cmd_roll_deg_s=0.0,
        cmd_pitch_deg_s=0.0,
        cmd_yaw_deg_s=0.0,
        cmd_thrust=thrust,
        cmd_actuators=np.full(4, thrust),
        state=np.full(13, np.nan),
        imu_accel=np.array([0.0, 0.0, accel_z]),
        imu_gyro=np.zeros(3) if gyro is None else np.asarray(gyro),
    )


def test_thrust_map_can_be_estimated_from_imu_without_state_telemetry():
    analyzer = object.__new__(AigpCharacterization)
    phases = {}
    for thrust in (0.3, 0.5, 0.7):
        phase = f"thrust_{thrust:.2f}"
        phases[phase] = [
            _imu_sample(i * 0.05, phase, thrust, accel_z=-20.0 * thrust)
            for i in range(12)
        ]

    points = analyzer.estimate_thrust_points(phases)
    result = analyzer.fit_thrust_map(points)

    assert all(point["source"] == "imu_specific_force" for point in points)
    assert abs(result["hover_thrust"] - G / 20.0) < 1e-6
    assert abs(result["thrust_to_weight"] - 20.0 / G) < 1e-6


def test_rate_response_falls_back_to_imu_gyro():
    analyzer = object.__new__(AigpCharacterization)
    analyzer.command_mode = "actuator"
    analyzer.actuator_step_delta = 0.1
    samples = [
        _imu_sample(
            i * 0.05,
            "rate_roll_step",
            0.5,
            accel_z=-G,
            gyro=[1.0 - np.exp(-(i * 0.05) / 0.1), 0.0, 0.0],
        )
        for i in range(12)
    ]

    result = analyzer.estimate_rate_tau(samples, axis_index=0)

    assert result is not None
    assert result["max_abs_rate_rad_s"] > 0.9
    assert result["command_actuator_delta"] == 0.1

