import math

from validate_sitl_plant import _command_for_per_motor_thrust


def test_measured_boreas_curve_has_expected_hover_command():
    command = _command_for_per_motor_thrust(
        [0.0, 0.0, 8.9828914], 0.8351086449771665 * 9.80665 / 4.0
    )
    assert math.isclose(command, 0.4774124685, abs_tol=1e-10)
