import numpy as np

from mpc.drone_mpc_controller import DroneMpcController


def test_identity_enu_flu_becomes_level_ned_frd_with_east_heading():
    state = np.zeros(13)
    state[6:10] = [0.0, 0.0, 0.0, 1.0]
    converted = DroneMpcController._ros_enu_flu_to_mpc_ned_frd(state)
    # An identity ENU pose points body x east, which is +90 degrees yaw in NED.
    assert np.allclose(
        np.abs(converted[6:10]),
        [np.sqrt(0.5), 0.0, 0.0, np.sqrt(0.5)],
    )


def test_position_velocity_and_body_rate_axes():
    state = np.zeros(13)
    state[0:3] = [1.0, 2.0, 3.0]
    state[3:6] = [4.0, 5.0, 6.0]
    state[6:10] = [0.0, 0.0, 0.0, 1.0]
    state[10:13] = [0.1, 0.2, 0.3]
    converted = DroneMpcController._ros_enu_flu_to_mpc_ned_frd(state)
    assert np.allclose(converted[0:3], [2.0, 1.0, -3.0])
    assert np.allclose(converted[3:6], [5.0, 4.0, -6.0])
    assert np.allclose(converted[10:13], [0.1, -0.2, -0.3])


def test_position_goal_builds_stationary_reference():
    state = np.zeros(13)
    state[6] = 1.0
    reference = DroneMpcController._reference_from_goal(state, [1.0, 2.0, 3.0])
    assert np.allclose(reference[0:3], [2.0, 1.0, -3.0])
    assert np.allclose(reference[3:6], 0.0)
    assert np.allclose(reference[6:10], state[6:10])


def test_mpc_rates_are_adapted_to_betaflight_stick_signs():
    controller = DroneMpcController.__new__(DroneMpcController)
    controller.max_thrust = 20.0
    controller.thrust_curve_per_motor_n = (0.0, 0.0, 5.0)
    command = controller.attitude_cmd_from_mpc_cmd(10.0, 0.1, 0.2, 0.3)

    assert np.isclose(command.roll_deg_s, np.degrees(0.1))
    assert np.isclose(command.pitch_deg_s, -np.degrees(0.2))
    assert np.isclose(command.yaw_deg_s, np.degrees(0.3))
    assert np.isclose(command.thrust_value, np.sqrt(0.5))


def test_mpc_hover_thrust_inverts_quadratic_plant_curve():
    controller = DroneMpcController.__new__(DroneMpcController)
    controller.max_thrust = 32.0
    controller.thrust_curve_per_motor_n = (0.0, 0.0, 8.0)

    assert np.isclose(controller._thrust_to_normalized(8.0), 0.5)
    assert np.isclose(controller._thrust_to_normalized(32.0), 1.0)
