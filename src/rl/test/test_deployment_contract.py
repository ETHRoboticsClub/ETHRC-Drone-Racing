import numpy as np

from rl.hovering_experiment import (
    BOREAS_MAX_THRUST_N,
    TRAINING_MAX_THRUST_N,
    policy_rates_to_betaflight_deg_s,
    training_thrust_to_betaflight_command,
    HoveringExperiment,
)


def test_policy_rates_match_linear_500_degree_boreas_profile():
    command = policy_rates_to_betaflight_deg_s([1.0, -0.5, 0.25])
    assert np.allclose(command, [500.0, -250.0, -125.0])


def test_training_force_is_inverted_through_quadratic_propeller_curve():
    command = training_thrust_to_betaflight_command(0.0)
    expected = np.sqrt((0.5 * TRAINING_MAX_THRUST_N) / BOREAS_MAX_THRUST_N)
    assert np.isclose(command, expected)


def test_training_action_extremes_remain_bounded():
    assert training_thrust_to_betaflight_command(-1.0) == 0.0
    assert training_thrust_to_betaflight_command(1.0) == 1.0


def test_residual_outer_loop_has_symmetric_cardinal_commands():
    base = np.zeros(12, dtype=np.float32)
    base[2] = -1.0

    east = base.copy()
    east[6] = -0.1
    west = base.copy()
    west[6] = 0.1
    north = base.copy()
    north[7] = -0.1
    south = base.copy()
    south[7] = 0.1

    assert HoveringExperiment._outer_loop_action(east)[2] < 0.0
    assert HoveringExperiment._outer_loop_action(west)[2] > 0.0
    assert HoveringExperiment._outer_loop_action(north)[1] > 0.0
    assert HoveringExperiment._outer_loop_action(south)[1] < 0.0


def test_safe_nominal_default_does_not_evaluate_checkpoint():
    class FailingPolicy:
        def inference(self, _observation):
            raise AssertionError("safe nominal mode must not run policy inference")

    experiment = HoveringExperiment.__new__(HoveringExperiment)
    experiment.policy = FailingPolicy()
    observation = np.zeros(12, dtype=np.float32)
    observation[2] = -1.0

    command = experiment.compute_actions(observation)

    assert np.all(np.isfinite(command))
    assert np.allclose(command[:3], 0.0)


def test_default_residual_mask_keeps_collective_and_yaw_nominal():
    from rl.hovering_experiment import _RESIDUAL_MASK

    assert np.array_equal(_RESIDUAL_MASK, [0.0, 1.0, 1.0, 0.0])
