from pathlib import Path
import importlib.util

import numpy as np
import pytest

from rl.aigp_policy_runtime import (
    AIGP_OURS_105,
    AIGP_OURS_105_MANIFEST,
    AigpOursObservationBuilder,
    Gate,
    LEGACY_CORNERS_25,
    build_observation,
    gate_rotation_from_rpy_deg,
    load_track,
    policy_action_to_attitude_cmd,
    TorchScriptPolicy,
)


def _identity_state():
    state = np.zeros(13, dtype=np.float64)
    state[6] = 1.0
    return state


def test_observation_shape_corner_order_and_previous_action():
    state = _identity_state()
    state[3:6] = [1.0, 2.0, 3.0]
    state[10:13] = [0.1, 0.2, 0.3]
    previous_action = np.array([0.1, -0.2, 0.3, -0.4], dtype=np.float32)
    gate = Gate(
        position=np.array([10.0, 0.0, 0.0]),
        rotation=gate_rotation_from_rpy_deg(0.0, 0.0, 0.0),
        inner_size_m=1.5,
    )

    obs = build_observation(state, [gate], 0, previous_action, velocity_frame="world_ned")

    assert obs.shape == (25,)
    np.testing.assert_allclose(obs[0:3], [1.0, -2.0, -3.0])
    np.testing.assert_allclose(obs[3:6], [0.1, -0.2, -0.3])
    np.testing.assert_allclose(obs[6:9], [0.0, 0.0, -1.0])
    np.testing.assert_allclose(
        obs[9:21],
        [
            9.25,
            0.0,
            0.75,
            10.75,
            0.0,
            0.75,
            10.75,
            0.0,
            -0.75,
            9.25,
            0.0,
            -0.75,
        ],
    )
    np.testing.assert_allclose(obs[21:25], previous_action)


def test_observation_supports_body_velocity_override():
    state = _identity_state()
    state[3:6] = [1.0, 2.0, 3.0]
    gate = Gate(
        position=np.zeros(3),
        rotation=gate_rotation_from_rpy_deg(0.0, 0.0, 0.0),
        inner_size_m=1.5,
    )

    obs = build_observation(state, [gate], 0, np.zeros(4), velocity_frame="body_frd")

    np.testing.assert_allclose(obs[0:3], [1.0, -2.0, -3.0])


def test_aigp_ours_105_manifest_and_oldest_to_newest_history():
    gates = [
        Gate(
            position=np.array(position, dtype=np.float64),
            rotation=gate_rotation_from_rpy_deg(0.0, 0.0, 0.0),
            inner_size_m=size,
        )
        for position, size in (
            ([10.0, 0.0, 0.0], 2.0),
            ([0.0, 10.0, 0.0], 4.0),
            ([0.0, 0.0, 10.0], 6.0),
        )
    ]
    builder = AigpOursObservationBuilder()
    first_state = _identity_state()
    first_action = np.array([0.1, 0.2, 0.3, 0.4], dtype=np.float32)
    first = builder.build(first_state, gates, 0, first_action, velocity_limit=10.0)

    assert builder.contract == AIGP_OURS_105
    assert first.shape == (105,)
    assert AIGP_OURS_105_MANIFEST == (
        ("base_lin_vel_body_flu_history", 0, 9),
        ("base_ang_vel_body_flu_history", 9, 18),
        ("projected_gravity_body_history", 18, 27),
        ("last_action_history", 27, 39),
        ("next_3_gates_rel_forward_half_width_history", 39, 102),
        ("velocity_limit_history", 102, 105),
    )
    expected_gate_sample = np.array(
        [
            1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0,
            0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 2.0,
            0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 3.0,
        ],
        dtype=np.float32,
    )
    np.testing.assert_allclose(first[27:39], np.tile(first_action, 3))
    np.testing.assert_allclose(first[39:102], np.tile(expected_gate_sample, 3))
    np.testing.assert_allclose(first[102:105], [10.0, 10.0, 10.0])

    second_state = _identity_state()
    second_state[3:6] = [2.0, 0.0, 0.0]
    second = builder.build(
        second_state,
        gates,
        0,
        np.array([-0.1, -0.2, -0.3, -0.4]),
        velocity_limit=12.0,
    )
    # Isaac Lab's circular observation histories are oldest -> newest and are
    # initialized by repeating the first sample.
    np.testing.assert_allclose(second[0:9], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0])
    np.testing.assert_allclose(second[102:105], [10.0, 10.0, 12.0])


def test_policy_action_mapping_clamps_and_flips_rates():
    cmd, clamped = policy_action_to_attitude_cmd(
        np.array([2.0, 1.0, 0.5, -0.5]),
        rate_command_scale_rad_s=30.0,
    )

    np.testing.assert_allclose(clamped, [1.0, 1.0, 0.5, -0.5])
    np.testing.assert_allclose(
        cmd,
        [
            np.degrees(30.0),
            np.degrees(-15.0),
            np.degrees(15.0),
            1.0,
        ],
    )


def test_load_packaged_track_uses_nwu_coordinates():
    repo_root = Path(__file__).resolve().parents[3]
    track = load_track(repo_root / "src" / "rl" / "config" / "aigp_competition_track.yaml")

    assert len(track) == 6
    np.testing.assert_allclose(track[0].position, [-23.298, 0.400, 0.032])
    assert track[0].inner_size_m == 1.5


def test_torchscript_policy_smoke_if_torch_available():
    if importlib.util.find_spec("torch") is None:
        pytest.skip("PyTorch is not installed in this environment.")

    repo_root = Path(__file__).resolve().parents[3]
    policy_path = repo_root / "ext" / "nicola" / "policy.pt"
    if not policy_path.exists():
        pytest.skip("Optional Nicola policy artifact is not present in this checkout.")
    policy = TorchScriptPolicy(policy_path)

    assert policy.observation_dim == 25
    assert policy.observation_contract == LEGACY_CORNERS_25
    action = policy(np.zeros(25, dtype=np.float32))
    assert action.shape == (4,)
    policy.reset()


def test_exported_tamed_policy_uses_105d_contract_if_available():
    if importlib.util.find_spec("torch") is None:
        pytest.skip("PyTorch is not installed in this environment.")

    repo_root = Path(__file__).resolve().parents[3]
    policy_path = repo_root / "ext" / "nicola" / "tamed_policy_perception_torchscript.pt"
    if not policy_path.exists():
        pytest.skip("Exported tamed policy artifact is not present.")

    policy = TorchScriptPolicy(policy_path)

    assert policy.observation_dim == 105
    assert policy.observation_contract == AIGP_OURS_105
    action = policy(np.zeros(105, dtype=np.float32))
    assert action.shape == (4,)
    assert np.all(np.isfinite(action))
