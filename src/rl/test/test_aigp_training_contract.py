from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[3]
ENV_ROOT = REPO_ROOT / "ext" / "DroneRacing_ManagerBased_Enviroment"


def test_aigp_sweep_selects_explicit_sensor_task_and_deployment_interface():
    config_path = ENV_ROOT / "configs" / "sweeps" / "single" / "aigp_sensor_policy.yaml"
    config = yaml.safe_load(config_path.read_text())

    assert config["task_train"] == "Isaac-Racing-Quadcopter-Competition-AIGP-Sensor-v0"
    assert config["task_play"] == "Isaac-Racing-Quadcopter-Competition-AIGP-Sensor-Play-v0"
    assert config["env_vars"] == {
        "DRONE_MODEL": "anemos_urdf",
        "ACTION_TYPE": "rates",
        "OBSERVATION_SPACE": "ours",
    }
    params = config["param_grid"]
    assert params["domain_randomization"] == [True]
    assert params["action_noise.enable"] == [True]
    assert params["randomize_velocity_limit"] == [True]


def test_aigp_task_is_separate_and_matches_physics_and_command_rates():
    registration = (
        ENV_ROOT
        / "source"
        / "Racing_Quadcopter"
        / "Racing_Quadcopter"
        / "tasks"
        / "manager_based"
        / "racing_quadcopter"
        / "__init__.py"
    ).read_text()
    env_cfg = (
        ENV_ROOT
        / "source"
        / "Racing_Quadcopter"
        / "Racing_Quadcopter"
        / "tasks"
        / "manager_based"
        / "racing_quadcopter"
        / "competition_env_cfg.py"
    ).read_text()

    assert 'id="Isaac-Racing-Quadcopter-Competition-v0"' in registration
    assert 'id="Isaac-Racing-Quadcopter-Competition-AIGP-Sensor-v0"' in registration
    assert "class CompetitionAigpSensorEnvCfg(CompetitionEnvCfg):" in env_cfg
    assert "policy: ObservationsCfg.OursObservationsCfg" in env_cfg
    assert "COMPETITION_PHYSICS_HZ = 120.0" in env_cfg
    assert "control_hz: float = 60.0" in env_cfg

    config = yaml.safe_load(
        (ENV_ROOT / "configs" / "sweeps" / "single" / "aigp_sensor_policy.yaml").read_text()
    )
    params = config["param_grid"]
    assert params["actions.rates.thrust_to_weight"] == [2.33]
    assert params["actions.rates.rate_command_scale"] == [4.0]
    assert params["actions.rates.max_delay"] == [2]
    assert params["actions.rates.pid_gain_randomization_frac"] == [0.2]


def test_isaac_ours_group_matches_the_105d_runtime_manifest():
    base_cfg = (
        ENV_ROOT
        / "source"
        / "Racing_Quadcopter"
        / "Racing_Quadcopter"
        / "tasks"
        / "manager_based"
        / "racing_quadcopter"
        / "racing_quadcopter_env_cfg.py"
    ).read_text()
    ours_group = base_cfg.split("class OursObservationsCfg", 1)[1].split(
        "class MonoRaceObservationsCfg", 1
    )[0]

    assert "_OURS_OBS_WINDOW = 3" in base_cfg
    assert "num_next_gates: int = 3" in base_cfg
    terms_in_order = (
        "base_lin_vel = ObsTerm",
        "base_ang_vel = ObsTerm",
        "projected_gravity = ObsTerm",
        "last_action = ObsTerm",
        "obs_gates = ObsTerm",
        "velocity_limit = ObsTerm",
    )
    offsets = [ours_group.index(term) for term in terms_in_order]
    assert offsets == sorted(offsets)
    assert ours_group.count("history_length=_OURS_OBS_WINDOW") == 6
