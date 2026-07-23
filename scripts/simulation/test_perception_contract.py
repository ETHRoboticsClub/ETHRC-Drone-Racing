from pathlib import Path

from perception_contract import validate_contract


ROOT = Path(__file__).resolve().parents[2]


def test_checked_in_sitl_perception_ingress_contract_is_valid():
    assert validate_contract(
        ROOT / "src/utils/gazebo_ros_bridges/config/gz_bridges.yaml",
        ROOT / "sim/gazebo/models/drone_sitl/model.sdf",
    ) == []
