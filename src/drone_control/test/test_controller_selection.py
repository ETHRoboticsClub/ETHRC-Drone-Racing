import pytest

from drone_control.drone_controller_node import create_controller


def test_unknown_controller_is_rejected_before_launch():
    with pytest.raises(ValueError, match="Unsupported controller_type"):
        create_controller("unknown")
