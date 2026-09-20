from types import SimpleNamespace

from drone_control.state_bridge_node import gazebo_message_time_seconds


def test_gazebo_message_timestamp_uses_simulation_time():
    message = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=12, nsec=250_000_000))
    )

    assert gazebo_message_time_seconds(message) == 12.25


def test_gazebo_message_timestamp_rejects_missing_or_zero_stamp():
    assert gazebo_message_time_seconds(SimpleNamespace()) is None
    assert gazebo_message_time_seconds(
        SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=0, nsec=0)))
    ) is None
