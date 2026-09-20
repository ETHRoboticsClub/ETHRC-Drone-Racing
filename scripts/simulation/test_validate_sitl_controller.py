from validate_sitl_controller import ControllerProbe


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def _probe(goal_delay_after_arm):
    probe = ControllerProbe.__new__(ControllerProbe)
    probe.goal = (1.0, -2.0, 0.5)
    probe.arm_delay = 5.5
    probe.goal_delay_after_arm = goal_delay_after_arm
    probe.controller_ready_at = 100.0
    probe.goal_published = False
    probe.goal_publisher = _Publisher()
    return probe


def test_zero_delay_publishes_requested_goal_immediately(monkeypatch):
    probe = _probe(0.0)
    monkeypatch.setattr("validate_sitl_controller.time.monotonic", lambda: 100.0)

    probe._publish_goal()

    message = probe.goal_publisher.messages[-1]
    assert (message.x, message.y, message.z) == (1.0, -2.0, 0.5)


def test_delayed_goal_waits_until_after_arm_delay(monkeypatch):
    probe = _probe(3.0)
    monkeypatch.setattr("validate_sitl_controller.time.monotonic", lambda: 108.4)
    probe._publish_goal()
    assert not probe.goal_publisher.messages

    monkeypatch.setattr("validate_sitl_controller.time.monotonic", lambda: 108.5)
    probe._publish_goal()
    assert len(probe.goal_publisher.messages) == 1
