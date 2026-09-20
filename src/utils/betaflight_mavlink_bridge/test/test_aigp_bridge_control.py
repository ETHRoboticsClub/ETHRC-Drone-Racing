from types import SimpleNamespace
import threading
import time

import numpy as np

from pymavlink import mavutil

from betaflight_mavlink_bridge.aigp_bridge_node import (
    ATTITUDE_TARGET_TYPEMASK_DCL_BODY_RATES_RADS,
    AigpMavlinkBridge,
    attitude_target_type_mask,
    body_velocity_to_world_ned,
    rate_thrust_to_motor_commands,
)


class _CaptureMav:
    def __init__(self):
        self.velocity_calls = []
        self.attitude_calls = []
        self.actuator_calls = []

    def set_position_target_local_ned_send(self, *args):
        self.velocity_calls.append(args)

    def set_attitude_target_send(self, *args):
        self.attitude_calls.append(args)

    def set_actuator_control_target_send(self, *args):
        self.actuator_calls.append(args)


class _CapturePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class _Logger:
    def __init__(self):
        self.errors = []
        self.warnings = []

    def error(self, message):
        self.errors.append(message)

    def warning(self, message):
        self.warnings.append(message)


def _bridge_stub(frame="body_ned"):
    mav = _CaptureMav()
    bridge = SimpleNamespace(
        is_auto=True,
        master=SimpleNamespace(
            target_system=1,
            target_component=2,
            mav=mav,
        ),
        velocity_command_frame=frame,
        attitude_target_type_mask=attitude_target_type_mask(True),
        rate_control_backend="attitude_target",
        actuator_rate_p_gains=np.array([0.006, 0.006, 0.010]),
        actuator_rate_feedforward_gains=np.array([0.014, 0.014, 0.019]),
        actuator_group=0,
        ang_vel=np.zeros(3),
        state_lock=threading.Lock(),
        system_boot_ms=0,
        velocity_command_timeout_s=0.25,
        last_velocity_command_time=None,
        velocity_timeout_zero_sent=False,
        attitude_command_timeout_s=0.25,
        last_attitude_command_time=None,
        attitude_timeout_zero_sent=False,
        get_logger=lambda: _Logger(),
    )
    bridge.send_velocity_target = lambda *args: (
        AigpMavlinkBridge.send_velocity_target(bridge, *args)
    )
    bridge.send_rate_thrust_target = lambda *args: (
        AigpMavlinkBridge.send_rate_thrust_target(bridge, *args)
    )
    return bridge


def test_body_velocity_command_uses_body_ned_and_preserves_yaw_rate():
    bridge = _bridge_stub()

    AigpMavlinkBridge.velocity_callback(
        bridge, SimpleNamespace(data=[2.0, -0.5, 0.25, 0.3])
    )

    assert len(bridge.master.mav.velocity_calls) == 1
    args = bridge.master.mav.velocity_calls[0]
    assert args[3] == mavutil.mavlink.MAV_FRAME_BODY_NED
    assert args[8:11] == (2.0, -0.5, 0.25)
    assert args[15] == 0.3
    assert args[4] & mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
    assert not args[4] & mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE


def test_velocity_watchdog_sends_zero_after_command_timeout():
    bridge = _bridge_stub(frame="local_ned")
    AigpMavlinkBridge.velocity_callback(
        bridge, SimpleNamespace(data=[1.0, 0.2, -0.1, 0.0])
    )
    bridge.last_velocity_command_time = time.monotonic() - 1.0

    AigpMavlinkBridge.velocity_command_watchdog_cb(bridge)

    assert len(bridge.master.mav.velocity_calls) == 2
    args = bridge.master.mav.velocity_calls[-1]
    assert args[3] == mavutil.mavlink.MAV_FRAME_LOCAL_NED
    assert args[8:11] == (0.0, 0.0, 0.0)
    assert bridge.velocity_timeout_zero_sent


def test_simulator_timestamp_is_used_for_highres_imu():
    publisher = _CapturePublisher()
    bridge = SimpleNamespace(
        publish_imu=True,
        publish_imu_aux=False,
        use_sim_timestamp=True,
        imu_pub=publisher,
    )
    highres = SimpleNamespace(
        time_usec=1_234_567,
        xacc=1.0,
        yacc=2.0,
        zacc=3.0,
        xgyro=0.1,
        ygyro=0.2,
        zgyro=0.3,
    )

    AigpMavlinkBridge.publish_imu_msg(bridge, highres)

    assert len(publisher.messages) == 1
    stamp = publisher.messages[0].header.stamp
    assert stamp.sec == 1
    assert stamp.nanosec == 234_567_000


def test_attitude_command_opts_into_v4_physical_rad_per_second_rates():
    bridge = _bridge_stub()

    AigpMavlinkBridge.auto_callback(
        bridge, SimpleNamespace(data=[180.0, -90.0, 45.0, 0.6])
    )

    assert len(bridge.master.mav.attitude_calls) == 1
    args = bridge.master.mav.attitude_calls[0]
    assert args[3] & mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE
    assert args[3] & ATTITUDE_TARGET_TYPEMASK_DCL_BODY_RATES_RADS
    assert args[5:9] == (3.141592653589793, -1.5707963267948966, 0.7853981633974483, 0.6)


def test_attitude_watchdog_cuts_thrust_after_command_timeout():
    bridge = _bridge_stub()
    AigpMavlinkBridge.auto_callback(
        bridge, SimpleNamespace(data=[20.0, -10.0, 0.0, 0.3])
    )
    bridge.last_attitude_command_time = time.monotonic() - 1.0

    AigpMavlinkBridge.attitude_command_watchdog_cb(bridge)

    assert len(bridge.master.mav.attitude_calls) == 2
    assert bridge.master.mav.attitude_calls[-1][5:9] == (0.0, 0.0, 0.0, 0.0)
    assert bridge.attitude_timeout_zero_sent


def test_odometry_body_velocity_is_rotated_to_local_ned():
    # 90 degrees yaw: body +X points toward local +Y.
    velocity_ned = body_velocity_to_world_ned(
        [2.0, 0.0, 0.0],
        [2**-0.5, 0.0, 0.0, 2**-0.5],
    )

    assert abs(velocity_ned[0]) < 1e-12
    assert abs(velocity_ned[1] - 2.0) < 1e-12
    assert abs(velocity_ned[2]) < 1e-12


def test_rate_mixer_uses_verified_aigp_motor_patterns_and_clamps():
    roll = rate_thrust_to_motor_commands(
        [2.0, 0.0, 0.0], [0.0, 0.0, 0.0], 0.5, [0.1, 0.1, 0.1]
    )
    pitch = rate_thrust_to_motor_commands(
        [0.0, 2.0, 0.0], [0.0, 0.0, 0.0], 0.5, [0.1, 0.1, 0.1]
    )
    yaw = rate_thrust_to_motor_commands(
        [0.0, 0.0, 20.0], [0.0, 0.0, 0.0], 0.5, [0.1, 0.1, 0.1]
    )

    np.testing.assert_allclose(roll, [0.3, 0.7, 0.3, 0.7])
    np.testing.assert_allclose(pitch, [0.3, 0.3, 0.7, 0.7])
    np.testing.assert_allclose(yaw, [1.0, 0.0, 0.0, 1.0])


def test_actuator_rate_backend_sends_four_mixed_motor_controls():
    bridge = _bridge_stub()
    bridge.rate_control_backend = "actuator"

    AigpMavlinkBridge.send_rate_thrust_target(bridge, 2.0, 0.0, 0.0, 0.5)

    assert len(bridge.master.mav.actuator_calls) == 1
    controls = bridge.master.mav.actuator_calls[0][4]
    np.testing.assert_allclose(controls[:4], [0.46, 0.54, 0.46, 0.54])
    np.testing.assert_allclose(controls[4:], np.zeros(4))
