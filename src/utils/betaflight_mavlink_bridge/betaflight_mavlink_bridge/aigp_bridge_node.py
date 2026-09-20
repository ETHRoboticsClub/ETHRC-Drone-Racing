#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.time import Time
from sensor_msgs.msg import Joy, Imu
from std_msgs.msg import Bool, Float64MultiArray, Int32
from std_srvs.srv import SetBool, Trigger
from pymavlink import mavutil
import time
import threading
import numpy as np
from collections import Counter
from scipy.spatial.transform import Rotation as R

from betaflight_mavlink_bridge.aigp_race_status import (
    ENCAPSULATED_RACE_STATUS_MSG_ID,
    ENCAPSULATED_TRACK_INFO_MSG_ID,
    parse_race_status_payload,
)


# Simulator revision 3390 added this extension bit to opt into the documented
# physical rad/s interpretation of SET_ATTITUDE_TARGET.body_*_rate. Bits 8 and
# 16 are unused by the standard ATTITUDE_TARGET_TYPEMASK enum.
ATTITUDE_TARGET_TYPEMASK_DCL_BODY_RATES_RADS = 16


def attitude_target_type_mask(use_rad_per_second: bool = True) -> int:
    mask = mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE
    if use_rad_per_second:
        mask |= ATTITUDE_TARGET_TYPEMASK_DCL_BODY_RATES_RADS
    return mask


def body_velocity_to_world_ned(velocity_body, quaternion_wxyz):
    """Rotate a BODY_NED/FRD velocity vector into LOCAL_NED."""

    velocity_body = np.asarray(velocity_body, dtype=np.float64)
    quaternion_wxyz = np.asarray(quaternion_wxyz, dtype=np.float64)
    if velocity_body.shape != (3,) or quaternion_wxyz.shape != (4,):
        raise ValueError("Expected a 3D velocity and wxyz quaternion")
    rotation_ned_from_frd = R.from_quat(
        [quaternion_wxyz[1], quaternion_wxyz[2], quaternion_wxyz[3], quaternion_wxyz[0]]
    )
    return rotation_ned_from_frd.apply(velocity_body)


def rate_thrust_to_motor_commands(
    desired_rates_frd,
    measured_rates_frd,
    thrust,
    rate_p_gains,
    rate_feedforward_gains=None,
):
    """Mix FRD body-rate errors into the verified four AIGP motor controls."""

    desired = np.asarray(desired_rates_frd, dtype=np.float64)
    measured = np.asarray(measured_rates_frd, dtype=np.float64)
    gains = np.asarray(rate_p_gains, dtype=np.float64)
    feedforward = (
        np.zeros(3, dtype=np.float64)
        if rate_feedforward_gains is None
        else np.asarray(rate_feedforward_gains, dtype=np.float64)
    )
    if (
        desired.shape != (3,)
        or measured.shape != (3,)
        or gains.shape != (3,)
        or feedforward.shape != (3,)
    ):
        raise ValueError("Body rates and proportional gains must be 3-vectors")
    # Empirically verified VQ motor patterns, columns = positive FRD roll,
    # pitch, yaw. Build 3390 was checked against live HIGHRES_IMU feedback.
    mixer = np.array(
        [
            [-1.0, -1.0, 1.0],
            [1.0, -1.0, -1.0],
            [-1.0, 1.0, -1.0],
            [1.0, 1.0, 1.0],
        ],
        dtype=np.float64,
    )
    differentials = feedforward * desired + gains * (desired - measured)
    motors = float(thrust) + mixer @ differentials
    return np.clip(motors, 0.0, 1.0)

class AigpMavlinkBridge(Node):
    def __init__(self):
        super().__init__('aigp_bridge_node')

        # Parameters
        self.declare_parameter('device', 'udpin:0.0.0.0:14550')
        self.declare_parameter('publish_imu', True)
        self.declare_parameter('publish_imu_aux', False)
        # HIGHRES_IMU uses simulator boot time while camera packets use server
        # epoch time. Local receipt stamps keep the VQ2 streams coherent.
        self.declare_parameter('use_sim_timestamp', False)
        self.declare_parameter('auto_arm', True)
        self.declare_parameter('use_rad_per_second_body_rates', True)
        # ODOMETRY is the coherent VQ1 source: its quaternion agrees with the
        # IMU gravity projection. ATTITUDE uses a different pitch convention.
        self.declare_parameter('ground_truth_source', 'odometry')
        self.declare_parameter('ground_truth_state_topic', '/aigp/ground_truth/state')
        self.declare_parameter('rate_control_backend', 'actuator')
        self.declare_parameter('actuator_rate_p_gains', [0.006, 0.006, 0.010])
        self.declare_parameter(
            'actuator_rate_feedforward_gains', [0.014, 0.014, 0.019]
        )
        self.declare_parameter('actuator_command_topic', '/drone_actuator_cmd')
        self.declare_parameter('velocity_command_topic', '/drone_velocity_cmd')
        self.declare_parameter('velocity_command_frame', 'body_ned')
        self.declare_parameter('velocity_command_timeout_s', 0.25)
        self.declare_parameter('attitude_command_timeout_s', 0.25)
        self.declare_parameter('actuator_group', 0)
        self.declare_parameter('auto_switch_index', 4)  # Switch for Auto mode (L1)
        self.declare_parameter('arm_switch_index', 5)   # Switch for Arming (R1)
        self.declare_parameter('reset_switch_index', 0) # Switch for Reset (A/Cross)

        self.device = self.get_parameter('device').get_parameter_value().string_value
        self.publish_imu = self.get_parameter('publish_imu').get_parameter_value().bool_value
        self.publish_imu_aux = self.get_parameter('publish_imu_aux').get_parameter_value().bool_value
        self.use_sim_timestamp = self.get_parameter('use_sim_timestamp').get_parameter_value().bool_value
        self.auto_arm = self.get_parameter('auto_arm').get_parameter_value().bool_value
        self.use_rad_per_second_body_rates = self.get_parameter(
            'use_rad_per_second_body_rates'
        ).get_parameter_value().bool_value
        self.ground_truth_source = self.get_parameter(
            'ground_truth_source'
        ).get_parameter_value().string_value
        self.ground_truth_state_topic = self.get_parameter(
            'ground_truth_state_topic'
        ).get_parameter_value().string_value
        self.rate_control_backend = self.get_parameter(
            'rate_control_backend'
        ).get_parameter_value().string_value
        self.actuator_rate_p_gains = np.asarray(
            self.get_parameter('actuator_rate_p_gains').value, dtype=np.float64
        )
        self.actuator_rate_feedforward_gains = np.asarray(
            self.get_parameter('actuator_rate_feedforward_gains').value,
            dtype=np.float64,
        )
        self.actuator_command_topic = self.get_parameter('actuator_command_topic').get_parameter_value().string_value
        self.velocity_command_topic = self.get_parameter('velocity_command_topic').get_parameter_value().string_value
        self.velocity_command_frame = self.get_parameter('velocity_command_frame').get_parameter_value().string_value
        self.velocity_command_timeout_s = float(
            self.get_parameter('velocity_command_timeout_s').value
        )
        self.attitude_command_timeout_s = float(
            self.get_parameter('attitude_command_timeout_s').value
        )
        self.actuator_group = self.get_parameter('actuator_group').get_parameter_value().integer_value
        self.auto_switch_index = self.get_parameter('auto_switch_index').get_parameter_value().integer_value
        self.arm_switch_index = self.get_parameter('arm_switch_index').get_parameter_value().integer_value
        self.reset_switch_index = self.get_parameter('reset_switch_index').get_parameter_value().integer_value
        if self.ground_truth_source not in ('local_position_attitude', 'odometry'):
            raise ValueError(
                "ground_truth_source must be 'local_position_attitude' or 'odometry'"
            )
        if self.rate_control_backend not in ('actuator', 'attitude_target'):
            raise ValueError(
                "rate_control_backend must be 'actuator' or 'attitude_target'"
            )
        if self.actuator_rate_p_gains.shape != (3,):
            raise ValueError('actuator_rate_p_gains must contain roll, pitch, yaw gains')
        if self.actuator_rate_feedforward_gains.shape != (3,):
            raise ValueError(
                'actuator_rate_feedforward_gains must contain roll, pitch, yaw gains'
            )
        self.attitude_target_type_mask = attitude_target_type_mask(
            self.use_rad_per_second_body_rates
        )

        # State
        self.is_auto = True
        self.is_armed = False
        self.system_boot_ms = int(time.time() * 1000)
        self.rx_counts = Counter()
        self.last_rx_log_time = time.time()
        self.track_chunks = {}
        self.expected_num_track_chunks = {}

        # State fields for /drone_state_sim
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)
        self.quat = np.array([1.0, 0.0, 0.0, 0.0]) # [qw, qx, qy, qz]
        self.ang_vel = np.zeros(3)
        self.has_local_position = False
        self.has_attitude = False
        self.state_lock = threading.Lock()

        # Manual control rates/thrust defaults
        self.manual_roll_rate = 0.0
        self.manual_pitch_rate = 0.0
        self.manual_yaw_rate = 0.0
        self.manual_thrust = 0.0
        self.last_velocity_command_time = None
        self.velocity_timeout_zero_sent = False
        self.last_attitude_command_time = None
        self.attitude_timeout_zero_sent = False

        # MAVLink setup
        self.get_logger().info(f"Connecting to AIGP simulator on {self.device}...")
        try:
            self.master = mavutil.mavlink_connection(self.device)
            self.get_logger().info("Waiting for heartbeat...")
            self.master.wait_heartbeat(timeout=10)
            self.get_logger().info(f"Connected to simulator system: {self.master.target_system}")
            if self.auto_arm:
                self.send_arm_command(True)
                self.is_armed = True
                self.get_logger().info("Auto-armed the simulator drone.")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to MAVLink: {e}")
            self.master = None

        # Publishers
        # Canonical 13D VQ1 ground truth: position/linear velocity in LOCAL_NED,
        # quaternion and angular velocity in BODY_FRD. Keep the historical topic
        # as a compatibility alias, but VQ1 launch selects the dedicated topic.
        self.state_pub = self.create_publisher(
            Float64MultiArray, self.ground_truth_state_topic, 10
        )
        self.legacy_state_pub = self.create_publisher(
            Float64MultiArray, '/drone_state_sim', 10
        )
        self.active_gate_pub = self.create_publisher(Int32, '/aigp/active_gate_index', 10)
        self.race_status_pub = self.create_publisher(Float64MultiArray, '/aigp/race_status', 10)
        self.collision_pub = self.create_publisher(Float64MultiArray, '/aigp/collision', 10)
        self.actuator_output_pub = self.create_publisher(
            Float64MultiArray, '/aigp/actuator_output_status', 10
        )
        self.armed_pub = self.create_publisher(Bool, '/aigp/armed', 10)
        if self.publish_imu:
            self.imu_pub = self.create_publisher(Imu, '/imu0', 10)
        if self.publish_imu_aux:
            self.imu_aux_pub = self.create_publisher(
                Float64MultiArray, '/aigp/highres_imu_aux', 10
            )

        # Subscribers
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.auto_sub = self.create_subscription(Float64MultiArray, '/drone_attitude_cmd', self.auto_callback, 10)
        self.actuator_sub = self.create_subscription(
            Float64MultiArray,
            self.actuator_command_topic,
            self.actuator_callback,
            10
        )
        self.velocity_sub = self.create_subscription(
            Float64MultiArray,
            self.velocity_command_topic,
            self.velocity_callback,
            10,
        )

        # Services
        self.arm_srv = self.create_service(SetBool, '/arm_drone', self.arm_drone_srv_cb)
        self.reset_srv = self.create_service(Trigger, '/reset_sim', self.reset_sim_srv_cb)

        # Background threads / timers
        if self.master:
            # 10Hz Timesync Timer
            self.timesync_timer = self.create_timer(0.1, self.send_timesync)
            # 50Hz Manual Control Timer
            self.manual_control_timer = self.create_timer(0.02, self.manual_control_timer_cb)
            self.velocity_watchdog_timer = self.create_timer(
                0.05, self.velocity_command_watchdog_cb
            )
            self.attitude_watchdog_timer = self.create_timer(
                0.05, self.attitude_command_watchdog_cb
            )

            # Receiver Thread
            self.stop_rx = False
            self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
            self.rx_thread.start()

    def send_timesync(self):
        if self.master:
            try:
                now_ns = int(time.time_ns())
                self.master.mav.timesync_send(
                    now_ns, # tc1: client time
                    0       # ts1: 0 (request)
                )
            except Exception as e:
                self.get_logger().error(f"Error sending timesync: {e}")

    def rx_loop(self):
        while not self.stop_rx:
            if not self.master:
                time.sleep(0.1)
                continue
            try:
                msg = self.master.recv_match(blocking=False)
            except Exception as e:
                self.get_logger().error(f"Error reading MAVLink packet: {e}")
                time.sleep(0.1)
                continue

            if msg is None:
                time.sleep(0.001)
                continue

            msg_type = msg.get_type()
            if msg_type == "BAD_DATA":
                continue
            self.rx_counts[msg_type] += 1
            self.log_rx_counts_periodically()

            if msg_type == "HEARTBEAT":
                armed = bool(
                    int(getattr(msg, 'base_mode', 0))
                    & int(mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                )
                self.is_armed = armed
                armed_msg = Bool()
                armed_msg.data = armed
                self.armed_pub.publish(armed_msg)

            elif msg_type == "ODOMETRY":
                frame_id = int(getattr(msg, 'frame_id', -1))
                child_frame = int(getattr(msg, 'child_frame_id', -1))
                body_frames = {
                    int(mavutil.mavlink.MAV_FRAME_BODY_NED),
                    int(getattr(mavutil.mavlink, 'MAV_FRAME_BODY_FRD', 12)),
                }
                if (
                    frame_id != int(mavutil.mavlink.MAV_FRAME_LOCAL_NED)
                    or child_frame not in body_frames
                ):
                    self.get_logger().warning(
                        "Ignoring ODOMETRY with unsupported frames "
                        f"frame_id={frame_id}, child_frame_id={child_frame}"
                    )
                    continue
                with self.state_lock:
                    self.pos = np.array([msg.x, msg.y, msg.z])
                    # MAVLink ODOMETRY q order is qw, qx, qy, qz
                    self.quat = np.array([msg.q[0], msg.q[1], msg.q[2], msg.q[3]])
                    velocity = np.array([msg.vx, msg.vy, msg.vz])
                    self.vel = body_velocity_to_world_ned(velocity, self.quat)
                    self.ang_vel = np.array([msg.rollspeed, msg.pitchspeed, msg.yawspeed])
                if self.ground_truth_source == 'odometry':
                    self.publish_state()

            elif msg_type == "LOCAL_POSITION_NED":
                with self.state_lock:
                    self.pos = np.array([msg.x, msg.y, msg.z])
                    self.vel = np.array([msg.vx, msg.vy, msg.vz])
                    self.has_local_position = True
                if self.ground_truth_source == 'local_position_attitude' and self.has_attitude:
                    self.publish_state()

            elif msg_type == "ATTITUDE":
                # Convert Euler angles roll, pitch, yaw (in rad) to quaternion [qw, qx, qy, qz]
                r = R.from_euler('xyz', [msg.roll, msg.pitch, msg.yaw])
                q = r.as_quat() # [qx, qy, qz, qw]
                with self.state_lock:
                    self.quat = np.array([q[3], q[0], q[1], q[2]])
                    self.ang_vel = np.array([msg.rollspeed, msg.pitchspeed, msg.yawspeed])
                    self.has_attitude = True
                if self.ground_truth_source == 'local_position_attitude' and self.has_local_position:
                    self.publish_state()

            elif msg_type == "HIGHRES_IMU":
                with self.state_lock:
                    self.ang_vel = np.array([msg.xgyro, msg.ygyro, msg.zgyro])
                if self.publish_imu or self.publish_imu_aux:
                    self.publish_imu_msg(msg)

            elif msg_type == "ACTUATOR_OUTPUT_STATUS":
                self.publish_actuator_output(msg)

            elif msg_type == "COLLISION":
                self.publish_collision(msg)

            elif msg_type == "ENCAPSULATED_DATA":
                self.handle_encapsulated_data(msg)

            elif msg_type == "DATA_TRANSMISSION_HANDSHAKE":
                self.handle_track_data_handshake(msg)

    def log_rx_counts_periodically(self):
        now = time.time()
        if now - self.last_rx_log_time < 2.0:
            return
        self.last_rx_log_time = now
        key_counts = {
            key: self.rx_counts.get(key, 0)
            for key in (
                "ODOMETRY",
                "LOCAL_POSITION_NED",
                "ATTITUDE",
                "HIGHRES_IMU",
                "ACTUATOR_OUTPUT_STATUS",
                "COLLISION",
            )
        }
        self.get_logger().info(f"MAVLink RX counts: {key_counts}")

    def publish_state(self):
        msg = Float64MultiArray()
        with self.state_lock:
            msg.data = np.concatenate((self.pos, self.vel, self.quat, self.ang_vel)).tolist()
        self.state_pub.publish(msg)
        self.legacy_state_pub.publish(msg)

    def publish_imu_msg(self, msg):
        if self.publish_imu:
            imu_msg = Imu()
            imu_msg.header.frame_id = "imu_link"
            if self.use_sim_timestamp and getattr(msg, 'time_usec', 0) > 0:
                imu_msg.header.stamp = Time(
                    nanoseconds=int(msg.time_usec) * 1000
                ).to_msg()
            else:
                imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.linear_acceleration.x = msg.xacc
            imu_msg.linear_acceleration.y = msg.yacc
            imu_msg.linear_acceleration.z = msg.zacc
            imu_msg.angular_velocity.x = msg.xgyro
            imu_msg.angular_velocity.y = msg.ygyro
            imu_msg.angular_velocity.z = msg.zgyro
            imu_msg.orientation_covariance[0] = -1.0
            self.imu_pub.publish(imu_msg)

        if self.publish_imu_aux:
            aux_msg = Float64MultiArray()
            aux_msg.data = [
                float(getattr(msg, 'xmag', float('nan'))),
                float(getattr(msg, 'ymag', float('nan'))),
                float(getattr(msg, 'zmag', float('nan'))),
                float(getattr(msg, 'abs_pressure', float('nan'))),
                float(getattr(msg, 'diff_pressure', float('nan'))),
                float(getattr(msg, 'pressure_alt', float('nan'))),
                float(getattr(msg, 'temperature', float('nan'))),
                float(getattr(msg, 'fields_updated', 0)),
                float(getattr(msg, 'time_usec', 0)),
            ]
            self.imu_aux_pub.publish(aux_msg)

    def publish_actuator_output(self, msg):
        output_msg = Float64MultiArray()
        actuator = list(getattr(msg, 'actuator', []))
        output_msg.data = [float(value) for value in actuator[:4]]
        self.actuator_output_pub.publish(output_msg)

    def publish_collision(self, msg):
        collision_msg = Float64MultiArray()
        collision_msg.data = [
            float(getattr(msg, 'id', -1)),
            float(getattr(msg, 'threat_level', 0)),
            float(getattr(msg, 'horizontal_minimum_delta', 0.0)),
        ]
        self.collision_pub.publish(collision_msg)

    def handle_encapsulated_data(self, msg):
        raw_payload = bytes(msg.data)
        if not raw_payload:
            return

        data_type = int(raw_payload[0])
        if data_type == ENCAPSULATED_RACE_STATUS_MSG_ID:
            self.publish_race_status(raw_payload)
        elif data_type == ENCAPSULATED_TRACK_INFO_MSG_ID:
            self.collect_track_data_packet(msg, raw_payload)

    def publish_race_status(self, raw_payload):
        try:
            status = parse_race_status_payload(raw_payload)
        except ValueError as exc:
            self.get_logger().warning(f"Ignoring malformed AIGP race-status packet: {exc}")
            return

        active_gate_msg = Int32()
        active_gate_msg.data = int(status.active_gate_index)
        self.active_gate_pub.publish(active_gate_msg)

        status_msg = Float64MultiArray()
        status_msg.data = [
            float(status.sim_boot_time_ms),
            float(status.race_start_boot_time_ms),
            float(status.race_finish_time_ns),
            float(status.active_gate_index),
            float(status.last_gate_race_time),
        ]
        self.race_status_pub.publish(status_msg)

    def handle_track_data_handshake(self, msg):
        transfer_id = msg.width
        self.track_chunks[transfer_id] = {}
        self.expected_num_track_chunks[transfer_id] = msg.packets

    def collect_track_data_packet(self, msg, raw_payload):
        if len(raw_payload) < 3:
            return
        transfer_id = int.from_bytes(raw_payload[1:3], byteorder="little", signed=False)
        if transfer_id not in self.expected_num_track_chunks:
            return
        self.track_chunks[transfer_id][msg.seqnr] = raw_payload[3:]

        if len(self.track_chunks[transfer_id]) != self.expected_num_track_chunks[transfer_id]:
            return

        del self.track_chunks[transfer_id]
        del self.expected_num_track_chunks[transfer_id]
        self.get_logger().debug(
            "Received complete AIGP track-data transfer; static track YAML remains the policy source."
        )

    def send_arm_command(self, arm=True):
        if self.master:
            try:
                self.master.mav.command_long_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0,
                    1 if arm else 0,
                    0, 0, 0, 0, 0, 0
                )
            except Exception as e:
                self.get_logger().error(f"Failed to send arm command: {e}")

    def send_reset_command(self):
        if self.master:
            try:
                self.master.mav.command_long_send(
                    self.master.target_system,
                    self.master.target_component,
                    31000, # MAVLINK_CMD_SIM_RESET
                    0,
                    0, 0, 0, 0, 0, 0, 0
                )
            except Exception as e:
                self.get_logger().error(f"Failed to send reset command: {e}")

    def arm_drone_srv_cb(self, request, response):
        req = request.data
        if self.master:
            self.send_arm_command(req)
            self.is_armed = req
            response.success = True
            response.message = f"Arm command sent: {req}"
        else:
            response.success = False
            response.message = "No connection to simulator"
        return response

    def reset_sim_srv_cb(self, request, response):
        if self.master:
            self.send_reset_command()
            response.success = True
            response.message = "Reset command sent to simulator"
        else:
            response.success = False
            response.message = "No connection to simulator"
        return response

    def joy_callback(self, msg):
        # Check Mode Switch (L1)
        if self.auto_switch_index < len(msg.buttons):
            new_auto = msg.buttons[self.auto_switch_index] == 1
            if new_auto != self.is_auto:
                self.is_auto = new_auto
                self.get_logger().info(f"Mode changed: {'AUTONOMOUS' if self.is_auto else 'MANUAL'}")

        # Check Arm Switch (R1)
        if self.arm_switch_index < len(msg.buttons):
            new_arm = msg.buttons[self.arm_switch_index] == 1
            if new_arm != self.is_armed:
                self.is_armed = new_arm
                self.get_logger().info(f"Arm status changed: {'ARMED' if self.is_armed else 'DISARMED'}")
                self.send_arm_command(new_arm)

        # Check Reset Switch (A/Cross)
        if self.reset_switch_index < len(msg.buttons):
            if msg.buttons[self.reset_switch_index] == 1:
                self.get_logger().info("Reset button pressed. Resetting sim...")
                self.send_reset_command()

        # Read manual axes if in MANUAL mode
        if not self.is_auto:
            # Map axes values [-1.0, 1.0] to body rates / thrust
            # roll/pitch/yaw: max 300 deg/s = ~5.23 rad/s
            self.manual_roll_rate = (msg.axes[0] if len(msg.axes) > 0 else 0.0) * 5.23
            self.manual_pitch_rate = (msg.axes[1] if len(msg.axes) > 1 else 0.0) * 5.23
            self.manual_yaw_rate = (msg.axes[2] if len(msg.axes) > 2 else 0.0) * 5.23
            # Throttle axis msg.axes[3] maps from [-1, 1] to [0, 1]
            throttle = msg.axes[3] if len(msg.axes) > 3 else -1.0
            self.manual_thrust = (throttle + 1.0) / 2.0

    def auto_callback(self, msg):
        if self.is_auto and len(msg.data) >= 4 and self.master:
            # Expecting [roll_deg_s, pitch_deg_s, yaw_deg_s, thrust]
            roll_rad_s = msg.data[0] * (np.pi / 180.0)
            pitch_rad_s = msg.data[1] * (np.pi / 180.0)
            yaw_rad_s = msg.data[2] * (np.pi / 180.0)
            thrust = msg.data[3]
            self.last_attitude_command_time = time.monotonic()
            self.attitude_timeout_zero_sent = False

            try:
                self.send_rate_thrust_target(
                    roll_rad_s, pitch_rad_s, yaw_rad_s, thrust
                )
            except Exception as e:
                self.get_logger().error(f"Failed to send autonomous rate target: {e}")

    def attitude_command_watchdog_cb(self):
        """Cut collective when the autonomous rate publisher disappears."""

        if (
            not self.is_auto
            or not self.master
            or self.last_attitude_command_time is None
            or self.attitude_timeout_zero_sent
        ):
            return
        age = time.monotonic() - self.last_attitude_command_time
        if age > self.attitude_command_timeout_s:
            self.send_rate_thrust_target(0.0, 0.0, 0.0, 0.0)
            self.attitude_timeout_zero_sent = True
            self.get_logger().warning(
                f"Attitude command stale for {age:.3f}s; sent zero thrust/rates."
            )

    def send_rate_thrust_target(self, roll_rad_s, pitch_rad_s, yaw_rad_s, thrust):
        if self.rate_control_backend == 'actuator':
            with self.state_lock:
                measured_rates = self.ang_vel.copy()
            controls = rate_thrust_to_motor_commands(
                [roll_rad_s, pitch_rad_s, yaw_rad_s],
                measured_rates,
                thrust,
                self.actuator_rate_p_gains,
                self.actuator_rate_feedforward_gains,
            )
            self.master.mav.set_actuator_control_target_send(
                int(time.time_ns() / 1000),
                self.actuator_group,
                self.master.target_system,
                self.master.target_component,
                [float(value) for value in controls] + [0.0] * 4,
            )
            return

        now_ms = int(time.time() * 1000)
        self.master.mav.set_attitude_target_send(
            now_ms - self.system_boot_ms,
            self.master.target_system,
            self.master.target_component,
            self.attitude_target_type_mask,
            [1.0, 0.0, 0.0, 0.0],
            roll_rad_s,
            pitch_rad_s,
            yaw_rad_s,
            thrust,
        )

    def actuator_callback(self, msg):
        if self.is_auto and self.master:
            # MAVLink SET_ACTUATOR_CONTROL_TARGET carries 8 normalized controls.
            # AIGP currently exposes the first four as motor commands.
            controls = [0.0] * 8
            for i, value in enumerate(msg.data[:8]):
                controls[i] = float(value)

            try:
                self.master.mav.set_actuator_control_target_send(
                    int(time.time_ns() / 1000),
                    self.actuator_group,
                    self.master.target_system,
                    self.master.target_component,
                    controls
                )
            except Exception as e:
                self.get_logger().error(f"Failed to send actuator target: {e}")

    def velocity_callback(self, msg):
        """Send [vx, vy, vz, yaw_rate] through SET_POSITION_TARGET_LOCAL_NED.

        ``velocity_command_frame`` may be ``body_ned`` (x forward, y right,
        z down) or ``local_ned``. This path exists so the simulator's
        stabilized velocity controller can be evaluated before committing a
        learned policy to raw rates or motor commands.
        """
        if not self.is_auto or not self.master or len(msg.data) < 3:
            return

        vx, vy, vz = (float(msg.data[i]) for i in range(3))
        yaw_rate = float(msg.data[3]) if len(msg.data) >= 4 else 0.0
        self.last_velocity_command_time = time.monotonic()
        self.velocity_timeout_zero_sent = False
        self.send_velocity_target(vx, vy, vz, yaw_rate)

    def send_velocity_target(self, vx, vy, vz, yaw_rate=0.0):
        """Send one stabilized velocity target in the configured frame."""

        frame_by_name = {
            'body_ned': mavutil.mavlink.MAV_FRAME_BODY_NED,
            'local_ned': mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        }
        coordinate_frame = frame_by_name.get(self.velocity_command_frame)
        if coordinate_frame is None:
            self.get_logger().error(
                "velocity_command_frame must be 'body_ned' or 'local_ned'"
            )
            return
        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
        )
        now_ms = int(time.time() * 1000)
        try:
            self.master.mav.set_position_target_local_ned_send(
                now_ms - self.system_boot_ms,
                self.master.target_system,
                self.master.target_component,
                coordinate_frame,
                type_mask,
                0.0,
                0.0,
                0.0,
                vx,
                vy,
                vz,
                0.0,
                0.0,
                0.0,
                0.0,
                yaw_rate,
            )
        except Exception as e:
            self.get_logger().error(f"Failed to send velocity target: {e}")

    def velocity_command_watchdog_cb(self):
        """Replace a stale nonzero velocity target with an explicit stop."""

        if (
            not self.is_auto
            or not self.master
            or self.last_velocity_command_time is None
            or self.velocity_timeout_zero_sent
        ):
            return
        age = time.monotonic() - self.last_velocity_command_time
        if age > self.velocity_command_timeout_s:
            self.send_velocity_target(0.0, 0.0, 0.0, 0.0)
            self.velocity_timeout_zero_sent = True
            self.get_logger().warning(
                f"Velocity command stale for {age:.3f}s; sent zero target."
            )

    def manual_control_timer_cb(self):
        if not self.is_auto and self.master:
            try:
                self.send_rate_thrust_target(
                    self.manual_roll_rate,
                    self.manual_pitch_rate,
                    self.manual_yaw_rate,
                    self.manual_thrust,
                )
            except Exception as e:
                self.get_logger().error(f"Failed to send manual rate target: {e}")

    def destroy_node(self):
        if self.master and self.last_attitude_command_time is not None:
            self.send_rate_thrust_target(0.0, 0.0, 0.0, 0.0)
        if self.master and self.last_velocity_command_time is not None:
            self.send_velocity_target(0.0, 0.0, 0.0, 0.0)
        self.stop_rx = True
        if hasattr(self, 'rx_thread'):
            self.rx_thread.join(timeout=1.0)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = AigpMavlinkBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
