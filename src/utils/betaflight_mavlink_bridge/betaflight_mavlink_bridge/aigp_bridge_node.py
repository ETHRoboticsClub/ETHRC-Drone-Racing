#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, Imu
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool, Trigger
from pymavlink import mavutil
import time
import threading
import numpy as np
from scipy.spatial.transform import Rotation as R

class AigpMavlinkBridge(Node):
    def __init__(self):
        super().__init__('aigp_bridge_node')

        # Parameters
        self.declare_parameter('device', 'udpin:0.0.0.0:14550')
        self.declare_parameter('publish_imu', True)
        self.declare_parameter('auto_arm', True)
        self.declare_parameter('auto_switch_index', 4)  # Switch for Auto mode (L1)
        self.declare_parameter('arm_switch_index', 5)   # Switch for Arming (R1)
        self.declare_parameter('reset_switch_index', 0) # Switch for Reset (A/Cross)

        self.device = self.get_parameter('device').get_parameter_value().string_value
        self.publish_imu = self.get_parameter('publish_imu').get_parameter_value().bool_value
        self.auto_arm = self.get_parameter('auto_arm').get_parameter_value().bool_value
        self.auto_switch_index = self.get_parameter('auto_switch_index').get_parameter_value().integer_value
        self.arm_switch_index = self.get_parameter('arm_switch_index').get_parameter_value().integer_value
        self.reset_switch_index = self.get_parameter('reset_switch_index').get_parameter_value().integer_value

        # State
        self.is_auto = True
        self.is_armed = False
        self.system_boot_ms = int(time.time() * 1000)

        # State fields for /drone_state_sim
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)
        self.quat = np.array([1.0, 0.0, 0.0, 0.0]) # [qw, qx, qy, qz]
        self.ang_vel = np.zeros(3)
        self.state_lock = threading.Lock()

        # Manual control rates/thrust defaults
        self.manual_roll_rate = 0.0
        self.manual_pitch_rate = 0.0
        self.manual_yaw_rate = 0.0
        self.manual_thrust = 0.0

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
        self.state_pub = self.create_publisher(Float64MultiArray, 'drone_state_sim', 10)
        if self.publish_imu:
            self.imu_pub = self.create_publisher(Imu, '/imu0', 10)

        # Subscribers
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.auto_sub = self.create_subscription(Float64MultiArray, '/drone_attitude_cmd', self.auto_callback, 10)

        # Services
        self.arm_srv = self.create_service(SetBool, '/arm_drone', self.arm_drone_srv_cb)
        self.reset_srv = self.create_service(Trigger, '/reset_sim', self.reset_sim_srv_cb)

        # Background threads / timers
        if self.master:
            # 10Hz Timesync Timer
            self.timesync_timer = self.create_timer(0.1, self.send_timesync)
            # 50Hz Manual Control Timer
            self.manual_control_timer = self.create_timer(0.02, self.manual_control_timer_cb)

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

            if msg_type == "ODOMETRY":
                with self.state_lock:
                    self.pos = np.array([msg.x, msg.y, msg.z])
                    self.vel = np.array([msg.vx, msg.vy, msg.vz])
                    # MAVLink ODOMETRY q order is qw, qx, qy, qz
                    self.quat = np.array([msg.q[0], msg.q[1], msg.q[2], msg.q[3]])
                    self.ang_vel = np.array([msg.rollspeed, msg.pitchspeed, msg.yawspeed])
                self.publish_state()

            elif msg_type == "LOCAL_POSITION_NED":
                with self.state_lock:
                    self.pos = np.array([msg.x, msg.y, msg.z])
                    self.vel = np.array([msg.vx, msg.vy, msg.vz])
                self.publish_state()

            elif msg_type == "ATTITUDE":
                # Convert Euler angles roll, pitch, yaw (in rad) to quaternion [qw, qx, qy, qz]
                r = R.from_euler('xyz', [msg.roll, msg.pitch, msg.yaw])
                q = r.as_quat() # [qx, qy, qz, qw]
                with self.state_lock:
                    self.quat = np.array([q[3], q[0], q[1], q[2]])
                    self.ang_vel = np.array([msg.rollspeed, msg.pitchspeed, msg.yawspeed])
                self.publish_state()

            elif msg_type == "HIGHRES_IMU" and self.publish_imu:
                self.publish_imu_msg(msg)

    def publish_state(self):
        msg = Float64MultiArray()
        with self.state_lock:
            msg.data = np.concatenate((self.pos, self.vel, self.quat, self.ang_vel)).tolist()
        self.state_pub.publish(msg)

    def publish_imu_msg(self, msg):
        imu_msg = Imu()
        imu_msg.header.frame_id = "imu_link"
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.linear_acceleration.x = msg.xacc
        imu_msg.linear_acceleration.y = msg.yacc
        imu_msg.linear_acceleration.z = msg.zacc
        imu_msg.angular_velocity.x = msg.xgyro
        imu_msg.angular_velocity.y = msg.ygyro
        imu_msg.angular_velocity.z = msg.zgyro
        imu_msg.orientation_covariance[0] = -1.0
        self.imu_pub.publish(imu_msg)

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

            now_ms = int(time.time() * 1000)
            time_boot_ms = now_ms - self.system_boot_ms

            try:
                self.master.mav.set_attitude_target_send(
                    time_boot_ms,
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE,
                    [1.0, 0.0, 0.0, 0.0],
                    roll_rad_s,
                    pitch_rad_s,
                    yaw_rad_s,
                    thrust
                )
            except Exception as e:
                self.get_logger().error(f"Failed to send autonomous attitude target: {e}")

    def manual_control_timer_cb(self):
        if not self.is_auto and self.master:
            now_ms = int(time.time() * 1000)
            time_boot_ms = now_ms - self.system_boot_ms
            try:
                self.master.mav.set_attitude_target_send(
                    time_boot_ms,
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE,
                    [1.0, 0.0, 0.0, 0.0],
                    self.manual_roll_rate,
                    self.manual_pitch_rate,
                    self.manual_yaw_rate,
                    self.manual_thrust
                )
            except Exception as e:
                self.get_logger().error(f"Failed to send manual attitude target: {e}")

    def destroy_node(self):
        self.stop_rx = True
        if hasattr(self, 'rx_thread'):
            self.rx_thread.join(timeout=1.0)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = AigpMavlinkBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
