#!/usr/bin/env python3
from betaflight_msp_controller.ros_runtime_env import configure_local_ros_defaults

configure_local_ros_defaults()

import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64
from sensor_msgs.msg import Joy
import socket
import struct
import time


def _param_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


class SitlRcBridge(Node):
    def __init__(self):
        super().__init__('sitl_rc_bridge')
        
        # Parameters
        self.declare_parameter('sitl_ip', '127.0.0.1')
        self.declare_parameter('sitl_rc_port', 9004)
        self.declare_parameter('arm_switch_id', 4)  # AUX1 (index 4 in Joy buttons)
        self.declare_parameter('override_switch_id', 5) # AUX2 (index 5 in Joy buttons)
        self.declare_parameter('attitude_cmd_topic', '/drone_attitude_cmd')
        self.declare_parameter('max_rate_deg_s', 500.0)
        self.declare_parameter('rc_warning_interval_s', 1.0)
        self.declare_parameter('joy_udp_port', 9010)
        self.declare_parameter('default_override', False)
        self.declare_parameter('lock_default_override', False)
        self.declare_parameter('override_arm_throttle_delay_s', 1.0)
        
        self.sitl_ip = self.get_parameter('sitl_ip').value
        self.sitl_rc_port = self.get_parameter('sitl_rc_port').value
        self.arm_switch_id = self.get_parameter('arm_switch_id').value
        self.override_switch_id = self.get_parameter('override_switch_id').value
        self.attitude_cmd_topic = self.get_parameter('attitude_cmd_topic').value
        self.max_rate_deg_s = self.get_parameter('max_rate_deg_s').value
        self.rc_warning_interval_s = self.get_parameter('rc_warning_interval_s').value
        self.joy_udp_port = int(self.get_parameter('joy_udp_port').value)
        self.default_override = _param_bool(self.get_parameter('default_override').value)
        self.lock_default_override = _param_bool(self.get_parameter('lock_default_override').value)
        self.override_arm_throttle_delay_s = float(self.get_parameter('override_arm_throttle_delay_s').value)
        
        # RC State [Roll, Pitch, Throttle, Yaw, AUX1, AUX2, AUX3, AUX4, AUX5-12]
        # Padded to 16 channels
        self.rc_channels = [1500, 1500, 1000, 1500, 1000, 1500, 1500, 1500,
                            1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000]
        self.is_armed = False
        self._armed_since_time = None
        self.use_override = self.default_override
        self._last_log_time = 0.0
        self._last_warning_time = 0.0
        self._udp_failures = 0
        
        # Setup UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.get_logger().info(f'Initialized SITL UDP RC socket to {self.sitl_ip}:{self.sitl_rc_port}')
        if self.use_override:
            self.get_logger().info('Default override enabled: attitude commands will drive SITL RC channels.')
        if self.lock_default_override:
            self.get_logger().info('Default override is locked: Joy AUX2 will not disable attitude-command control.')

        self.joy_udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.joy_udp_sock.setblocking(False)
        self.joy_udp_sock.bind(('127.0.0.1', self.joy_udp_port))
        self.get_logger().info(f'Listening for local Joy UDP fallback on 127.0.0.1:{self.joy_udp_port}')
        
        # Start time for timestamps
        self.start_time = time.time()
        
        # Publishers
        self.throttle_pub = self.create_publisher(Float64, '/drone/throttle', 10)
        
        # Subscribers
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.att_sub = self.create_subscription(
            Float64MultiArray, 
            self.attitude_cmd_topic, 
            self.attitude_cmd_callback, 
            10)
        
        # Timer (500Hz for RC heartbeat). Betaflight SITL can advance FC time
        # faster than wall time, so 50Hz can trip RXLOSS even while ROS is alive.
        self.timer = self.create_timer(0.002, self.timer_callback)
        
        self.get_logger().info('SITL RC Bridge Initialized (Handles Joy and Attitude Cmd via UDP)')

    def attitude_cmd_callback(self, msg):
        if not self.use_override:
            return
        if len(msg.data) < 4:
            return

        # msg.data = [roll_rate, pitch_rate, yaw_rate, thrust]
        roll_rate = msg.data[0]
        pitch_rate = msg.data[1]
        yaw_rate = msg.data[2]
        thrust = msg.data[3]

        armed_for = 0.0 if self._armed_since_time is None else time.time() - self._armed_since_time
        if not self.is_armed or armed_for < self.override_arm_throttle_delay_s:
            self.rc_channels[0] = 1500
            self.rc_channels[1] = 1500
            self.rc_channels[2] = 1000
            self.rc_channels[3] = 1500
        else:
            # Update RC stick channels (AETR)
            # Roll, Pitch, Yaw: 1500 +/- 500
            # Throttle: 1000 + thrust * 1000 (0.0 to 1.0)
            self.rc_channels[0] = int(max(1000, min(2000, 1500 + (roll_rate / self.max_rate_deg_s) * 500)))
            self.rc_channels[1] = int(max(1000, min(2000, 1500 + (pitch_rate / self.max_rate_deg_s) * 500)))
            self.rc_channels[2] = int(max(1000, min(2000, 1000 + thrust * 1000)))
            self.rc_channels[3] = int(max(1000, min(2000, 1500 + (yaw_rate / self.max_rate_deg_s) * 500)))

    def joy_callback(self, msg):
        self.apply_joy(msg.axes, msg.buttons)

    def apply_joy(self, joy_axes, joy_buttons):
        # Handle Arming (AUX1)
        if len(joy_buttons) > self.arm_switch_id:
            was_armed = self.is_armed
            self.is_armed = joy_buttons[self.arm_switch_id] == 1
            if self.is_armed and not was_armed:
                self._armed_since_time = time.time()
            elif not self.is_armed:
                self._armed_since_time = None
                self.rc_channels[2] = 1000
            self.rc_channels[4] = 2000 if self.is_armed else 1000
        
        # Handle MSP Override Mode status (AUX2)
        if len(joy_buttons) > self.override_switch_id:
            if self.lock_default_override:
                self.use_override = self.default_override
            else:
                self.use_override = joy_buttons[self.override_switch_id] == 1
            # In SITL, we keep AUX2 low (1000) so the flight controller does not enter BOXMSPOVERRIDE
            # and ignore our UDP RX stream.
            self.rc_channels[5] = 1000

        # Update manual RC channels if not in Override mode
        if not self.use_override:
            roll = joy_axes[0] if len(joy_axes) > 0 else 0.0
            pitch = joy_axes[1] if len(joy_axes) > 1 else 0.0
            yaw = joy_axes[2] if len(joy_axes) > 2 else 0.0
            throttle = joy_axes[3] if len(joy_axes) > 3 else -1.0

            self.rc_channels[0] = int(max(1000, min(2000, 1500 + roll * 500)))
            self.rc_channels[1] = int(max(1000, min(2000, 1500 + pitch * 500)))
            self.rc_channels[2] = int(max(1000, min(2000, 1500 + throttle * 500)))
            self.rc_channels[3] = int(max(1000, min(2000, 1500 + yaw * 500)))

    def poll_joy_udp(self):
        while True:
            try:
                payload, _ = self.joy_udp_sock.recvfrom(2048)
            except BlockingIOError:
                return
            except OSError:
                return

            try:
                msg = json.loads(payload.decode('utf-8'))
                joy_axes = [float(v) for v in msg.get('axes', [])]
                joy_buttons = [int(v) for v in msg.get('buttons', [])]
            except (ValueError, TypeError, json.JSONDecodeError):
                continue

            self.apply_joy(joy_axes, joy_buttons)

    def timer_callback(self):
        self.poll_joy_udp()

        # Publish throttle topic for monitoring
        throttle_msg = Float64()
        throttle_msg.data = float(self.rc_channels[2])
        self.throttle_pub.publish(throttle_msg)

        now = time.time()
        # Periodic logging at ~1Hz
        if now - self._last_log_time >= 1.0:
            self.get_logger().info(
                f"SITL RC: R:{self.rc_channels[0]} P:{self.rc_channels[1]} "
                f"T:{self.rc_channels[2]} Y:{self.rc_channels[3]} "
                f"A1:{self.rc_channels[4]} A2:{self.rc_channels[5]} {'(OVERRIDE)' if self.use_override else '(MANUAL)'}"
            )
            self._last_log_time = now

        # Pack rc_packet structure:
        # double timestamp (8 bytes)
        # uint16_t channels[16] (32 bytes)
        # Total size: 40 bytes
        timestamp = now - self.start_time
        pkt = struct.pack('<d16H', timestamp, *self.rc_channels)

        try:
            self.sock.sendto(pkt, (self.sitl_ip, self.sitl_rc_port))
        except Exception as e:
            self._udp_failures += 1
            if now - self._last_warning_time >= self.rc_warning_interval_s:
                self.get_logger().warning(
                    f"Failed to send SITL RC UDP packet to {self.sitl_ip}:{self.sitl_rc_port} "
                    f"({self._udp_failures} failures): {e}"
                )
                self._last_warning_time = now

    def destroy_node(self):
        self.sock.close()
        self.joy_udp_sock.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SitlRcBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
