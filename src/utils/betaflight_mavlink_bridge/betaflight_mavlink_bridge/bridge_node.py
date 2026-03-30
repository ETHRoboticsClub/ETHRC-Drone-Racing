#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray
from pymavlink import mavutil
import time
import threading

class BetaflightMavlinkBridge(Node):
    def __init__(self):
        super().__init__('bridge_node')

        # Parameters
        self.declare_parameter('device', '/dev/ttyTHS1')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('chan_roll', 1)
        self.declare_parameter('chan_pitch', 2)
        self.declare_parameter('chan_throttle', 3)
        self.declare_parameter('chan_yaw', 4)
        self.declare_parameter('auto_switch_index', 4)  # Switch for Auto mode
        self.declare_parameter('mode_channel', 6)       # RC Channel to signal mode to FC
        self.declare_parameter('arm_switch_index', 5)   # Switch for Arming
        self.declare_parameter('arm_channel', 5)        # RC Channel for Arming

        self.device = self.get_parameter('device').get_parameter_value().string_value
        self.baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.chan_roll = self.get_parameter('chan_roll').get_parameter_value().integer_value
        self.chan_pitch = self.get_parameter('chan_pitch').get_parameter_value().integer_value
        self.chan_throttle = self.get_parameter('chan_throttle').get_parameter_value().integer_value
        self.chan_yaw = self.get_parameter('chan_yaw').get_parameter_value().integer_value
        self.auto_switch_index = self.get_parameter('auto_switch_index').get_parameter_value().integer_value
        self.mode_channel = self.get_parameter('mode_channel').get_parameter_value().integer_value
        self.arm_switch_index = self.get_parameter('arm_switch_index').get_parameter_value().integer_value
        self.arm_channel = self.get_parameter('arm_channel').get_parameter_value().integer_value

        # State
        self.is_auto = False
        self.is_armed = False
        self.rc_channels = [1500] * 8  # Default center for 8 channels
        self.rc_channels[self.chan_throttle - 1] = 1000  # Low throttle by default
        self.rc_channels[self.arm_channel - 1] = 1000    # Disarmed by default
        self.rc_channels[self.mode_channel - 1] = 1000   # Manual mode by default

        self.last_joy_msg = None
        self.last_auto_msg = None

        # MAVLink setup
        self.get_logger().info(f"Connecting to Betaflight on {self.device} at {self.baud} baud...")
        try:
            self.master = mavutil.mavlink_connection(self.device, baud=self.baud)
            # Wait for heartbeat
            self.get_logger().info("Waiting for heartbeat...")
            self.master.wait_heartbeat(timeout=5)
            self.get_logger().info("Connected to Flight Controller.")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to MAVLink: {e}")
            # We continue but the node won't be able to send commands
            self.master = None

        # Subscribers
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.auto_sub = self.create_subscription(Float64MultiArray, '/drone_attitude_cmd', self.auto_callback, 10)

        # Heartbeat and Command Timer (50Hz)
        self.timer = self.create_timer(0.02, self.timer_callback)
        
        # Start heartbeat thread
        self.stop_heartbeat = False
        self.heartbeat_thread = threading.Thread(target=self.send_heartbeat)
        self.heartbeat_thread.start()

    def send_heartbeat(self):
        while not self.stop_heartbeat:
            if self.master:
                self.master.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, 0)
            time.sleep(1.0)

    def joy_callback(self, msg):
        self.last_joy_msg = msg
        
        # Check Mode Switch
        if self.auto_switch_index < len(msg.buttons):
            new_auto = msg.buttons[self.auto_switch_index] == 1
            if new_auto != self.is_auto:
                self.is_auto = new_auto
                self.get_logger().info(f"Mode changed: {'AUTONOMOUS' if self.is_auto else 'MANUAL'}")
        
        # Check Arm Switch
        if self.arm_switch_index < len(msg.buttons):
            new_arm = msg.buttons[self.arm_switch_index] == 1
            if new_arm != self.is_armed:
                self.is_armed = new_arm
                self.get_logger().info(f"Arm status changed: {'ARMED' if self.is_armed else 'DISARMED'}")

        # Update manual RC channels if in Manual mode
        if not self.is_auto:
            # AETR mapping (typically Axes are: R:0, P:1, T:2 (or 3), Y:3 (or 2))
            # We assume a standard mapping for now, but this should be configurable
            # Let's use indices: R:0, P:1, T:3, Y:2 as a common default
            roll = msg.axes[0] if len(msg.axes) > 0 else 0.0
            pitch = msg.axes[1] if len(msg.axes) > 1 else 0.0
            yaw = msg.axes[2] if len(msg.axes) > 2 else 0.0
            throttle = msg.axes[3] if len(msg.axes) > 3 else -1.0 # Default low

            self.rc_channels[self.chan_roll - 1] = int(1500 + roll * 500)
            self.rc_channels[self.chan_pitch - 1] = int(1500 + pitch * 500)
            self.rc_channels[self.chan_yaw - 1] = int(1500 + yaw * 500)
            self.rc_channels[self.chan_throttle - 1] = int(1500 + throttle * 500)

        # Update Aux channels
        self.rc_channels[self.arm_channel - 1] = 2000 if self.is_armed else 1000
        self.rc_channels[self.mode_channel - 1] = 2000 if self.is_auto else 1000

    def auto_callback(self, msg):
        self.last_auto_msg = msg
        if self.is_auto and len(msg.data) >= 4:
            # Expecting [Roll, Pitch, Yaw, Thrust] in normalized range [-1, 1]
            roll, pitch, yaw, thrust = msg.data[:4]
            self.rc_channels[self.chan_roll - 1] = int(1500 + roll * 500)
            self.rc_channels[self.chan_pitch - 1] = int(1500 + pitch * 500)
            self.rc_channels[self.chan_yaw - 1] = int(1500 + yaw * 500)
            self.rc_channels[self.chan_throttle - 1] = int(1500 + thrust * 500)

    def timer_callback(self):
        if not self.master:
            return

        # Constrain all channels to 1000-2000
        channels = [max(1000, min(2000, c)) for c in self.rc_channels]
        
        # Pad with 65535 (ignore) for channels 9-18
        full_channels = channels + [65535] * (18 - len(channels))

        try:
            self.master.mav.rc_channels_override_send(
                self.master.target_system,
                self.master.target_component,
                *full_channels
            )
        except Exception as e:
            self.get_logger().error(f"Error sending RC Override: {e}")

    def destroy_node(self):
        self.stop_heartbeat = True
        self.heartbeat_thread.join()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = BetaflightMavlinkBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
