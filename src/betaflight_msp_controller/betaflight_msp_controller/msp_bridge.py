#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64
from sensor_msgs.msg import Joy
import errno
import os
import serial
import struct
import time

class MspBridge(Node):
    def __init__(self):
        super().__init__('msp_bridge')
        
        # Parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200) # Updated for USB-VCP
        self.declare_parameter('arm_switch_id', 4)  # AUX1
        self.declare_parameter('override_switch_id', 5) # AUX2
        self.declare_parameter('attitude_cmd_topic', '/drone_attitude_cmd')
        self.declare_parameter('max_rate_deg_s', 500.0)
        self.declare_parameter('roll_rate_sign', -1.0)
        self.declare_parameter('pitch_rate_sign', -1.0)
        self.declare_parameter('yaw_rate_sign', 1.0)
        self.declare_parameter('write_timeout_s', 0.0)
        self.declare_parameter('serial_warning_interval_s', 1.0)
        
        self.port_name = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.arm_switch_id = self.get_parameter('arm_switch_id').value
        self.override_switch_id = self.get_parameter('override_switch_id').value
        self.attitude_cmd_topic = self.get_parameter('attitude_cmd_topic').value
        self.max_rate_deg_s = self.get_parameter('max_rate_deg_s').value
        self.roll_rate_sign = self.get_parameter('roll_rate_sign').value
        self.pitch_rate_sign = self.get_parameter('pitch_rate_sign').value
        self.yaw_rate_sign = self.get_parameter('yaw_rate_sign').value
        self.write_timeout_s = self.get_parameter('write_timeout_s').value
        self.serial_warning_interval_s = self.get_parameter('serial_warning_interval_s').value
        
        # RC State [Roll, Pitch, Throttle, Yaw, AUX1, AUX2, AUX3, AUX4]
        self.rc_channels = [1500, 1500, 1000, 1500, 1000, 1500, 1500, 1500]
        self.is_armed = False
        self.use_override = False
        self._last_log_time = 0.0
        self._last_serial_warning_time = 0.0
        self._serial_write_failures = 0

        try:
            self.serial_port = serial.Serial(
                self.port_name,
                self.baudrate,
                timeout=0.1,
                write_timeout=self.write_timeout_s,
            )
            self.get_logger().info(
                f'Opened MSP RX port {self.port_name} at {self.baudrate} '
                f'(write timeout {self.write_timeout_s:.3f}s)'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to open MSP RX port {self.port_name}: {e}')
            self.serial_port = None

        # Publishers
        self.throttle_pub = self.create_publisher(Float64, '/drone/throttle', 10)

        # Subscribers
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.att_sub = self.create_subscription(
            Float64MultiArray, 
            self.attitude_cmd_topic, 
            self.attitude_cmd_callback, 
            10)
        
        # Timer (50Hz for RC heartbeat)
        self.timer = self.create_timer(0.02, self.timer_callback)
        
        self.get_logger().info(
            'MSP Bridge Initialized (Handles Joy and Attitude Cmd); '
            f'RC rate signs roll={self.roll_rate_sign:+.0f}, '
            f'pitch={self.pitch_rate_sign:+.0f}, yaw={self.yaw_rate_sign:+.0f}'
        )

    def attitude_cmd_callback(self, msg):
        if len(msg.data) < 4:
            return
            
        # msg.data = [roll_rate, pitch_rate, yaw_rate, thrust]
        roll_rate = self.roll_rate_sign * msg.data[0]
        pitch_rate = self.pitch_rate_sign * msg.data[1]
        yaw_rate = self.yaw_rate_sign * msg.data[2]
        thrust = msg.data[3]

        # Update RC stick channels (AETR)
        # Roll, Pitch, Yaw: 1500 +/- 500
        # Throttle: 1000 + thrust * 1000 (0.0 to 1.0)
        self.rc_channels[0] = int(max(1000, min(2000, 1500 + (roll_rate / self.max_rate_deg_s) * 500)))
        self.rc_channels[1] = int(max(1000, min(2000, 1500 + (pitch_rate / self.max_rate_deg_s) * 500)))
        self.rc_channels[3] = int(max(1000, min(2000, 1500 + (yaw_rate / self.max_rate_deg_s) * 500)))
        self.rc_channels[2] = int(max(1000, min(2000, 1000 + thrust * 1000)))

    def joy_callback(self, msg):
        # Handle Arming (AUX1)
        if len(msg.buttons) > self.arm_switch_id:
            self.is_armed = msg.buttons[self.arm_switch_id] == 1
            self.rc_channels[4] = 2000 if self.is_armed else 1000
        
        # Handle MSP Override Mode status (AUX2) - We still track it for logging, but don't gate
        if len(msg.buttons) > self.override_switch_id:
            self.use_override = msg.buttons[self.override_switch_id] == 1
            self.rc_channels[5] = 2000 if self.use_override else 1000

    def timer_callback(self):
        throttle_msg = Float64()
        throttle_msg.data = float(self.rc_channels[2])
        self.throttle_pub.publish(throttle_msg)

        if not self.serial_port or not self.serial_port.is_open:
            return

        # Periodic logging at ~1Hz
        now = time.time()
        if now - self._last_log_time >= 1.0:
            self.get_logger().info(
                f"Sending MSP RC: R:{self.rc_channels[0]} P:{self.rc_channels[1]} "
                f"T:{self.rc_channels[2]} Y:{self.rc_channels[3]} "
                f"A1:{self.rc_channels[4]} A2:{self.rc_channels[5]} {'(OVERRIDE)' if self.use_override else '(MANUAL)'}"
            )
            self._last_log_time = now

        msg = self.build_msp_set_raw_rc()

        try:
            written = os.write(self.serial_port.fileno(), msg)
            if written != len(msg):
                raise BlockingIOError(
                    errno.EAGAIN,
                    f'partial write ({written}/{len(msg)} bytes)'
                )
        except Exception as e:
            self._serial_write_failures += 1
            try:
                self.serial_port.reset_output_buffer()
            except Exception:
                pass

            if now - self._last_serial_warning_time >= self.serial_warning_interval_s:
                self.get_logger().warn(
                    f"MSP serial output is not draining on {self.port_name}; "
                    f"dropped stale TX buffer ({self._serial_write_failures} failures): {e}"
                )
                self._last_serial_warning_time = now

    def build_msp_set_raw_rc(self):
        # MSP_SET_RAW_RC (200)
        payload = struct.pack('<8H', *self.rc_channels)
        size = len(payload)
        cmd = 200
        cs = size ^ cmd
        for b in payload:
            cs ^= b
        return b'$M<' + struct.pack('<BB', size, cmd) + payload + struct.pack('<B', cs)

    def destroy_node(self):
        if self.serial_port:
            self.serial_port.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MspBridge()
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
