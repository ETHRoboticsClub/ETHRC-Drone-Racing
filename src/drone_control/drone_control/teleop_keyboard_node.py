#!/usr/bin/env python3
"""
Keyboard Teleoperation Node for AIGP Drone Simulator.

Controls:
  w / s : Pitch Forward / Backward rate (deg/s)
  a / d : Roll Left / Right rate (deg/s)
  q / e : Yaw Left / Right rate (deg/s)
  i / k : Increase / Decrease Thrust (0.0 to 1.0, hover ~0.42)
  SPACE : Arm / Disarm drone
  r     : Reset simulation level
  x     : Zero all rates & set hover thrust
  CTRL+C: Exit teleop
"""

import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool, Trigger

BANNER = """
==================================================
   AIGP DRONE KEYBOARD TELEOPERATION CONTROL
==================================================
  w / s : Pitch Forward / Backward  (±30 deg/s)
  a / d : Roll Left / Right         (±30 deg/s)
  q / e : Yaw Left / Right          (±45 deg/s)
  i / k : Thrust Up / Down          (±0.05 step)
  SPACE : Toggle Arm / Disarm
  r     : Reset Simulation Level
  x     : Emergency Zero Rates / Hover
==================================================
"""

class TeleopKeyboardNode(Node):
    def __init__(self):
        super().__init__('teleop_keyboard_node')

        self.declare_parameter('cmd_topic', '/drone_attitude_cmd')
        self.declare_parameter('max_roll_rate_deg_s', 45.0)
        self.declare_parameter('max_pitch_rate_deg_s', 45.0)
        self.declare_parameter('max_yaw_rate_deg_s', 60.0)
        self.declare_parameter('hover_thrust', 0.42)

        cmd_topic = self.get_parameter('cmd_topic').value
        self.max_roll = float(self.get_parameter('max_roll_rate_deg_s').value)
        self.max_pitch = float(self.get_parameter('max_pitch_rate_deg_s').value)
        self.max_yaw = float(self.get_parameter('max_yaw_rate_deg_s').value)
        self.hover_thrust = float(self.get_parameter('hover_thrust').value)

        self.cmd_pub = self.create_publisher(Float64MultiArray, cmd_topic, 10)
        self.arm_client = self.create_client(SetBool, '/arm_drone')
        self.reset_client = self.create_client(Trigger, '/reset_sim')

        self.roll_rate = 0.0
        self.pitch_rate = 0.0
        self.yaw_rate = 0.0
        self.thrust = self.hover_thrust
        self.is_armed = False

        self.timer = self.create_timer(0.05, self.timer_callback) # 20 Hz teleop publish loop
        self.get_logger().info("Keyboard Teleop Node Initialized. Press keys in terminal to fly!")
        print(BANNER)

    def getKey(self, timeout=0.02):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], timeout)
            if rlist:
                key = sys.stdin.read(1)
            else:
                key = ''
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def toggle_arm(self):
        self.is_armed = not self.is_armed
        if self.arm_client.service_is_ready():
            req = SetBool.Request()
            req.data = self.is_armed
            self.arm_client.call_async(req)
            self.get_logger().info(f"Drone Arm State -> {self.is_armed}")

    def reset_sim(self):
        if self.reset_client.service_is_ready():
            req = Trigger.Request()
            self.reset_client.call_async(req)
            self.get_logger().info("Simulation Reset Call Sent.")

    def timer_callback(self):
        key = self.getKey(0.01)

        if key == 'w':
            self.pitch_rate = min(self.pitch_rate + 5.0, self.max_pitch)
        elif key == 's':
            self.pitch_rate = max(self.pitch_rate - 5.0, -self.max_pitch)
        elif key == 'a':
            self.roll_rate = max(self.roll_rate - 5.0, -self.max_roll)
        elif key == 'd':
            self.roll_rate = min(self.roll_rate + 5.0, self.max_roll)
        elif key == 'q':
            self.yaw_rate = max(self.yaw_rate - 10.0, -self.max_yaw)
        elif key == 'e':
            self.yaw_rate = min(self.yaw_rate + 10.0, self.max_yaw)
        elif key == 'i':
            self.thrust = min(self.thrust + 0.02, 1.0)
        elif key == 'k':
            self.thrust = max(self.thrust - 0.02, 0.0)
        elif key == ' ':
            self.toggle_arm()
        elif key == 'r':
            self.reset_sim()
        elif key == 'x':
            self.roll_rate = 0.0
            self.pitch_rate = 0.0
            self.yaw_rate = 0.0
            self.thrust = self.hover_thrust
        elif key == '\x03':  # CTRL+C
            rclpy.shutdown()
            return
        else:
            # Decay rates gradually if key released
            self.roll_rate *= 0.85
            self.pitch_rate *= 0.85
            self.yaw_rate *= 0.85

        # Publish command: [roll_deg_s, pitch_deg_s, yaw_deg_s, thrust]
        msg = Float64MultiArray()
        msg.data = [float(self.roll_rate), float(self.pitch_rate), float(self.yaw_rate), float(self.thrust)]
        self.cmd_pub.publish(msg)

        sys.stdout.write(
            f"\r[Teleop] Roll: {self.roll_rate:+5.1f}°/s | Pitch: {self.pitch_rate:+5.1f}°/s | "
            f"Yaw: {self.yaw_rate:+5.1f}°/s | Thrust: {self.thrust:4.2f} | Armed: {self.is_armed}  "
        )
        sys.stdout.flush()


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboardNode()
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
