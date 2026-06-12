#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray
import time
import numpy as np

class PolicyVerifier(Node):
    def __init__(self):
        super().__init__('policy_verifier')
        self.goal_pub = self.create_publisher(Point, '/drone_goal', 10)
        self.cmd_sub = self.create_subscription(Float64MultiArray, '/drone_attitude_cmd', self.cmd_callback, 10)
        self.received_cmds = []

    def cmd_callback(self, msg):
        self.received_cmds.append(msg.data)

    def test_goal(self, x, y, z, name):
        self.received_cmds = []
        goal_msg = Point()
        goal_msg.x = float(x)
        goal_msg.y = float(y)
        goal_msg.z = float(z)
        self.goal_pub.publish(goal_msg)
        self.get_logger().info(f"Published goal {name}: [{x}, {y}, {z}]")
        
        # Spin the node to process callbacks for 1.0 second
        start_time = time.monotonic()
        while time.monotonic() - start_time < 1.0:
            rclpy.spin_once(self, timeout_sec=0.01)
        
        if not self.received_cmds:
            self.get_logger().warn("No commands received!")
            return None
            
        cmds = np.array(self.received_cmds)
        mean_cmds = np.mean(cmds, axis=0)
        self.get_logger().info(f"Mean response for {name}:")
        self.get_logger().info(f"  Roll rate:  {mean_cmds[0]:.2f} deg/s")
        self.get_logger().info(f"  Pitch rate: {mean_cmds[1]:.2f} deg/s")
        self.get_logger().info(f"  Yaw rate:   {mean_cmds[2]:.2f} deg/s")
        self.get_logger().info(f"  Thrust:     {mean_cmds[3]:.4f}")
        return mean_cmds

def main():
    rclpy.init()
    node = PolicyVerifier()
    
    # Spin node for 2 seconds to allow network connections to establish
    start_time = time.monotonic()
    while time.monotonic() - start_time < 2.0:
        rclpy.spin_once(node, timeout_sec=0.05)
    
    print("\n=== STARTING POLICY RESPONSE VERIFICATION ===\n")
    
    # 1. Base/Hover Goal
    node.test_goal(0.0, 0.0, 1.0, "HOVER")
    
    # 2. Altitude changes
    node.test_goal(0.0, 0.0, 1.8, "HIGH_ALTITUDE")
    node.test_goal(0.0, 0.0, 0.5, "LOW_ALTITUDE")
    
    # 3. Y position (Lateral movement)
    node.test_goal(0.0, -1.0, 1.0, "LEFT_GOAL")
    node.test_goal(0.0, 1.0, 1.0, "RIGHT_GOAL")
    
    # 4. X position (Longitudinal movement)
    node.test_goal(1.0, 0.0, 1.0, "FORWARD_GOAL")
    node.test_goal(-1.0, 0.0, 1.0, "BACKWARD_GOAL")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
