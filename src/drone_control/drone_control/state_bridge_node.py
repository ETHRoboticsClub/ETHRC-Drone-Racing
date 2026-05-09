#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray

class StateBridgeNode(Node):
    def __init__(self):
        super().__init__('state_bridge_node')
        
        self.declare_parameter('odom_topic', '/ekf/odometry')
        self.declare_parameter('state_topic', 'drone_state_sim')
        
        odom_topic = self.get_parameter('odom_topic').value
        state_topic = self.get_parameter('state_topic').value
        
        self.subscription = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10)
            
        self.publisher = self.create_publisher(
            Float64MultiArray,
            state_topic,
            10)
            
        self.get_logger().info(f'State Bridge started: {odom_topic} -> {state_topic}')

    def odom_callback(self, msg: Odometry):
        # Extract 13-element state: [p(3), v(3), q(4), w(3)]
        # ROS Odometry uses:
        # pose.pose.position (x, y, z)
        # pose.pose.orientation (x, y, z, w)
        # twist.twist.linear (vx, vy, vz) - usually in child_frame (body)
        # twist.twist.angular (wx, wy, wz) - usually in child_frame (body)
        
        state_msg = Float64MultiArray()
        state_msg.data = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z
        ]
        self.publisher.publish(state_msg)

def main(args=None):
    rclpy.init(args=args)
    node = StateBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
