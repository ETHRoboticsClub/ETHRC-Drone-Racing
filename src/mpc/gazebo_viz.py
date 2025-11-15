#!/usr/bin/env python3
"""
Gazebo visualization for MPC reference trajectory and predictions.
Publishes ROS markers to visualize target position and predicted path.
"""
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import numpy as np


class GazeboMPCVisualizer(Node):
    """ROS2 node for visualizing MPC data in Gazebo."""
    
    def __init__(self):
        super().__init__('gazebo_mpc_visualizer')
        
        # Publishers
        self.marker_pub = self.create_publisher(MarkerArray, '/mpc_visualization', 10)
        
        # Timer for publishing
        self.timer = self.create_timer(0.1, self.publish_markers)  # 10 Hz
        
        # Data storage
        self.target_position = np.array([1.0, 1.0, -2.0])  # Default target
        self.predicted_trajectory = None
        self.current_position = None
        
        self.get_logger().info("Gazebo MPC Visualizer started")
    
    def update_target(self, target_pos):
        """Update target position."""
        self.target_position = np.array(target_pos)
    
    def update_trajectory(self, predicted_states):
        """Update predicted trajectory from MPC."""
        if predicted_states is not None and len(predicted_states) > 0:
            # Extract positions (first 3 elements of each state)
            self.predicted_trajectory = predicted_states[:, 0:3]
    
    def update_current_position(self, current_pos):
        """Update current drone position."""
        self.current_position = np.array(current_pos)
    
    def publish_markers(self):
        """Publish visualization markers."""
        markers = MarkerArray()
        
        # 1. Target position marker (large red sphere)
        target_marker = Marker()
        target_marker.header.frame_id = "map"
        target_marker.header.stamp = self.get_clock().now().to_msg()
        target_marker.ns = "mpc_target"
        target_marker.id = 0
        target_marker.type = Marker.SPHERE
        target_marker.action = Marker.ADD
        
        # Position (convert from NED to ENU for ROS/Gazebo)
        target_marker.pose.position.x = float(self.target_position[1])  # East
        target_marker.pose.position.y = float(self.target_position[0])  # North  
        target_marker.pose.position.z = float(-self.target_position[2]) # Up (negate down)
        target_marker.pose.orientation.w = 1.0
        
        # Size and color
        target_marker.scale.x = 0.3
        target_marker.scale.y = 0.3
        target_marker.scale.z = 0.3
        target_marker.color.r = 1.0
        target_marker.color.g = 0.0
        target_marker.color.b = 0.0
        target_marker.color.a = 0.8
        
        markers.markers.append(target_marker)
        
        # 2. Predicted trajectory (line strip)
        if self.predicted_trajectory is not None and len(self.predicted_trajectory) > 1:
            traj_marker = Marker()
            traj_marker.header.frame_id = "map"
            traj_marker.header.stamp = self.get_clock().now().to_msg()
            traj_marker.ns = "mpc_trajectory"
            traj_marker.id = 1
            traj_marker.type = Marker.LINE_STRIP
            traj_marker.action = Marker.ADD
            
            # Add points (convert NED to ENU)
            for i, pos in enumerate(self.predicted_trajectory):
                point = Point()
                point.x = float(pos[1])   # East
                point.y = float(pos[0])   # North
                point.z = float(-pos[2])  # Up
                traj_marker.points.append(point)
                
                # Color gradient from blue to green
                color = ColorRGBA()
                alpha = i / len(self.predicted_trajectory)
                color.r = 0.0
                color.g = alpha
                color.b = 1.0 - alpha
                color.a = 0.7
                traj_marker.colors.append(color)
            
            # Line properties
            traj_marker.scale.x = 0.05  # Line width
            traj_marker.pose.orientation.w = 1.0
            
            markers.markers.append(traj_marker)
        
        # 3. Current position marker (small blue sphere)
        if self.current_position is not None:
            current_marker = Marker()
            current_marker.header.frame_id = "map"
            current_marker.header.stamp = self.get_clock().now().to_msg()
            current_marker.ns = "mpc_current"
            current_marker.id = 2
            current_marker.type = Marker.SPHERE
            current_marker.action = Marker.ADD
            
            # Position (convert NED to ENU)
            current_marker.pose.position.x = float(self.current_position[1])  # East
            current_marker.pose.position.y = float(self.current_position[0])  # North
            current_marker.pose.position.z = float(-self.current_position[2]) # Up
            current_marker.pose.orientation.w = 1.0
            
            # Size and color
            current_marker.scale.x = 0.15
            current_marker.scale.y = 0.15
            current_marker.scale.z = 0.15
            current_marker.color.r = 0.0
            current_marker.color.g = 0.0
            current_marker.color.b = 1.0
            current_marker.color.a = 0.9
            
            markers.markers.append(current_marker)
        
        # Publish all markers
        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    
    visualizer = GazeboMPCVisualizer()
    
    try:
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        pass
    
    visualizer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()