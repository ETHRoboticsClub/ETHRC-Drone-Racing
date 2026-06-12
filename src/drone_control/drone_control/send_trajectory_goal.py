import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
# The Time message is needed for the stamp, but let's assume we use the Node's clock
from nav2_msgs.action import FollowWaypoints
import sys
import argparse
import math
import yaml
import os

# --- Default Config Path ---
# NOTE: In a real ROS 2 setup, you would use rclpy.get_parameters_from_file or 
# a parameter file, but for a single file example, we'll hardcode or use a CLI arg.
DEFAULT_CONFIG_PATH = os.path.expanduser('~/ros_ws/src/drone_control/config/waypoints_config.yaml') 

class DroneWaypointGoalClient(Node):
    def __init__(self, config_path=DEFAULT_CONFIG_PATH):
        super().__init__('drone_waypoint_goal_client')
        
        # --- Action Client for FollowWaypoints ---
        # The Action name should align with the server, assuming 'drone_set_trajectory_goal'
        self._action_client = ActionClient(
            self,
            FollowWaypoints,
            'drone_set_trajectory_goal'
        )
        
        self._goal_handle: ClientGoalHandle = None
        self._cancel_timer = None 
        self._cancel_delay_sec = 0.0 # Will be set by CLI argument
        self.waypoints_data = self._read_waypoints_config(config_path)

        self.get_logger().info("Drone Waypoint Goal Client Initialized.")
        
    def _read_waypoints_config(self, config_path):
        """Reads waypoints from the specified YAML config file."""
        if not os.path.exists(config_path):
            self.get_logger().error(f"Config file not found: {config_path}")
            rclpy.shutdown()
            return None
        
        self.get_logger().info(f"Reading waypoints from: {config_path}")
        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
            
            waypoints = config.get('waypoints', [])
            if not waypoints:
                self.get_logger().error("No 'waypoints' list found in config file.")
                rclpy.shutdown()
                return None

            return waypoints

        except Exception as e:
            self.get_logger().error(f"Error reading or parsing YAML file: {e}")
            rclpy.shutdown()
            return None

    # --- Utility Function ---
    def euler_to_quaternion(self, roll, pitch, yaw):
        """Converts roll, pitch, yaw (in radians) to geometry_msgs/Quaternion."""
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)

        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q

    # --- Action Goal Handling ---
    def send_waypoints_goal(self, mode, cancel_delay_sec=0.0):
        if self.waypoints_data is None:
            return

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action Server 'drone_set_trajectory_goal' not available.")
            return

        self._cancel_delay_sec = cancel_delay_sec
        goal_msg = FollowWaypoints.Goal()
        
        waypoints_list = []
        for wp in self.waypoints_data:
            # 1. Convert Yaw to Quaternion (default to 0.0 if not specified)
            yaw_rad = math.radians(wp.get('yaw_deg', 0.0))
            q = self.euler_to_quaternion(0.0, 0.0, yaw_rad)
            
            # 2. Create PoseStamped message
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            
            # *** IMPORTANT MODIFICATION FOR time_from_beginning ***
            # Set the stamp to be relative to the *start* of the action, 
            # represented by a time offset. The time_from_beginning field 
            # is assumed to be in seconds.
            # We convert seconds to ROS 2 time message fields (sec, nanosec).
            total_sec = wp.get('time_from_beginning', 0.0)
            pose.header.stamp.sec = int(total_sec)
            pose.header.stamp.nanosec = int((total_sec - int(total_sec)) * 1e9)
            # The receiver (Action Server) must interpret this stamp as a desired
            # time offset from when the goal was received/started.

            pose.pose.position.x = float(wp['x'])
            pose.pose.position.y = float(wp['y'])
            pose.pose.position.z = float(wp['z'])
            pose.pose.orientation = q
            
            waypoints_list.append(pose)

        # 3. Assign the list of PoseStamped messages to the goal
        goal_msg.poses = waypoints_list
        
        self.get_logger().info(f'Sending goal with {len(waypoints_list)} waypoints.')
        
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        
        # Pass the mode flag to the response callback
        self._send_goal_future.add_done_callback(
            lambda future: self.goal_response_callback(future, mode)
        )

    def goal_response_callback(self, future, mode):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by server.')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted. Waiting for result...')
        self._goal_handle = goal_handle # Store the handle
        
        if mode == 'cancel':
            # --- START CANCELLATION TIMER ---
            self.get_logger().warn(f'Mode "cancel": Setting up timer to cancel in {self._cancel_delay_sec:.2f} seconds.')
            # Create a one-shot timer (period=delay, callback=send_cancel_action)
            if self._cancel_delay_sec > 0:
                 self._cancel_timer = self.create_timer(self._cancel_delay_sec, self.send_cancel_action)
            else:
                 self.get_logger().warn("Cancel delay is 0 or less, cancelling immediately.")
                 self.send_cancel_action()

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        # Clean up the timer if it hasn't fired yet
        if self._cancel_timer is not None:
            self.destroy_timer(self._cancel_timer)
            self._cancel_timer = None

        status = future.result().status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Goal SUCCEEDED! ✅ All waypoints reached.')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn(f'Goal was CANCELED. 🛑')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error(f'Goal ABORTED. ❌')
        else:
            self.get_logger().info(f'Goal finished with status: {status}')
        
        # Shutdown the node after result is final
        rclpy.shutdown()


    # --- Cancel Command (Called by Timer) ---
    def send_cancel_action(self):
        """Sends a cancellation request for the last accepted goal."""
        
        # Destroy the timer immediately if it was created
        if self._cancel_timer is not None:
            self.destroy_timer(self._cancel_timer)
            self._cancel_timer = None

        if self._goal_handle is None or self._goal_handle.status not in [GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING]:
            self.get_logger().error("Cancellation called, but no active/accepted goal to cancel.")
            return

        self.get_logger().info(f"!!! CANCEL TRIGGERED: Sending cancellation request after {self._cancel_delay_sec:.2f} seconds !!!")
        
        self._action_client._cancel_goal_async(self._goal_handle)
        
        # The get_result_callback handles the final CANCELED status.

# --- Main Execution ---

def main(args=None):
    rclpy.init(args=args)
    
    # --- Parse command line arguments ---
    parser = argparse.ArgumentParser(description="Send a waypoint trajectory or a cancel command to the drone.")
    
    # 1. New arguments: config file path
    parser.add_argument('config_file', type=str, default=DEFAULT_CONFIG_PATH, nargs='?', 
                        help=f'Path to the YAML config file defining waypoints (default: {DEFAULT_CONFIG_PATH}).')

    # 2. Simplified mode argument
    parser.add_argument('mode', choices=['goal_only', 'cancel'], 
                        help='Mode of operation: "goal_only" (execute trajectory) or "cancel" (execute trajectory and then cancel).')
    
    # 3. New argument: time after start for cancellation
    parser.add_argument('--cancel_delay', type=float, default=2.0, 
                        help='Time in seconds after goal acceptance to send the cancel request (only used if mode is "cancel").')
    
    # Isolate ROS arguments
    ros_args = [arg for arg in sys.argv if not arg.startswith('__')]
    parsed_args = parser.parse_args(ros_args[1:] if len(ros_args) > 1 else [])

    # Instantiate the client, which reads the config
    client = DroneWaypointGoalClient(parsed_args.config_file)

    # Send the goal with the list of waypoints
    client.send_waypoints_goal(parsed_args.mode, parsed_args.cancel_delay)
    
    # Spin the node to process the action client futures and the cancellation timer
    rclpy.spin(client) 
        
    client.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()