import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from std_srvs.srv import Empty
# Assuming nav2_msgs.action.FollowWaypoints is the correct action type
from nav2_msgs.action import FollowWaypoints 
import sys
import argparse
import math

# Define the delay for cancellation in seconds
CANCEL_DELAY_SEC = 0.4

class DronePoseGoalClient(Node):
    def __init__(self):
        super().__init__('drone_pose_goal_client')
        
        # --- Action Client for Pose Goal ---
        self._action_client = ActionClient(
            self,
            FollowWaypoints,
            'drone_set_trajectory_goal'
        )
        
        self._goal_handle: ClientGoalHandle = None
        self._cancel_timer = None # Timer object for timed cancellation
        
        self.get_logger().info("Drone Pose Goal Client Initialized.")
        
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
    def send_goal(self, x, y, z, yaw_deg, mode):
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action Server 'drone_set_pose_goal' not available.")
            return

        goal_msg = FollowWaypoints.Goal()
        
        # 1. Convert Yaw to Quaternion
        yaw_rad = math.radians(yaw_deg)
        q = self.euler_to_quaternion(0.0, 0.0, yaw_rad)
        
        # 2. Create PoseStamped message
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        pose.pose.orientation = q
        
        # 3. Assign to goal (FollowWaypoints expects a list of PoseStamped)
        goal_msg.poses = [pose]
        
        self.get_logger().info(f'[MODE: {mode}] Sending goal to ({x:.1f}, {y:.1f}, {z:.1f}) with Yaw {yaw_deg}°')
        
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
            self.get_logger().warn(f'Mode "cancel": Setting up timer to cancel in {CANCEL_DELAY_SEC} seconds.')
            # Create a one-shot timer (period=delay, callback=send_cancel_action)
            self._cancel_timer = self.create_timer(CANCEL_DELAY_SEC, self.send_cancel_action)

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        # Clean up the timer if it hasn't fired yet
        if self._cancel_timer is not None:
            self.destroy_timer(self._cancel_timer)
            self._cancel_timer = None

        status = future.result().status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Goal SUCCEEDED! ✅')
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
        
        # Destroy the timer immediately so it only runs once
        if self._cancel_timer is not None:
            self.destroy_timer(self._cancel_timer)
            self._cancel_timer = None

        # 1. Check if there's an active goal handle stored
        if self._goal_handle is None or self._goal_handle.status != GoalStatus.STATUS_EXECUTING:
            self.get_logger().error("Cancellation called, but no active goal to cancel.")
            return

        self.get_logger().info("!!! TIMED CANCEL TRIGGERED: Sending cancellation request now !!!")
        
        # 2. Send the cancel request
        cancel_future = self._action_client._cancel_goal_async(self._goal_handle)
        
        # Note: We rely on the get_result_callback to handle the final CANCELED status
        # and shut down the node.

# --- Main Execution ---

def main(args=None):
    rclpy.init(args=args)
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="Send a pose goal or cancel command to the drone.")
    parser.add_argument('mode', choices=['goal_only', 'cancel'], help='Mode of operation: "goal_only" or "cancel".')
    parser.add_argument('-x', type=float, default=1.0, help='Target X position (m).')
    parser.add_argument('-y', type=float, default=0.0, help='Target Y position (m).')
    parser.add_argument('-z', type=float, default=2.0, help='Target Z position (m).')
    parser.add_argument('-yaw', type=float, default=90.0, help='Target Yaw angle (degrees).')
    
    ros_args = [arg for arg in sys.argv if not arg.startswith('__')]
    parsed_args = parser.parse_args(ros_args[1:] if len(ros_args) > 1 else [])

    client = DronePoseGoalClient()

    # Pass the mode argument to send_goal
    client.send_goal(parsed_args.x, parsed_args.y, parsed_args.z, parsed_args.yaw, parsed_args.mode)
    
    # Spin the node to process the action client futures and the cancellation timer
    rclpy.spin(client) 
        
    client.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()