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
import signal
import threading
import time

# Define the delay for cancellation in seconds
CANCEL_DELAY_SEC = 1.0

class DronePoseGoalClient(Node):
    def __init__(self):
        super().__init__('drone_pose_goal_client')
        
        # --- Action Client for Pose Goal ---
        self._action_client = ActionClient(
            self,
            FollowWaypoints,
            'drone_set_pose_goal'
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
        """Send a ROS ENU/FLU goal: x=east, y=north, z=up, yaw CCW from east."""
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action Server 'drone_set_pose_goal' not available.")
            rclpy.shutdown()
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
        # ROS ENU: z is up. Bias 15 cm low to compensate for steady-state hover drift.
        pose.pose.position.z = float(z) - 0.15
        pose.pose.orientation = q
        
        # 3. Assign to goal (FollowWaypoints expects a list of PoseStamped)
        goal_msg.poses = [pose]
        
        self.get_logger().info(
            f'[MODE: {mode}] Sending ENU goal to east={x:.1f}, north={y:.1f}, up={z:.1f}, yaw={yaw_deg} deg')
        
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        
        # Pass the mode flag to the response callback
        self._send_goal_future.add_done_callback(
            lambda future: self.goal_response_callback(future, mode)
        )

    def goal_response_callback(self, future, mode):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning('Goal rejected by server.')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted. Waiting for result...')
        self._goal_handle = goal_handle # Store the handle
        
        if mode == 'cancel':
            # --- START CANCELLATION TIMER ---
            self.get_logger().warning(f'Mode "cancel": Setting up timer to cancel in {CANCEL_DELAY_SEC} seconds.')
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
            self.get_logger().warning(f'Goal was CANCELED. 🛑')
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
        if self._goal_handle is None:
            self.get_logger().error("Cancellation called, but no active goal to cancel.")
            return

        self.get_logger().info("!!! SENDING CANCELLATION REQUEST !!!")
        
        # 2. Send the cancel request
        self._action_client.cancel_goal_async(self._goal_handle)

# --- Main Execution ---

def main(args=None):
    rclpy.init(args=args)

    # Parse command line arguments
    parser = argparse.ArgumentParser(description="Send a ROS ENU pose goal or cancel command to the drone.")
    parser.add_argument('mode', choices=['goal_only', 'cancel'], help='Mode of operation: "goal_only" or "cancel".')
    parser.add_argument('-x', type=float, default=1.0, help='Target east position in map/ENU (m).')
    parser.add_argument('-y', type=float, default=0.0, help='Target north position in map/ENU (m).')
    parser.add_argument('-z', type=float, default=2.0, help='Target up altitude in map/ENU (m).')
    parser.add_argument('-yaw', type=float, default=90.0, help='Target ENU yaw angle, CCW from east (degrees).')
    
    ros_args = [arg for arg in sys.argv if not arg.startswith('__')]
    parsed_args = parser.parse_args(ros_args[1:] if len(ros_args) > 1 else [])

    client = DronePoseGoalClient()

    def signal_handler(sig, frame):
        """On Ctrl+C: send a cancel to the server so it clears active_goal,
        then shut down.  Without this the server rejects the very next goal."""
        client.get_logger().warning('[SIGINT] Ctrl+C — cancelling active goal before exit...')
        if client._goal_handle is not None:
            try:
                cancel_future = client._goal_handle.cancel_goal_async()
                # Spin briefly so the cancel request is actually transmitted.
                deadline = time.monotonic() + 2.0
                while not cancel_future.done() and time.monotonic() < deadline:
                    rclpy.spin_once(client, timeout_sec=0.05)
                if cancel_future.done():
                    client.get_logger().info('[SIGINT] Cancel acknowledged by server ✅')
                else:
                    client.get_logger().warning('[SIGINT] Cancel timed out — server may still have an active goal')
            except Exception as exc:
                client.get_logger().warning(f'[SIGINT] Cancel failed: {exc}')
        else:
            client.get_logger().info('[SIGINT] No active goal to cancel.')
        if rclpy.ok():
            rclpy.shutdown()

    signal.signal(signal.SIGINT, signal_handler)

    # Pass the mode argument to send_goal
    client.send_goal(parsed_args.x, parsed_args.y, parsed_args.z, parsed_args.yaw, parsed_args.mode)

    # Spin the node to process the action client futures and the cancellation timer.
    # KeyboardInterrupt is suppressed because our SIGINT handler takes care of cleanup.
    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            client.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
