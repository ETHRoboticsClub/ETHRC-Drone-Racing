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
        self._goal_done_event = threading.Event()  # Event to signal goal completion
        self._shutdown_requested = False  # Flag for graceful shutdown
        
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
        # Apply a Z offset (-0.15m) to safely compensate for the controller's steady-state hover dropping
        pose.pose.position.z = float(z) - 0.15
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
            self._goal_done_event.set()
            return

        self.get_logger().info('Goal accepted by server. Waiting for completion...')
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
        
        # Signal that goal is done (for interactive mode)
        self._goal_done_event.set()
        self._goal_handle = None
        
        # Only shutdown if requested
        if self._shutdown_requested:
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
    """
    Interactive mode: Prompts for pose goals in a loop, allowing sequential goal commands.
    Press Ctrl+C at any time to cancel the current goal and exit.
    """
    rclpy.init(args=args)
    
    client = DronePoseGoalClient()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(client)
    
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    
    def signal_handler(sig, frame):
        """Handle Ctrl+C by setting shutdown flag."""
        print('\n[INFO] Ctrl+C detected. Initiating shutdown...')
        client._shutdown_requested = True
        if client._goal_handle is not None:
            try:
                client.get_logger().info("Cancelling active goal...")
                cancel_future = client._action_client._cancel_goal_async(client._goal_handle)
            except Exception as e:
                client.get_logger().warn(f'Error cancelling goal: {e}')
    
    # Register signal handler for Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        while rclpy.ok() and not client._shutdown_requested:
            print("\n" + "="*60)
            print("INTERACTIVE POSE GOAL SENDER")
            print("="*60)
            
            try:
                # Prompt for goal parameters
                x_input = input("Enter X position (m) [default: 0.0]: ").strip()
                x = float(x_input) if x_input else 0.0
                
                y_input = input("Enter Y position (m) [default: 0.0]: ").strip()
                y = float(y_input) if y_input else 0.0
                
                z_input = input("Enter Z position (m) [default: -2.0]: ").strip()
                z = float(z_input) if z_input else -2.0
                
                yaw_input = input("Enter Yaw angle (degrees) [default: 90.0]: ").strip()
                yaw = float(yaw_input) if yaw_input else 90.0
                
                print(f"\n[INFO] Sending goal: x={x}, y={y}, z={z}, yaw={yaw}")
                
                # Cancel previous goal if active
                if client._goal_handle is not None and not client._goal_done_event.is_set():
                    print("\n[INFO] Cancelling previous goal to set a new one...")
                    try:
                        client._action_client._cancel_goal_async(client._goal_handle)
                        time.sleep(0.5)  # Wait briefly for server to abort previous goal
                    except Exception as e:
                        print(f"[WARNING] Cancel error: {e}")

                # Reset event for new goal
                client._goal_done_event.clear()
                
                # Send the goal
                client.send_goal(x, y, z, yaw, "goal_only")

                # Wait for the goal to finish (succeeded, canceled, or aborted)
                # before prompting for the next one, so the result message is
                # always printed and we never accidentally cancel a live goal.
                client._goal_done_event.wait()

                if client._shutdown_requested:
                    break
                    
            except ValueError as e:
                print(f"[ERROR] Invalid input: {e}. Please enter numeric values.")
                continue
            except KeyboardInterrupt:
                # Handle Ctrl+C during input
                print("\n[INFO] Ctrl+C detected during input. Exiting...")
                client._shutdown_requested = True
                break
    
    finally:
        # Cleanup
        print("\n[INFO] Shutting down...")
        client._shutdown_requested = True
        client.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("[INFO] Done.")


if __name__ == '__main__':
    main()