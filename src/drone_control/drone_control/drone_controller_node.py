"""
RL drone controller node.

Topic wiring:

    drone_state_sim   ──► this node (state)
        published by:
          - src/my_mavsdk/my_mavsdk/mavsdk_client.py        (SITL / MAVSDK path)
          - src/drone_control/drone_control/state_bridge_node.py  (hardware: Vicon + IMU)
        payload: std_msgs/Float64MultiArray, 13 floats
            [0:3]   pos x,y,z          (world)
            [3:6]   vel x,y,z          (world)
            [6:10]  quat x,y,z,w       (xyzw, NOT wxyz)
            [10:13] body angular vel   (rad/s)

    drone_goal        ──► this node (target)
        publish a geometry_msgs/Point with x,y,z (world frame) to set the goal.
        Goal is sticky: it only changes when a new message arrives.
        Goals outside the workspace bounds are rejected.

    drone_attitude_cmd ◄── this node (command)
        subscribed by src/my_mavsdk/my_mavsdk/mavsdk_client.py, which forwards
        to MAVSDK offboard.set_attitude_rate → Betaflight via MSP.
        payload: std_msgs/Float64MultiArray, 4 floats
            [roll_deg_s, pitch_deg_s, yaw_deg_s, thrust 0..1]

Safety model:
    The control loop only publishes when ALL of the following hold:
      - a goal has been received,
      - the latest state is fresh (age < state_timeout_sec),
      - the controller call succeeds.
    In any other case the node stays silent so MAVSDK keeps its last command
    or hits its own failsafe — it never auto-commands zero thrust mid-flight.
"""

import threading
import time
from typing import Optional

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Point

from rl.drone_rl_controller import DroneRLController


class DroneController(Node):
    """
    Bridges drone state → RL policy → attitude-rate commands.

    Stays silent (no publishes) until a valid goal is set AND a fresh state
    message is available. See module docstring for the safety model.
    """

    def __init__(self):
        super().__init__('drone_controller_node')

        # Topic defaults are relative so they follow the node namespace.
        self.declare_parameter('state_topic', 'drone_state_sim')
        self.declare_parameter('attitude_command_topic', 'drone_attitude_cmd')
        self.declare_parameter('goal_topic', 'drone_goal')
        self.declare_parameter('controller_rate', 100.0)
        self.declare_parameter('observation_topic', 'drone_observation')

        # Watchdog: max age of the latest state before we stop publishing.
        self.declare_parameter('state_timeout_sec', 0.1)

        # Workspace bounds (world frame, metres). A goal is rejected if any
        # axis falls outside [min, max]. Tune to your arena before flying.
        self.declare_parameter('goal_bounds.x_min', -5.0)
        self.declare_parameter('goal_bounds.x_max',  5.0)
        self.declare_parameter('goal_bounds.y_min', -5.0)
        self.declare_parameter('goal_bounds.y_max',  5.0)
        self.declare_parameter('goal_bounds.z_min',  0.0)
        self.declare_parameter('goal_bounds.z_max',  3.0)

        state_topic = self.get_parameter('state_topic').get_parameter_value().string_value
        attitude_command_topic = self.get_parameter('attitude_command_topic').get_parameter_value().string_value
        goal_topic = self.get_parameter('goal_topic').get_parameter_value().string_value
        control_rate = self.get_parameter('controller_rate').get_parameter_value().double_value
        self.state_timeout_sec = self.get_parameter('state_timeout_sec').get_parameter_value().double_value
        observation_topic = self.get_parameter('observation_topic').get_parameter_value().string_value

        self.goal_bounds = (
            np.array([
                self.get_parameter('goal_bounds.x_min').get_parameter_value().double_value,
                self.get_parameter('goal_bounds.y_min').get_parameter_value().double_value,
                self.get_parameter('goal_bounds.z_min').get_parameter_value().double_value,
            ]),
            np.array([
                self.get_parameter('goal_bounds.x_max').get_parameter_value().double_value,
                self.get_parameter('goal_bounds.y_max').get_parameter_value().double_value,
                self.get_parameter('goal_bounds.z_max').get_parameter_value().double_value,
            ]),
        )

        self.controller = DroneRLController()
        self.get_logger().info("Using RL Controller")

        self.state_lock = threading.Lock()
        self.goal_lock = threading.Lock()

        self.state: np.ndarray = np.zeros(13)
        self.state[9] = 1.0  # quat w for identity rotation
        self.last_state_time: Optional[float] = None  # monotonic seconds
        self.goal: Optional[np.ndarray] = None

        self.state_subscriber = self.create_subscription(
            Float64MultiArray, state_topic, self.drone_state_cb, 10)
        self.goal_subscriber = self.create_subscription(
            Point, goal_topic, self.goal_cb, 10)
        self.attitude_command_publisher = self.create_publisher(
            Float64MultiArray, attitude_command_topic, 10)
        self.observation_publisher = self.create_publisher(
            Float64MultiArray, observation_topic, 10)

        self.create_timer(1.0 / control_rate, self.control_loop)

    def initialize(self):
        self.controller.initialize()
        self.get_logger().info("Controller node initialized")

    def drone_state_cb(self, msg: Float64MultiArray):
        """Get the latest state, after shape and finite-value checks."""
        data = np.array(msg.data)
        if data.shape[0] != 13:
            self.get_logger().error(
                f"Ignoring state message: expected 13 elements, got {data.shape[0]}")
            return
        if not np.isfinite(data).all():
            self.get_logger().error(
                "Ignoring state message: contains NaN or Inf")
            return
        with self.state_lock:
            self.state = data
            self.last_state_time = time.monotonic()

    def goal_cb(self, msg: Point):
        """Replace the current goal, if it lies inside the workspace bounds.

        Goals are persistent: nothing else mutates `self.goal`, so the drone
        continues tracking the last accepted target indefinitely.
        """
        new_goal = np.array([msg.x, msg.y, msg.z])
        if not np.isfinite(new_goal).all():
            self.get_logger().error(
                f"Rejecting goal: non-finite values {new_goal.tolist()}")
            return
        lo, hi = self.goal_bounds
        if np.any(new_goal < lo) or np.any(new_goal > hi):
            self.get_logger().error(
                f"Rejecting goal {new_goal.tolist()}: outside bounds "
                f"x[{lo[0]},{hi[0]}] y[{lo[1]},{hi[1]}] z[{lo[2]},{hi[2]}]")
            return
        with self.goal_lock:
            self.goal = new_goal
        self.get_logger().info(f"New goal accepted: {new_goal.tolist()}")

    def control_loop(self):
        """Timer callback: produce one attitude-rate command per tick.

        Publishes only when goal is set AND state is fresh AND controller
        succeeds. Any other case → silent (MAVSDK keeps its last command
        or trips its own failsafe.
        """
        with self.goal_lock:
            goal = None if self.goal is None else self.goal.copy()
        if goal is None:
            return

        with self.state_lock:
            current_state = self.state.copy()
            last_state_time = self.last_state_time

        if last_state_time is None:
            self.get_logger().warn(
                "No state received yet; holding command.",
                throttle_duration_sec=1.0)
            return
        age = time.monotonic() - last_state_time
        if age > self.state_timeout_sec:
            self.get_logger().warn(
                f"State stale ({age*1000:.0f} ms > {self.state_timeout_sec*1000:.0f} ms); holding command.",
                throttle_duration_sec=1.0)
            return

        try:
            attitude_cmd = self.controller.controller_update(current_state, goal)
        except Exception as e:
            self.get_logger().error(f'Control Loop Error: {e}')
            return

        cmd_msg = Float64MultiArray()
        cmd_msg.data = [
            float(attitude_cmd.roll_deg_s),
            float(attitude_cmd.pitch_deg_s),
            float(attitude_cmd.yaw_deg_s),
            float(attitude_cmd.thrust_value),
        ]
        self.attitude_command_publisher.publish(cmd_msg)

        # Publish observation vector if available
        if getattr(self.controller, 'last_observation', None) is not None:
            obs_msg = Float64MultiArray()
            obs_msg.data = [float(val) for val in self.controller.last_observation]
            self.observation_publisher.publish(obs_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DroneController()
    node.initialize()

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
