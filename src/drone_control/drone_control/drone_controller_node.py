# Add src directory to Python path for mpc, interfaces, rl imports
import sys
import os
src_path = os.path.expanduser('~/ros_ws/src')
if src_path not in sys.path:
    sys.path.insert(0, src_path)

# ros related
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node, Publisher, Subscription
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle as GoalHandle
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import time

# msgs
from std_msgs.msg import Float64MultiArray
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import Point, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

# drone
from my_mavsdk.mavsdk_client import AttitudeRate, PositionNedYaw
from interfaces import ControllerInterface
from mpc.drone_mpc_controller import DroneMpcController
from mpc.drone_mpc_controller_rotor import DroneMpcControllerRotor
from .drone_position_ned_controller import DronePositionNedController

try:
    from mpcc.drone_mpcc_controller import DroneMpccController
    MPCC_AVAILABLE = True
except ImportError:
    MPCC_AVAILABLE = False
    print("[WARNING] DroneMpccController not available (mpcc package not built)")

try:
    from rl.drone_rl_controller import DroneRLController
    RL_AVAILABLE = True
except ImportError:
    RL_AVAILABLE = False
    print("[WARNING] DroneRLController not available (torch not installed)")

# ── Config flags ──
START_AND_HOVER = True  # If True, auto-hover at 10 cm on launch

# general
import numpy as np
import threading
import yaml
from typing import Optional


class thresholds:
    position: float = 0.1
    velocity: float = 0.1
    angular_velocity: float = 0.1
    orientation: float = 0.1


def _build_registry() -> dict:
    """Build the controller registry from available controller classes."""
    registry: dict[str, type] = {
        'mpc':          DroneMpcController,
        'mpc_rotor':    DroneMpcControllerRotor,
        'position_ned': DronePositionNedController,
    }
    if MPCC_AVAILABLE:
        registry['mpcc'] = DroneMpccController
    if RL_AVAILABLE:
        registry['rl'] = DroneRLController
    return registry


_CONTROLLER_REGISTRY = _build_registry()


class DroneController(Node):

    controller: ControllerInterface

    # parameters
    control_rate: float
    controller_type: str
    thresholds_: thresholds = thresholds()

    # ros related
    state_subscriber: Subscription
    attitude_command_publisher: Publisher
    rotor_command_publisher: Publisher
    position_command_publisher: Publisher
    current_target_publisher: Publisher
    mpcc_ref_path_publisher: Publisher
    mpcc_horizon_publisher: Publisher
    position_goal_server: ActionServer
    trajectory_goal_server: ActionServer

    # states & goals
    state: np.ndarray = np.zeros(13)
    current_goal_state: np.ndarray = np.zeros(13)
    active_goal: bool = False
    hovering_active: bool = False
    stop_hover = threading.Event()

    # Misc
    state_lock: threading.Lock
    active_goal_lock: threading.Lock

    def __init__(self):
        super().__init__('drone_controller_node')

        self.active_goal_lock = threading.Lock()
        self.state_lock = threading.Lock()
        self.state[6] = 1.0
        self._state_received = False

        # cb groups
        self.action_cb_group = MutuallyExclusiveCallbackGroup()
        self.timer_cb_group = MutuallyExclusiveCallbackGroup()

        # parameters
        self.declare_parameter('controller_type', 'mpc')
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('state_topic', 'drone_state_sim')
        self.declare_parameter('attitude_command_topic', '/drone_attitude_cmd')
        self.declare_parameter('position_command_topic', '/drone_position_cmd')
        self.declare_parameter('controller_rate', 100.0)
        self.declare_parameter('threshold.positon', 0.1)  # typo kept for backwards compatibility
        self.declare_parameter('threshold.velocity', 0.02)
        self.declare_parameter('threshold.angular_velocity', 0.02)
        self.declare_parameter('threshold.orientation', 0.1)

        self.controller_type = self.get_parameter('controller_type').get_parameter_value().string_value
        state_topic = self.get_parameter('state_topic').get_parameter_value().string_value
        attitude_command_topic = self.get_parameter('attitude_command_topic').get_parameter_value().string_value
        position_command_topic = self.get_parameter('position_command_topic').get_parameter_value().string_value
        self.control_rate = self.get_parameter('controller_rate').get_parameter_value().double_value
        self.thresholds_.position = self.get_parameter('threshold.positon').get_parameter_value().double_value
        self.thresholds_.velocity = self.get_parameter('threshold.velocity').get_parameter_value().double_value
        self.thresholds_.angular_velocity = self.get_parameter('threshold.angular_velocity').get_parameter_value().double_value
        self.thresholds_.orientation = self.get_parameter('threshold.orientation').get_parameter_value().double_value

        # pub/sub
        self.state_subscriber = self.create_subscription(
            Float64MultiArray, state_topic, self.drone_state_cb, 10)
        self.attitude_command_publisher = self.create_publisher(
            Float64MultiArray, attitude_command_topic, 10)
        self.rotor_command_publisher = self.create_publisher(
            Float64MultiArray, '/drone_rotor_cmd', 10)
        self.position_command_publisher = self.create_publisher(
            Float64MultiArray, position_command_topic, 10)
        self.current_target_publisher = self.create_publisher(
            PoseStamped, '/drone_current_target',
            QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        self.mpcc_ref_path_publisher = self.create_publisher(
            MarkerArray, '/mpcc/reference_path',
            QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        self.mpcc_horizon_publisher = self.create_publisher(
            MarkerArray, '/mpcc/predicted_horizon', 10)
        self.mpcc_progress_point_publisher = self.create_publisher(
            MarkerArray, '/mpcc/progress_point', 10)
        self._mpcc_pub_counter = 0

        # ── Controller instantiation via registry ──────────────────────────
        waypoints_file = self.get_parameter('waypoints_file').get_parameter_value().string_value
        waypoints = self._load_waypoints(waypoints_file) if waypoints_file else None

        controller_class = _CONTROLLER_REGISTRY.get(self.controller_type)
        if controller_class is None:
            self.get_logger().error(
                f'Unknown or unavailable controller_type: "{self.controller_type}". '
                f'Falling back to MPC.')
            controller_class = DroneMpcController

        config = {'waypoints': waypoints}
        self.controller = controller_class.from_config(config)
        self.get_logger().info(f'Using controller: {controller_class.__name__}')
        # ──────────────────────────────────────────────────────────────────

        self.controller_rate = self.create_rate(self.control_rate)

        self.position_goal_server = rclpy.action.ActionServer(
            self, FollowWaypoints, 'drone_set_pose_goal',
            callback_group=self.action_cb_group,
            goal_callback=self.pose_goal_accept_callback,
            cancel_callback=self.pose_goal_cancel_callback,
            execute_callback=self.pose_goal_execute_callback)

        self.trajectory_goal_server = rclpy.action.ActionServer(
            self, FollowWaypoints, 'drone_set_trajectory_goal',
            callback_group=self.action_cb_group,
            goal_callback=self.trajectory_goal_accept_callback,
            cancel_callback=self.trajectory_goal_cancel_callback,
            execute_callback=self.trajectory_goal_execute_callback)

    # ── State callback ────────────────────────────────────────────────────

    def drone_state_cb(self, msg: Float64MultiArray):
        with self.state_lock:
            self.state = np.array(msg.data)
            self._state_received = True

    # ── Lifecycle ─────────────────────────────────────────────────────────

    def initialize(self):
        """Constructor returns fast; heavy initialization (optimizer build) happens here."""
        self.controller.initialize()

        # The initial auto-hover always uses MPC regardless of controller_type.
        if isinstance(self.controller, DroneMpcController):
            self._mpc_hover_controller = self.controller
        else:
            self._mpc_hover_controller = DroneMpcController()
            self._mpc_hover_controller.initialize()

        self.get_logger().info("Controller node initialized")

        if isinstance(self.controller, DroneMpccController):
            self._publish_mpcc_reference_path()

        if START_AND_HOVER:
            self.get_logger().info("Auto-hover will start once drone state is available")
            self._auto_hover_thread = threading.Thread(
                target=self._wait_for_state_and_hover, daemon=True)
            self._auto_hover_thread.start()

    def _wait_for_state_and_hover(self):
        """Wait for a valid state from MAVSDK, then start hovering at 10 cm altitude."""
        timeout = 60.0
        start = time.time()
        while time.time() - start < timeout:
            with self.state_lock:
                if self._state_received:
                    break
            time.sleep(0.1)
        else:
            self.get_logger().error("Timeout waiting for drone state. Auto-hover aborted.")
            return

        with self.state_lock:
            hover_state = self.state.copy()
        hover_state[2] = -0.1   # NED: 10 cm above ground
        hover_state[3:6] = 0.0  # zero velocity
        hover_state[10:13] = 0.0  # zero angular velocity
        # Keep current yaw only (zero roll and pitch)
        qw, qx, qy, qz = hover_state[6], hover_state[7], hover_state[8], hover_state[9]
        yaw_rad = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy**2 + qz**2))
        hover_state[6] = np.cos(yaw_rad / 2)
        hover_state[7] = 0.0
        hover_state[8] = 0.0
        hover_state[9] = np.sin(yaw_rad / 2)

        self.get_logger().info("Starting auto-hover at 10 cm altitude (NED z=-0.1)")
        threading.Thread(
            target=self.hover_at_currrent_position,
            args=(hover_state,),
            kwargs={'controller': self._mpc_hover_controller},
        ).start()

    # ── Hover ─────────────────────────────────────────────────────────────

    def ensure_not_hovering(self):
        if self.hovering_active:
            self.stop_hover.set()
            while self.hovering_active:
                time.sleep(5e-4)

    def hover_at_currrent_position(self, hover_state: Optional[np.ndarray] = None,
                                    controller: Optional[ControllerInterface] = None):
        self.hovering_active = True
        if hover_state is None:
            with self.state_lock:
                hover_state = self.state.copy()
        if controller is None:
            controller = self.controller

        # Project to yaw-only orientation (zero roll and pitch)
        qw, qx, qy, qz = hover_state[6], hover_state[7], hover_state[8], hover_state[9]
        yaw_rad = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        hover_state[6] = np.cos(yaw_rad / 2)
        hover_state[7] = 0.0
        hover_state[8] = 0.0
        hover_state[9] = np.sin(yaw_rad / 2)
        hover_state[3:6] = 0.0
        hover_state[10:13] = 0.0
        current_goal_state = hover_state

        while rclpy.ok() and not self.stop_hover.is_set():
            with self.state_lock:
                current_state = self.state.copy()
            cmd = controller.controller_update(current_state, current_goal_state)
            self._publish_command(cmd)
            self.controller_rate.sleep()

        self.stop_hover.clear()
        self.hovering_active = False

    # ── Waypoint-reached check ────────────────────────────────────────────

    def waypoint_reached(self) -> bool:
        with self.state_lock:
            current_state = self.state.copy()

        pos_error = np.linalg.norm(current_state[0:3] - self.current_goal_state[0:3])
        vel_error = np.linalg.norm(current_state[3:6] - self.current_goal_state[3:6])
        orientation_error = 2.0 * np.arccos(np.clip(
            np.dot(self.current_goal_state[6:10], current_state[6:10]), -1.0, 1.0))
        ang_vel_error = np.linalg.norm(current_state[10:13] - self.current_goal_state[10:13])

        return (pos_error < self.thresholds_.position and
                vel_error < self.thresholds_.velocity and
                orientation_error < self.thresholds_.orientation and
                ang_vel_error < self.thresholds_.angular_velocity)

    # ── Publishing ────────────────────────────────────────────────────────

    def _publish_mpcc_reference_path(self) -> None:
        """Publish the MPCC reference spline once as a latching MarkerArray."""
        pts = self.controller.sample_reference_path(n=300)
        m = Marker()
        m.header.frame_id = 'world'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'mpcc_reference'
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 0.06
        m.color.r, m.color.g, m.color.b, m.color.a = 0.8, 0.0, 1.0, 0.9
        for p in pts:
            m.points.append(Point(x=float(p[1]), y=float(p[0]), z=float(-p[2])))
        markers = MarkerArray()
        markers.markers.append(m)
        self.mpcc_ref_path_publisher.publish(markers)

    def _publish_mpcc_horizon(self) -> None:
        """Publish the MPCC predicted horizon at ~10 Hz (throttled)."""
        self._mpcc_pub_counter += 1
        if self._mpcc_pub_counter % 10 != 0:
            return
        pred = self.controller.last_predicted_positions
        if pred is None:
            return
        m = Marker()
        m.header.frame_id = 'world'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'mpcc_horizon'
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 0.1
        m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 1.0, 0.3, 0.9
        for p in pred:
            m.points.append(Point(x=float(p[1]), y=float(p[0]), z=float(-p[2])))
        markers = MarkerArray()
        markers.markers.append(m)
        self.mpcc_horizon_publisher.publish(markers)

    def _publish_mpcc_progress_point(self) -> None:
        """Publish the path reference point and contouring error line at every control step."""
        path_pos = self.controller.current_path_position  # NED (3,)

        now = self.get_clock().now().to_msg()
        markers = MarkerArray()

        # Yellow sphere at the path reference point
        sphere = Marker()
        sphere.header.frame_id = 'world'
        sphere.header.stamp = now
        sphere.ns = 'mpcc_progress'
        sphere.id = 0
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position.x = float(path_pos[1])    # NED east  → RViz x
        sphere.pose.position.y = float(path_pos[0])    # NED north → RViz y
        sphere.pose.position.z = float(-path_pos[2])   # NED down  → RViz z
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.2
        sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = 1.0, 1.0, 0.0, 1.0
        markers.markers.append(sphere)

        # Orange line: drone position → path reference point (contouring error)
        with self.state_lock:
            drone_pos = self.state[0:3].copy()

        line = Marker()
        line.header.frame_id = 'world'
        line.header.stamp = now
        line.ns = 'mpcc_progress'
        line.id = 1
        line.type = Marker.LINE_LIST
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = 0.03
        line.color.r, line.color.g, line.color.b, line.color.a = 1.0, 0.5, 0.0, 0.9
        line.points.append(Point(
            x=float(drone_pos[1]), y=float(drone_pos[0]), z=float(-drone_pos[2])))
        line.points.append(Point(
            x=float(path_pos[1]), y=float(path_pos[0]), z=float(-path_pos[2])))
        markers.markers.append(line)

        self.mpcc_progress_point_publisher.publish(markers)

    def _publish_command(self, cmd) -> None:
        """
        Publish a control command to the correct ROS topic.

        Dispatches on the type of cmd:
          - AttitudeRate    → attitude_command_publisher   (/drone_attitude_cmd)
          - PositionNedYaw  → position_command_publisher   (/drone_position_cmd)
          - anything else   → rotor_command_publisher      (/drone_rotor_cmd)
            (per-rotor thrust: np.ndarray (4,), used by mpc_rotor)
        """
        if isinstance(self.controller, DroneMpccController):
            self._publish_mpcc_horizon()
            self._publish_mpcc_progress_point()

        msg = Float64MultiArray()
        if isinstance(cmd, AttitudeRate):
            msg.data = [cmd.roll_deg_s, cmd.pitch_deg_s, cmd.yaw_deg_s, cmd.thrust_value]
            self.attitude_command_publisher.publish(msg)
        elif isinstance(cmd, PositionNedYaw):
            msg.data = [cmd.north_m, cmd.east_m, cmd.down_m, cmd.yaw_deg]
            self.position_command_publisher.publish(msg)
        else:
            # Per-rotor thrust: np.ndarray (4,) or similar sequence
            msg.data = [float(cmd[0]), float(cmd[1]), float(cmd[2]), float(cmd[3])]
            self.rotor_command_publisher.publish(msg)

    # ── Helpers ───────────────────────────────────────────────────────────

    def _load_waypoints(self, filepath: str) -> Optional[np.ndarray]:
        """
        Load waypoints from a YAML file.

        Expected format:
            waypoints:
              - {x: 1.0, y: 0.0, z: -1.0}
              - {x: 2.0, y: 1.0, z: -1.0}

        Returns an (N, 3) float64 array. Returns None if filepath is empty.
        Raises RuntimeError if the file is missing or malformed.
        """
        if not filepath:
            return None
        try:
            with open(filepath, 'r') as f:
                data = yaml.safe_load(f)
            pts = data['waypoints']
            return np.array([[p['x'], p['y'], p['z']] for p in pts], dtype=np.float64)
        except Exception as e:
            raise RuntimeError(f"Failed to load waypoints from '{filepath}': {e}")

    def _pose_stamped_to_goal_state(self, pose: PoseStamped) -> np.ndarray:
        """Convert a PoseStamped message to a 13-dim goal state vector."""
        goal = np.zeros(13)
        goal[0] = pose.pose.position.x
        goal[1] = pose.pose.position.y
        goal[2] = pose.pose.position.z
        goal[6] = pose.pose.orientation.w
        goal[7] = pose.pose.orientation.x
        goal[8] = pose.pose.orientation.y
        goal[9] = pose.pose.orientation.z
        return goal

    def terminate_action(self):
        with self.active_goal_lock:
            self.active_goal = False

    # ── Action: pose goal (single point) ─────────────────────────────────

    def pose_goal_accept_callback(self, goal: FollowWaypoints.Goal):
        """Accept a single-point goal. Rejects if another goal is active or if >1 waypoint is sent."""
        self.get_logger().info('[Pose Goal] Received goal request')
        with self.active_goal_lock:
            if self.active_goal:
                self.get_logger().error('[Pose Goal] Controller already has an active goal. Rejecting.')
                return GoalResponse.REJECT
            if not goal.poses:
                self.get_logger().error('[Pose Goal] Empty goal received.')
                return GoalResponse.REJECT
            if len(goal.poses) != 1:
                self.get_logger().error(
                    '[Pose Goal] Multiple waypoints received. Use the trajectory interface instead.')
                return GoalResponse.REJECT
            self.active_goal = True
        return GoalResponse.ACCEPT

    def pose_goal_cancel_callback(self, goal: FollowWaypoints.Goal):
        self.get_logger().info('[Pose Goal] Received cancel request')
        return CancelResponse.ACCEPT

    def pose_goal_execute_callback(self, goal_handle: GoalHandle):
        """
        Mode A — single point: fly to goal and hover there.
        Calls controller_update() in a loop until waypoint_reached().
        """
        goal_poses: FollowWaypoints.Goal = goal_handle.request
        pose = goal_poses.poses[0]
        self.current_target_publisher.publish(pose)
        self.current_goal_state = self._pose_stamped_to_goal_state(pose)

        result = FollowWaypoints.Result()
        result.missed_waypoints = []
        self.ensure_not_hovering()

        while rclpy.ok() and not goal_handle.is_cancel_requested and not self.waypoint_reached():
            with self.state_lock:
                current_state = self.state.copy()
            cmd = self.controller.controller_update(current_state, self.current_goal_state)
            self._publish_command(cmd)
            self.controller_rate.sleep()

        self.get_logger().info('[Pose Goal] Exiting control loop')
        threading.Thread(target=self.hover_at_currrent_position).start()

        if goal_handle.is_cancel_requested and not self.waypoint_reached():
            result.missed_waypoints = [0]
            goal_handle.canceled()
            self.get_logger().info('[Pose Goal] Request cancelled')
            self.terminate_action()
            return result

        goal_handle.succeed()
        self.get_logger().info('[Pose Goal] Goal succeeded')
        self.terminate_action()
        return result

    # ── Action: trajectory goal (waypoint sequence) ───────────────────────

    def trajectory_goal_accept_callback(self, goal: FollowWaypoints.Goal):
        """Accept a multi-waypoint goal. Rejects if another goal is active."""
        self.get_logger().info('[Trajectory Goal] Received goal request')
        with self.active_goal_lock:
            if self.active_goal:
                self.get_logger().error(
                    '[Trajectory Goal] Controller already has an active goal. Rejecting.')
                return GoalResponse.REJECT
            if not goal.poses:
                self.get_logger().error('[Trajectory Goal] Empty goal received.')
                return GoalResponse.REJECT
            self.active_goal = True
        return GoalResponse.ACCEPT

    def trajectory_goal_cancel_callback(self, goal: FollowWaypoints.Goal):
        self.get_logger().info('[Trajectory Goal] Received cancel request')
        return CancelResponse.ACCEPT

    def trajectory_goal_execute_callback(self, goal_handle: GoalHandle):
        """
        Mode B — waypoint sequence: visit each waypoint in order, looping forever.
        Restarts from the first waypoint after the last one is reached.
        Exits only when a cancel request is received.
        """
        goal_poses: FollowWaypoints.Goal = goal_handle.request
        waypoints = goal_poses.poses

        result = FollowWaypoints.Result()
        result.missed_waypoints = []
        self.ensure_not_hovering()

        lap = 0
        while rclpy.ok() and not goal_handle.is_cancel_requested:
            lap += 1
            self.get_logger().info(f'[Trajectory Goal] Starting lap {lap}')

            for i, pose in enumerate(waypoints):
                if goal_handle.is_cancel_requested:
                    break

                self.current_target_publisher.publish(pose)
                self.current_goal_state = self._pose_stamped_to_goal_state(pose)
                self.get_logger().info(
                    f'[Trajectory Goal] Lap {lap} — waypoint {i + 1}/{len(waypoints)}')

                while rclpy.ok() and not goal_handle.is_cancel_requested and not self.waypoint_reached():
                    with self.state_lock:
                        current_state = self.state.copy()
                    cmd = self.controller.controller_update(current_state, self.current_goal_state)
                    self._publish_command(cmd)
                    self.controller_rate.sleep()

        self.get_logger().info('[Trajectory Goal] Cancelled — hovering')
        threading.Thread(target=self.hover_at_currrent_position).start()

        result.missed_waypoints = [0]
        goal_handle.canceled()
        self.terminate_action()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = DroneController()
    node.initialize()

    executor = MultiThreadedExecutor()
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
