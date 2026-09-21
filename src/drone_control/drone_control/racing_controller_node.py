# Add src directory to Python path for mpc, interfaces, rl imports
import sys
import os

current_file = os.path.abspath(__file__)
current_dir = os.path.dirname(current_file)

# Try resolving via source structure (works for symlink install)
src_path = os.path.abspath(os.path.join(current_dir, '..', '..'))

# Try resolving via standard install structure (works for normal install)
src_path_standard = os.path.abspath(os.path.join(current_dir, '..', '..', '..', '..', '..', '..', 'src'))

# Fallback search paths
fallback_paths = [
    os.path.expanduser('~/proj/ETHRC-Drone-Racing/src'),
    os.path.expanduser('~/ros_ws/src'),
]

inserted = False
for path in [src_path, src_path_standard] + fallback_paths:
    if os.path.isdir(path) and os.path.exists(os.path.join(path, 'interfaces', '__init__.py')):
        if path not in sys.path:
            sys.path.insert(0, path)
            # Also insert mpcc_pp if it exists under src
            mpcc_pp_path = os.path.join(path, 'mpcc_pp')
            if os.path.isdir(mpcc_pp_path) and mpcc_pp_path not in sys.path:
                sys.path.insert(0, mpcc_pp_path)
        inserted = True
        break

# ros related
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node, Publisher, Subscription
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle as GoalHandle
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import time
import math

# msgs
from std_msgs.msg import Float64MultiArray
from nav2_msgs.action import FollowWaypoints
from std_srvs.srv import SetBool
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
    from mpcc_pp.drone_mpcc_pp_controller import DroneMpccPpController
    MPCC_PP_AVAILABLE = True
except ImportError:
    MPCC_PP_AVAILABLE = False
    print("[WARNING] DroneMpccPpController not available (mpcc_pp package not built)")

_MPCC_CLASSES = tuple(filter(None, [
    DroneMpccController if MPCC_AVAILABLE else None,
    DroneMpccPpController if MPCC_PP_AVAILABLE else None,
]))

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
from enum import Enum
from typing import Optional


class thresholds:
    position: float = 0.1
    velocity: float = 0.1
    angular_velocity: float = 0.1
    orientation: float = 0.1


class StateType(Enum):
    POSE_STAMPED = 'PoseStamped'
    FLOAT64_MULTI_ARRAY = 'Float64MultiArray'


_ENU_TO_NED = np.array([
    [0.0, 1.0, 0.0],
    [1.0, 0.0, 0.0],
    [0.0, 0.0, -1.0],
])
_FLU_TO_FRD = np.diag([1.0, -1.0, -1.0])


def _parse_state_type(state_msg_type: str) -> StateType:
    if state_msg_type == 'PoseStamped':
        return StateType.POSE_STAMPED
    if state_msg_type == 'Float64MultiArray':
        return StateType.FLOAT64_MULTI_ARRAY
    return StateType.POSE_STAMPED


def _build_registry() -> dict:
    """Build the controller registry from available controller classes."""
    registry: dict[str, type] = {
        'mpc':          DroneMpcController,
        'mpc_rotor':    DroneMpcControllerRotor,
        'position_ned': DronePositionNedController,
    }
    if MPCC_AVAILABLE:
        registry['mpcc'] = DroneMpccController
    if MPCC_PP_AVAILABLE:
        registry['mpcc_pp'] = DroneMpccPpController
    if RL_AVAILABLE:
        registry['rl'] = DroneRLController
    return registry


def _quaternion_angle_error(q_a: np.ndarray, q_b: np.ndarray) -> float:
    """Return the smallest angle between two quaternions [w, x, y, z]."""
    q_a_norm = np.linalg.norm(q_a)
    q_b_norm = np.linalg.norm(q_b)
    if q_a_norm < 1e-12 or q_b_norm < 1e-12:
        return np.inf
    dot = np.dot(q_a / q_a_norm, q_b / q_b_norm)
    return 2.0 * np.arccos(np.clip(np.abs(dot), -1.0, 1.0))


def _normalize_quat(q: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(q)
    if norm < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / norm


def _quat_to_rot(q: np.ndarray) -> np.ndarray:
    qw, qx, qy, qz = _normalize_quat(q)
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
        [2 * (qx * qy + qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qw * qx)],
        [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx * qx + qy * qy)],
    ])


def _rot_to_quat(r: np.ndarray) -> np.ndarray:
    trace = np.trace(r)
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (r[2, 1] - r[1, 2]) / s
        qy = (r[0, 2] - r[2, 0]) / s
        qz = (r[1, 0] - r[0, 1]) / s
    else:
        i = int(np.argmax(np.diag(r)))
        if i == 0:
            s = np.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
            qw = (r[2, 1] - r[1, 2]) / s
            qx = 0.25 * s
            qy = (r[0, 1] + r[1, 0]) / s
            qz = (r[0, 2] + r[2, 0]) / s
        elif i == 1:
            s = np.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
            qw = (r[0, 2] - r[2, 0]) / s
            qx = (r[0, 1] + r[1, 0]) / s
            qy = 0.25 * s
            qz = (r[1, 2] + r[2, 1]) / s
        else:
            s = np.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
            qw = (r[1, 0] - r[0, 1]) / s
            qx = (r[0, 2] + r[2, 0]) / s
            qy = (r[1, 2] + r[2, 1]) / s
            qz = 0.25 * s
    return _normalize_quat(np.array([qw, qx, qy, qz]))


def _pose_frame_is_ned(frame_id: str) -> bool:
    return frame_id.strip('/').lower() in ('ned', 'map_ned', 'world_ned')


def _pose_stamped_to_ned_frd_state(pose: PoseStamped) -> np.ndarray:
    """Convert a ROS PoseStamped into controller state [NED p, zero v, FRD q, zero w]."""
    state = np.zeros(13, dtype=np.float64)
    raw_pos = np.array([
        pose.pose.position.x,
        pose.pose.position.y,
        pose.pose.position.z,
    ], dtype=np.float64)
    q_world_body = np.array([
        pose.pose.orientation.w,
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
    ], dtype=np.float64)

    if _pose_frame_is_ned(pose.header.frame_id):
        state[0:3] = raw_pos
        state[6:10] = _normalize_quat(q_world_body)
    else:
        state[0:3] = _ENU_TO_NED @ raw_pos
        state[6:10] = _rot_to_quat(_ENU_TO_NED @ _quat_to_rot(q_world_body) @ _FLU_TO_FRD.T)
    return state


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
    mpcc_tunnel_publisher: Publisher
    mpcc_horizon_publisher: Publisher
    mpcc_rotor_thrust_publisher: Publisher
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
        self.declare_parameter('state_msg_type', 'PoseStamped')
        self.declare_parameter('simulation_mode', False)
        self.declare_parameter('use_mavsdk', True)
        self.declare_parameter('publish_simulator_action', False)
        self.declare_parameter('simulator_action_topic', '/mpcc_pp/mpc_action')
        self.declare_parameter('mpcc_stabilization_sec', 3.0)
        self.declare_parameter('attitude_command_topic', '/drone_attitude_cmd')
        self.declare_parameter('position_command_topic', '/drone_position_cmd')
        self.declare_parameter('controller_rate', 100.0)
        self.declare_parameter('verbose_input', False)
        self.declare_parameter('model_logging', False)
        self.declare_parameter('residual_model_path', '')
        self.declare_parameter('print_mpc_inputs', False)
        self.declare_parameter('mpc_input_debug_interval_sec', 1.0)
        self.declare_parameter('speed_report_interval_sec', 5.0)
        self.declare_parameter('optuna_progress_interval_sec', 0.5)
        self.declare_parameter('optuna_crash_down_threshold_m', -0.02)
        self.declare_parameter('optuna_airborne_down_threshold_m', -0.05)
        self.declare_parameter('optuna_crash_detection_delay_sec', 5.0)
        self.declare_parameter('optuna_ground_contact_duration_sec', 1.0)
        self.declare_parameter('threshold.positon', 0.1)  # typo kept for backwards compatibility
        self.declare_parameter('threshold.position', 1.0)  # typo kept for backwards compatibility, was 0.02
        self.declare_parameter('threshold.velocity', 0.02)
        self.declare_parameter('threshold.angular_velocity', 0.02)
        self.declare_parameter('threshold.orientation', 0.05)

        self.controller_type = self.get_parameter('controller_type').get_parameter_value().string_value
        state_topic = self.get_parameter('state_topic').get_parameter_value().string_value
        state_msg_type = self.get_parameter('state_msg_type').get_parameter_value().string_value
        self._simulation_mode = (
            self.get_parameter('simulation_mode').get_parameter_value().bool_value)
        self._use_mavsdk = (
            self.get_parameter('use_mavsdk').get_parameter_value().bool_value)
        self._publish_simulator_action = (
            self.get_parameter('publish_simulator_action').get_parameter_value().bool_value)
        simulator_action_topic = (
            self.get_parameter('simulator_action_topic').get_parameter_value().string_value)
        self._mpcc_stabilization_s = max(
            0.0,
            self.get_parameter('mpcc_stabilization_sec').get_parameter_value().double_value)
        self.state_type = _parse_state_type(state_msg_type)
        if self.state_type.value != state_msg_type:
            self.get_logger().warn(
                f'Unknown state_msg_type: "{state_msg_type}". Falling back to PoseStamped.')
        attitude_command_topic = self.get_parameter('attitude_command_topic').get_parameter_value().string_value
        position_command_topic = self.get_parameter('position_command_topic').get_parameter_value().string_value
        self.control_rate = self.get_parameter('controller_rate').get_parameter_value().double_value
        self._verbose = self.get_parameter('verbose_input').get_parameter_value().bool_value
        self._model_logging = self.get_parameter('model_logging').get_parameter_value().bool_value
        self._residual_model_path = self.get_parameter('residual_model_path').get_parameter_value().string_value
        self._print_mpc_inputs = self.get_parameter('print_mpc_inputs').get_parameter_value().bool_value
        self._mpc_input_debug_interval_s = max(
            0.1,
            self.get_parameter('mpc_input_debug_interval_sec').get_parameter_value().double_value)
        self._last_mpc_input_debug_time = 0.0
        self._publish_debug_counter = 0
        self._speed_report_interval_s = max(
            0.1,
            self.get_parameter('speed_report_interval_sec').get_parameter_value().double_value)
        self._speed_interval_start = None
        self._speed_sum_kmh = 0.0
        self._speed_max_kmh = 0.0
        self._speed_samples = 0
        self._optuna_progress_interval_s = max(
            0.1,
            self.get_parameter(
                'optuna_progress_interval_sec').get_parameter_value().double_value)
        self._optuna_crash_down_threshold_m = self.get_parameter(
            'optuna_crash_down_threshold_m').get_parameter_value().double_value
        self._optuna_airborne_down_threshold_m = self.get_parameter(
            'optuna_airborne_down_threshold_m').get_parameter_value().double_value
        self._optuna_crash_detection_delay_s = max(
            0.0,
            self.get_parameter(
                'optuna_crash_detection_delay_sec').get_parameter_value().double_value)
        self._optuna_ground_contact_duration_s = max(
            0.0,
            self.get_parameter(
                'optuna_ground_contact_duration_sec').get_parameter_value().double_value)
        self._optuna_progress_last_report = 0.0
        self._optuna_liftoff_time = None
        self._optuna_ground_contact_start = None
        self._optuna_crash_reported = False
        self._optuna_last_gate_progress = 0.0
        self._optuna_checked_gate_crossings: set[int] = set()
        self._optuna_missed_gates = 0
        self._path_timer_start = None
        self._last_lap_count = 0
        self.thresholds_.position = self.get_parameter('threshold.position').get_parameter_value().double_value
        self.thresholds_.velocity = self.get_parameter('threshold.velocity').get_parameter_value().double_value
        self.thresholds_.angular_velocity = self.get_parameter('threshold.angular_velocity').get_parameter_value().double_value
        self.thresholds_.orientation = self.get_parameter('threshold.orientation').get_parameter_value().double_value

        # pub/sub
        if self.state_type == StateType.POSE_STAMPED:
            self.state_subscriber = self.create_subscription(
                PoseStamped, state_topic, self.drone_state_pose_stamped_cb, 10)
        else:
            self.state_subscriber = self.create_subscription(
                Float64MultiArray, state_topic, self.drone_state_float64_multi_array_cb, 10)
        self.attitude_command_publisher = self.create_publisher(
            Float64MultiArray, attitude_command_topic, 10)
        self.rotor_command_publisher = self.create_publisher(
            Float64MultiArray, '/drone_rotor_cmd', 10)
        self.position_command_publisher = self.create_publisher(
            Float64MultiArray, position_command_topic, 10)
        self.simulator_action_publisher = self.create_publisher(
            Float64MultiArray, simulator_action_topic, 10)
        self.current_target_publisher = self.create_publisher(
            PoseStamped, '/drone_current_target',
            QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        self.mpcc_ref_path_publisher = self.create_publisher(
            MarkerArray, '/mpcc/reference_path',
            QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        self.mpcc_tunnel_publisher = self.create_publisher(
            MarkerArray, '/mpcc/tunnel',
            QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        self.mpcc_horizon_publisher = self.create_publisher(
            MarkerArray, '/mpcc/predicted_horizon', 10)
        self.mpcc_progress_point_publisher = self.create_publisher(
            MarkerArray, '/mpcc/progress_point', 10)
        self.mpcc_rotor_thrust_publisher = self.create_publisher(
            Float64MultiArray, '/mpcc/optimized_rotor_thrust', 10)
        self._mpcc_pub_counter = 0

        # ── Controller instantiation via registry ──────────────────────────
        waypoints_file = self.get_parameter('waypoints_file').get_parameter_value().string_value
        config = self._load_track_config(waypoints_file) if waypoints_file else {}

        controller_class = _CONTROLLER_REGISTRY.get(self.controller_type)
        if controller_class is None:
            self.get_logger().error(
                f'Unknown or unavailable controller_type: "{self.controller_type}". '
                f'Falling back to MPC.')
            controller_class = DroneMpcController

        self.controller = controller_class.from_config(config)
        self.get_logger().info(f'Using controller: {controller_class.__name__}')
        if hasattr(self.controller, 'verbose'):
            self.controller.verbose = self._verbose
        if self._model_logging and isinstance(self.controller, _MPCC_CLASSES):
            self.controller.enable_model_logging()
            self.get_logger().info('Model-validation logging enabled')
        if self._residual_model_path and isinstance(self.controller, _MPCC_CLASSES):
            if hasattr(self.controller, 'enable_residual_correction'):
                self.controller.enable_residual_correction(self._residual_model_path)
                self.get_logger().info(
                    f'Residual correction enabled: {self._residual_model_path}')
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

    def drone_state_float64_multi_array_cb(self, msg: Float64MultiArray):
        data = np.asarray(msg.data, dtype=np.float64)
        if data.shape == (19,):
            state = self._augmented_mpcc_state_to_external(data)
            controller = getattr(self, 'controller', None)
            sync_augmented_state = getattr(controller, 'sync_augmented_state', None)
            if callable(sync_augmented_state):
                sync_augmented_state(data)
        elif data.shape == (13,):
            state = data.copy()
        else:
            self.get_logger().warn(
                f'Ignoring Float64MultiArray state with {data.size} values; expected 13 or 19.')
            return

        with self.state_lock:
            self.state = state
            self._state_received = True

    @staticmethod
    def _augmented_mpcc_state_to_external(x_aug: np.ndarray) -> np.ndarray:
        state = np.zeros(13, dtype=np.float64)
        state[0:3] = x_aug[0:3]
        state[3:6] = x_aug[7:10]
        state[6:10] = x_aug[3:7]
        state[10:13] = x_aug[10:13]
        return state

    def drone_state_pose_stamped_cb(self, msg: PoseStamped):
        with self.state_lock:
            self.state = _pose_stamped_to_ned_frd_state(msg)
            self._state_received = True

    # ── Lifecycle ─────────────────────────────────────────────────────────

    def initialize(self):
        """Constructor returns fast; heavy initialization (optimizer build) happens here."""
        self.controller.initialize()

        # The initial auto-hover always uses MPC regardless of controller_type.
        self._mpc_hover_controller = None

        if self._mpcc_stabilization_s > 0.0:
            if isinstance(self.controller, DroneMpcController):
                self._mpc_hover_controller = self.controller
            else:
                self._mpc_hover_controller = DroneMpcController()
                self._mpc_hover_controller.verbose = self._verbose
                self._mpc_hover_controller.initialize()
        else:
            self.get_logger().info(
                "MPC hover disabled because mpcc_stabilization_sec is 0"
            )

        self.get_logger().info("Controller node initialized")

        if isinstance(self.controller, _MPCC_CLASSES):
            self._publish_mpcc_reference_path()
            self._publish_mpcc_tunnel()

        if self._use_mavsdk:
            threading.Thread(target=self._arm_when_ready, daemon=True).start()
        else:
            self.get_logger().info(
                'MAVSDK disabled: using ROS-only dynamics simulator loop')

        if START_AND_HOVER:
            self.get_logger().info("Auto-hover will start once drone state is available")
            self._auto_hover_thread = threading.Thread(
                target=self._wait_for_state_and_hover, daemon=True)
            self._auto_hover_thread.start()

    def _arm_when_ready(self):
        """
        Background thread: call /arm_drone service once the executor is running.

        Waits up to 60 s for the MavSDKClient arming service to appear, then
        arms the drone (which also starts offboard mode).  Must run in a thread
        rather than directly in initialize() because the ROS executor has not
        started yet when initialize() is called.
        """
        arm_client = self.create_client(SetBool, '/arm_drone')
        deadline = time.time() + 60.0
        while not arm_client.service_is_ready() and time.time() < deadline:
            time.sleep(0.2)
        if not arm_client.service_is_ready():
            self.get_logger().error(
                '[DroneController] /arm_drone service not available after 60 s — drone not armed')
            return
        req = SetBool.Request()
        req.data = True
        future = arm_client.call_async(req)
        deadline2 = time.time() + 15.0
        while not future.done() and time.time() < deadline2:
            time.sleep(0.05)
        resp = future.result()
        if resp and resp.success:
            self.get_logger().info('[DroneController] Drone armed and offboard started')
        else:
            self.get_logger().error(
                f'[DroneController] Arm failed: {getattr(resp, "message", "no response")}')

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

        if self.controller_type in ('mpcc', 'mpcc_pp'):
            self.get_logger().info(
                "MPCC/MPCC++ mode: hovering 3 s to stabilise, then starting racing loop")
            threading.Thread(
                target=self._mpcc_auto_start,
                args=(hover_state,),
                daemon=True,
            ).start()
        else:
            self.get_logger().info("Starting auto-hover at 10 cm altitude (NED z=-0.1)")
            threading.Thread(
                target=self.hover_at_currrent_position,
                args=(hover_state,),
                kwargs={'controller': self._mpc_hover_controller},
            ).start()

    # ── MPCC auto-start ───────────────────────────────────────────────────

    def _mpcc_auto_start(self, hover_state: np.ndarray):
        """Hover 3 s with MPC to stabilise, then hand off to the MPCC racing loop."""
        self.hovering_active = True

        # Phase 1: optional MPC hover before handing off to MPCC.
        if self._mpcc_stabilization_s > 0.0:
            deadline = time.time() + self._mpcc_stabilization_s
            while rclpy.ok() and time.time() < deadline and not self.stop_hover.is_set():
                with self.state_lock:
                    current_state = self.state.copy()
                cmd = self._mpc_hover_controller.controller_update(current_state, hover_state)
                self._publish_command(cmd, current_state)
                self.controller_rate.sleep()

            if not rclpy.ok() or self.stop_hover.is_set():
                self.stop_hover.clear()
                self.hovering_active = False
                return

            self.get_logger().info("MPCC: stabilisation done, starting racing loop")
        else:
            self.get_logger().info("MPCC: starting racing loop without MAVSDK stabilisation")
        self._reset_path_timer()
        self._optuna_progress_last_report = 0.0
        self._optuna_liftoff_time = None
        self._optuna_ground_contact_start = None
        self._optuna_crash_reported = False
        self._optuna_last_gate_progress = 0.0
        self._optuna_checked_gate_crossings.clear()
        self._optuna_missed_gates = 0

        # Phase 2: MPCC racing loop — runs until stop_hover is set externally
        while rclpy.ok() and not self.stop_hover.is_set():
            with self.state_lock:
                current_state = self.state.copy()
            cmd = self.controller.controller_update(current_state, current_state)
            self._publish_command(cmd, current_state)
            self._report_mpcc_lap_if_completed()
            if self._simulation_mode:
                self._report_optuna_missed_gates(current_state)
                if self._report_optuna_crash_if_detected(current_state):
                    self.stop_hover.set()
                    break
                self._report_optuna_progress()
            # Republish reference path once after lead-in is added (first step)
            if isinstance(self.controller, _MPCC_CLASSES) and self.controller.path_updated:
                self._publish_mpcc_reference_path()
                self._publish_mpcc_tunnel()
                self.controller.path_updated = False
            self.controller_rate.sleep()

        self.stop_hover.clear()
        self.hovering_active = False

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
            self._log_mpc_inputs(controller, current_state, current_goal_state, 'hover')
            cmd = controller.controller_update(current_state, current_goal_state)
            self._publish_command(cmd, current_state)
            self.controller_rate.sleep()

        self.stop_hover.clear()
        self.hovering_active = False

    # ── Waypoint-reached check ────────────────────────────────────────────

    def waypoint_reached(self) -> bool:
        with self.state_lock:
            current_state = self.state.copy()

        pos_error = np.linalg.norm(current_state[0:3] - self.current_goal_state[0:3])

        # MPCC flies at racing speed — only check position (drone passes through
        # gate waypoints at high velocity; velocity/orientation checks never trigger).
        if isinstance(self.controller, _MPCC_CLASSES):
            return pos_error < self.thresholds_.position

        vel_error = np.linalg.norm(current_state[3:6] - self.current_goal_state[3:6])
        orientation_error = _quaternion_angle_error(
            self.current_goal_state[6:10], current_state[6:10]
        )
        ang_vel_error = np.linalg.norm(current_state[10:13] - self.current_goal_state[10:13])

        return (pos_error < self.thresholds_.position and
                vel_error < self.thresholds_.velocity and
                orientation_error < self.thresholds_.orientation and
                ang_vel_error < self.thresholds_.angular_velocity)

    # ── Publishing ────────────────────────────────────────────────────────

    def _publish_mpcc_reference_path(self) -> None:
        """Publish the MPCC reference spline once as a latching MarkerArray."""
        pts = self.controller.sample_reference_path()
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

    def _publish_mpcc_tunnel(self) -> None:
        """Publish the MPCC track-constraint tube as a latching MarkerArray."""
        tunnel = self.controller.sample_tunnel()
        if tunnel is None:
            return

        positions = tunnel['positions']
        normals = tunnel['normals']
        binormals = tunnel['binormals']
        widths = tunnel['widths']
        heights = tunnel['heights']
        now = self.get_clock().now().to_msg()

        def ned_to_rviz(p: np.ndarray) -> Point:
            return Point(x=float(p[1]), y=float(p[0]), z=float(-p[2]))

        rings = []
        for p, n, b, W, H in zip(positions, normals, binormals, widths, heights):
            rings.append((
                p - W * n - H * b,
                p + W * n - H * b,
                p + W * n + H * b,
                p - W * n + H * b,
            ))

        surface = Marker()
        surface.header.frame_id = 'world'
        surface.header.stamp = now
        surface.ns = 'mpcc_tunnel_surface'
        surface.id = 0
        surface.type = Marker.TRIANGLE_LIST
        surface.action = Marker.ADD
        surface.pose.orientation.w = 1.0
        surface.color.r, surface.color.g, surface.color.b, surface.color.a = 0.0, 0.8, 1.0, 0.16

        for ring_a, ring_b in zip(rings[:-1], rings[1:]):
            for j in range(4):
                a0 = ring_a[j]
                a1 = ring_a[(j + 1) % 4]
                b0 = ring_b[j]
                b1 = ring_b[(j + 1) % 4]
                surface.points.extend([
                    ned_to_rviz(a0), ned_to_rviz(b0), ned_to_rviz(b1),
                    ned_to_rviz(a0), ned_to_rviz(b1), ned_to_rviz(a1),
                ])

        wire = Marker()
        wire.header.frame_id = 'world'
        wire.header.stamp = now
        wire.ns = 'mpcc_tunnel_wire'
        wire.id = 1
        wire.type = Marker.LINE_LIST
        wire.action = Marker.ADD
        wire.pose.orientation.w = 1.0
        wire.scale.x = 0.035
        wire.color.r, wire.color.g, wire.color.b, wire.color.a = 0.0, 0.95, 1.0, 0.75

        ring_stride = max(1, len(rings) // 36)
        for i, ring in enumerate(rings):
            if i % ring_stride != 0 and i != len(rings) - 1:
                continue
            for j in range(4):
                wire.points.append(ned_to_rviz(ring[j]))
                wire.points.append(ned_to_rviz(ring[(j + 1) % 4]))

        for ring_a, ring_b in zip(rings[:-1], rings[1:]):
            for j in range(4):
                wire.points.append(ned_to_rviz(ring_a[j]))
                wire.points.append(ned_to_rviz(ring_b[j]))

        markers = MarkerArray()
        markers.markers.append(surface)
        markers.markers.append(wire)
        self.mpcc_tunnel_publisher.publish(markers)

    def _publish_mpcc_horizon(self) -> None:
        """Publish the MPCC predicted horizon at ~10 Hz (throttled)."""
        self._mpcc_pub_counter += 1
        publish_every = max(
            1, int(getattr(self.controller, 'horizon_publish_every', 10)))
        if self._mpcc_pub_counter % publish_every != 0:
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
        publish_every = max(
            1, int(getattr(self.controller, 'progress_point_publish_every', 1)))
        if self._mpcc_pub_counter % publish_every != 0:
            return

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
        point_scale = float(getattr(self.controller, 'progress_point_scale', 0.2))  # Was 0.2
        point_color = getattr(
            self.controller, 'progress_point_color_rgba', (1.0, 1.0, 0.0, 1.0))
        sphere.scale.x = sphere.scale.y = sphere.scale.z = point_scale
        sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = point_color
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
        line.scale.x = float(getattr(self.controller, 'progress_line_width', 0.03))
        line_color = getattr(
            self.controller, 'progress_line_color_rgba', (1.0, 0.5, 0.0, 0.9))
        line.color.r, line.color.g, line.color.b, line.color.a = line_color
        line.points.append(Point(
            x=float(drone_pos[1]), y=float(drone_pos[0]), z=float(-drone_pos[2])))
        line.points.append(Point(
            x=float(path_pos[1]), y=float(path_pos[0]), z=float(-path_pos[2])))
        markers.markers.append(line)

        self.mpcc_progress_point_publisher.publish(markers)

    def _publish_command(self, cmd, current_state: Optional[np.ndarray] = None) -> None:
        """
        Publish a control command to the correct ROS topic.

        Dispatches on the type of cmd:
          - AttitudeRate    → attitude_command_publisher   (/drone_attitude_cmd)
          - PositionNedYaw  → position_command_publisher   (/drone_position_cmd)
          - anything else   → rotor_command_publisher      (/drone_rotor_cmd)
            (per-rotor thrust: np.ndarray (4,), used by mpc_rotor)
        """
        if current_state is not None:
            self._record_speed_sample(current_state)

        if isinstance(self.controller, _MPCC_CLASSES):
            self._publish_mpcc_horizon()
            self._publish_mpcc_progress_point()
            self._publish_mpcc_simulator_action()
            self._publish_mpcc_rotor_thrust()
        elif isinstance(self.controller, DroneMpcController):
            self._publish_mpcc_horizon()

        if self._verbose:
            self._publish_debug_counter += 1
            if self._publish_debug_counter % 50 == 0:
                if isinstance(cmd, AttitudeRate):
                    print(f"[DEBUG] to MAVSDK (AttitudeRate): "
                          f"thrust={cmd.thrust_value:.3f}, "
                          f"roll={cmd.roll_deg_s:.1f} deg/s, "
                          f"pitch={cmd.pitch_deg_s:.1f} deg/s, "
                          f"yaw={cmd.yaw_deg_s:.1f} deg/s")
                elif isinstance(cmd, PositionNedYaw):
                    print(f"[DEBUG] to MAVSDK (PositionNedYaw): "
                          f"N={cmd.north_m:.3f} m, E={cmd.east_m:.3f} m, "
                          f"D={cmd.down_m:.3f} m, yaw={cmd.yaw_deg:.1f} deg")
                else:
                    print(f"[DEBUG] to MAVSDK (rotor cmd): "
                          f"f=[{float(cmd[0]):.3f}, {float(cmd[1]):.3f}, "
                          f"{float(cmd[2]):.3f}, {float(cmd[3]):.3f}]")

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

    def _publish_mpcc_simulator_action(self) -> None:
        """Publish the raw MPCC++ input consumed by the lightweight simulator."""
        if not self._publish_simulator_action:
            return
        u_opt = getattr(self.controller, 'last_control_input', None)
        if u_opt is None:
            return
        u_opt = np.asarray(u_opt, dtype=np.float64).reshape(-1)
        if u_opt.shape != (5,) or not np.isfinite(u_opt).all():
            return
        msg = Float64MultiArray()
        msg.data = [float(v) for v in u_opt]
        self.simulator_action_publisher.publish(msg)

    def _publish_mpcc_rotor_thrust(self) -> None:
        """Publish the MPCC++ optimized next-step per-rotor thrust [N]."""
        x_pred = getattr(self.controller, '_last_x_pred', None)
        if x_pred is None:
            return

        x_pred = np.asarray(x_pred, dtype=np.float64)
        if x_pred.ndim != 2 or x_pred.shape[0] < 2 or x_pred.shape[1] < 17:
            return

        rotor_thrust = x_pred[1, 13:17]
        if rotor_thrust.shape != (4,) or not np.isfinite(rotor_thrust).all():
            return

        msg = Float64MultiArray()
        msg.data = [float(v) for v in rotor_thrust]
        self.mpcc_rotor_thrust_publisher.publish(msg)
    def _log_mpc_inputs(self, controller: ControllerInterface, current_state: np.ndarray,
                        goal_state: np.ndarray, context: str) -> None:
        """Print the state and goal passed into the MPC at a throttled rate."""
        if not self._print_mpc_inputs or not isinstance(controller, DroneMpcController):
            return

        now = time.monotonic()
        if now - self._last_mpc_input_debug_time < self._mpc_input_debug_interval_s:
            return
        self._last_mpc_input_debug_time = now

        pos_error = np.linalg.norm(current_state[0:3] - goal_state[0:3])
        self.get_logger().info(
            '[MPC input:%s] '
            'state p=(%.3f, %.3f, %.3f) v=(%.3f, %.3f, %.3f) '
            'q=(%.3f, %.3f, %.3f, %.3f) w=(%.3f, %.3f, %.3f); '
            'goal p=(%.3f, %.3f, %.3f) q=(%.3f, %.3f, %.3f, %.3f); '
            'pos_err=%.3f state_received=%s' % (
                context,
                current_state[0], current_state[1], current_state[2],
                current_state[3], current_state[4], current_state[5],
                current_state[6], current_state[7], current_state[8], current_state[9],
                current_state[10], current_state[11], current_state[12],
                goal_state[0], goal_state[1], goal_state[2],
                goal_state[6], goal_state[7], goal_state[8], goal_state[9],
                pos_error,
                self._state_received,
            )
        )

    def _record_speed_sample(self, current_state: np.ndarray) -> None:
        """Accumulate speed stats and print them every fixed interval."""
        now = time.monotonic()
        if self._speed_interval_start is None:
            self._speed_interval_start = now

        vx, vy, vz = current_state[3], current_state[4], current_state[5]
        speed_kmh = math.sqrt(vx * vx + vy * vy + vz * vz) * 3.6
        self._speed_sum_kmh += speed_kmh
        if speed_kmh > self._speed_max_kmh:
            self._speed_max_kmh = speed_kmh
        self._speed_samples += 1

        elapsed = now - self._speed_interval_start
        if elapsed < self._speed_report_interval_s:
            return

        avg_kmh = (self._speed_sum_kmh / self._speed_samples
                   if self._speed_samples else 0.0)
        self.get_logger().info(
            f'[Speed] intervallo {elapsed:.1f}s: '
            f'media={avg_kmh:.1f} km/h, max={self._speed_max_kmh:.1f} km/h')
        self._speed_interval_start = now
        self._speed_sum_kmh = 0.0
        self._speed_max_kmh = 0.0
        self._speed_samples = 0

    def _reset_path_timer(self) -> None:
        self._path_timer_start = time.monotonic()
        self._last_lap_count = int(getattr(self.controller, 'lap_count', 0))

    def _mpcc_completed_laps(self) -> tuple[float, float]:
        """Return lap-equivalent progress and current-lap fraction."""
        lap_count = float(getattr(self.controller, 'lap_count', 0))
        path_length = float(getattr(self.controller, 'path_length', 0.0))
        theta = float(getattr(self.controller, 'current_theta', 0.0))
        lap_fraction = 0.0
        if path_length > 1e-9:
            lap_fraction = float(np.clip(theta / path_length, 0.0, 1.0))
        return lap_count + lap_fraction, lap_fraction

    def _report_optuna_progress(self) -> None:
        """Emit compact progress lines consumed by mpcc_pp_optuna_tune."""
        if not self._simulation_mode:
            return
        if not isinstance(self.controller, _MPCC_CLASSES):
            return
        now = time.monotonic()
        if now - self._optuna_progress_last_report < self._optuna_progress_interval_s:
            return
        completed_laps, lap_fraction = self._mpcc_completed_laps()
        self.get_logger().info(
            f'[Progress] completed_laps={completed_laps:.6f} '
            f'lap_fraction={lap_fraction:.6f}')
        self._optuna_progress_last_report = now

    def _report_optuna_crash_if_detected(self, current_state: np.ndarray) -> bool:
        """Detect sustained ground contact after liftoff and emit one crash line."""
        if not self._simulation_mode:
            return False
        if self._optuna_crash_reported:
            return True
        now_s = self.get_clock().now().nanoseconds * 1e-9
        down_m = float(current_state[2])
        if down_m <= self._optuna_airborne_down_threshold_m:
            if self._optuna_liftoff_time is None:
                self._optuna_liftoff_time = now_s
            self._optuna_ground_contact_start = None
            return False
        if self._optuna_liftoff_time is None:
            return False
        if now_s - self._optuna_liftoff_time < self._optuna_crash_detection_delay_s:
            return False
        if down_m < self._optuna_crash_down_threshold_m:
            self._optuna_ground_contact_start = None
            return False
        if self._optuna_ground_contact_start is None:
            self._optuna_ground_contact_start = now_s
            return False
        if now_s - self._optuna_ground_contact_start < self._optuna_ground_contact_duration_s:
            return False
        completed_laps, lap_fraction = self._mpcc_completed_laps()
        self.get_logger().error(
            f'[Crash] collision=true down_m={down_m:.3f} '
            f'ground_contact_s={now_s - self._optuna_ground_contact_start:.3f} '
            f'completed_laps={completed_laps:.6f} '
            f'lap_fraction={lap_fraction:.6f}')
        self._optuna_crash_reported = True
        return True

    def _report_optuna_missed_gates(self, current_state: np.ndarray) -> None:
        """Emit a missed-gate line when progress crosses a gate too far away."""
        if not self._simulation_mode:
            return
        if not isinstance(self.controller, _MPCC_CLASSES):
            return

        path_length = float(getattr(self.controller, 'path_length', 0.0))
        gate_thetas = getattr(self.controller, 'gate_thetas', None)
        gate_positions = getattr(self.controller, 'gate_positions', None)
        gate_miss_distances = getattr(
            self.controller, 'gate_miss_distances', None)
        if (gate_thetas is None or gate_positions is None
                or gate_miss_distances is None or path_length <= 1e-9):
            return

        gate_thetas = np.asarray(gate_thetas, dtype=np.float64)
        gate_positions = np.asarray(gate_positions, dtype=np.float64)
        gate_miss_distances = np.asarray(
            gate_miss_distances, dtype=np.float64)
        if (len(gate_thetas) == 0
                or len(gate_thetas) != len(gate_positions)
                or len(gate_thetas) != len(gate_miss_distances)):
            return

        current_progress, lap_fraction = self._mpcc_completed_laps()
        previous_progress = self._optuna_last_gate_progress
        if current_progress < previous_progress:
            previous_progress = current_progress

        gate_fractions = np.clip(gate_thetas / path_length, 0.0, 1.0)
        first_lap = max(0, int(math.floor(previous_progress)) - 1)
        last_lap = int(math.floor(current_progress)) + 1
        drone_pos = current_state[0:3]
        for lap_idx in range(first_lap, last_lap + 1):
            for gate_idx, gate_fraction in enumerate(gate_fractions):
                crossing_progress = lap_idx + float(gate_fraction)
                if not previous_progress < crossing_progress <= current_progress:
                    continue
                crossing_id = lap_idx * len(gate_fractions) + gate_idx
                if crossing_id in self._optuna_checked_gate_crossings:
                    continue
                self._optuna_checked_gate_crossings.add(crossing_id)

                distance = float(np.linalg.norm(
                    drone_pos - gate_positions[gate_idx]))
                tolerance = float(gate_miss_distances[gate_idx])
                if distance <= tolerance:
                    continue
                self._optuna_missed_gates += 1
                self.get_logger().warn(
                    f'[MissedGate] gate_index={gate_idx + 1} '
                    f'lap_index={lap_idx + 1} '
                    f'distance_m={distance:.3f} '
                    f'tolerance_m={tolerance:.3f} '
                    f'missed_gates={self._optuna_missed_gates} '
                    f'completed_laps={current_progress:.6f} '
                    f'lap_fraction={lap_fraction:.6f}')

        self._optuna_last_gate_progress = max(
            self._optuna_last_gate_progress,
            current_progress,
        )

    def _report_mpcc_lap_if_completed(self) -> None:
        if not isinstance(self.controller, _MPCC_CLASSES):
            return
        lap_count = int(getattr(self.controller, 'lap_count', 0))
        if self._path_timer_start is None or lap_count < self._last_lap_count:
            self._reset_path_timer()
            return
        if lap_count == self._last_lap_count:
            return

        now = time.monotonic()
        for completed_lap in range(self._last_lap_count + 1, lap_count + 1):
            elapsed = now - self._path_timer_start
            self.get_logger().info(
                f'[Lap] percorso {completed_lap} completato in {elapsed:.2f} s')
            self._path_timer_start = now
        self._last_lap_count = lap_count

        get_solver_stats = getattr(self.controller, 'get_solver_stats', None)
        if callable(get_solver_stats):
            stats = get_solver_stats()
            self.get_logger().info(
                '[SolverStats] '
                f"solve_count={stats.get('solve_count', 0)} "
                f"solver_fail_count={stats.get('solver_fail_count', 0)} "
                f"solver_fail_rate={stats.get('solver_fail_rate', 0.0):.6f} "
                f"last_status={stats.get('last_status')}")

    # ── Helpers ───────────────────────────────────────────────────────────

    def _load_track_config(self, filepath: str) -> dict:
        """
        Load a track configuration YAML and return a unified config dict.

        Supports two formats:

        Gates format (gates_config.yaml — preferred for MPCC):
            track: {W_nominal: 3.0, ...}
            gates:
              - {name: G1, x: 0.0, y: 0.0, z: -3.0, yaw_deg: 0.0, width: 2.0, height: 2.0}

        Waypoints format (waypoints_config.yaml — legacy):
            waypoints:
              - {x: 1.0, y: 0.0, z: -1.0}

        Returns a dict with keys:
          'gates'     — list of gate dicts (only for gates format)
          'track'     — tunnel parameter dict (only for gates format)
          'track_name'— selected track name, used by MPCC++ parameter profiles
          'parameter_file' — optional MPCC++ parameter YAML override path
          'waypoints' — (N, 3) float64 array (always present; for gates format
                        this contains the gate centres so non-MPCC controllers
                        can still use it as a waypoint list)
        """
        if not filepath:
            return {}
        try:
            with open(filepath, 'r') as f:
                data = yaml.safe_load(f)
            parameter_file = (
                data.get('mpcc_pp_parameters_file') or data.get('parameter_file'))
            if 'tracks' in data:
                active = data.get('active_track', next(iter(data['tracks'])))
                if active not in data['tracks']:
                    raise RuntimeError(
                        f"Track '{active}' not found. Available: {list(data['tracks'].keys())}")
                track_data = data['tracks'][active]
                parameter_file = (
                    track_data.get('mpcc_pp_parameters_file')
                    or track_data.get('parameter_file')
                    or parameter_file)
                gates = track_data.get('gates', [])
                track = track_data.get('track', {})
            elif 'gates' in data:
                active = data.get('track_name')
                gates = data['gates']
                track = data.get('track', {})
            elif 'waypoints' in data:
                pts = data['waypoints']
                waypoints = np.array(
                    [[p['x'], p['y'], p['z']] for p in pts], dtype=np.float64)
                return {'waypoints': waypoints, 'parameter_file': parameter_file}
            else:
                return {}
            waypoints = np.array(
                [[g['x'], g['y'], g['z']] for g in gates], dtype=np.float64)
            return {
                'gates': gates,
                'track': track,
                'track_name': active,
                'parameter_file': parameter_file,
                'waypoints': waypoints,
            }
        except Exception as e:
            raise RuntimeError(f"Failed to load track config from '{filepath}': {e}")

    def _pose_stamped_to_goal_state(self, pose: PoseStamped) -> np.ndarray:
        """Convert an external PoseStamped goal to the controller's NED/FRD state."""
        return _pose_stamped_to_ned_frd_state(pose)

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
            self._log_mpc_inputs(self.controller, current_state, self.current_goal_state, 'pose_goal')
            cmd = self.controller.controller_update(current_state, self.current_goal_state)
            self._publish_command(cmd, current_state)
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

        # Reset MPCC so it rebuilds the lead-in from the current drone position
        # (handles the case where _mpcc_auto_start already advanced θ past G1).
        if isinstance(self.controller, _MPCC_CLASSES):
            self.controller.reset()
            self._reset_path_timer()

        lap = 0
        while rclpy.ok() and not goal_handle.is_cancel_requested:
            lap += 1
            lap_start = time.monotonic()
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
                    self._log_mpc_inputs(self.controller, current_state, self.current_goal_state, 'trajectory_goal')
                    cmd = self.controller.controller_update(current_state, self.current_goal_state)
                    self._publish_command(cmd, current_state)
                    self.controller_rate.sleep()

            if not goal_handle.is_cancel_requested:
                elapsed = time.monotonic() - lap_start
                self.get_logger().info(
                    f'[Trajectory Goal] Percorso {lap} completato in {elapsed:.2f} s')

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
