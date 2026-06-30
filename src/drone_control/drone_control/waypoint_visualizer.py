import math
import os

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import Float64MultiArray
import yaml
from ament_index_python.packages import get_package_share_directory


def _ned_to_enu(x_ned: float, y_ned: float, z_ned: float) -> tuple[float, float, float]:
    """NED → ENU: x_enu=y_ned, y_enu=x_ned, z_enu=-z_ned."""
    return y_ned, x_ned, -z_ned


def _ned_vec_to_enu(v_ned: np.ndarray) -> np.ndarray:
    """Convert a NED direction vector to ENU."""
    return np.array([v_ned[1], v_ned[0], -v_ned[2]])


_ENU_TO_NED = np.array([
    [0.0, 1.0, 0.0],
    [1.0, 0.0, 0.0],
    [0.0, 0.0, -1.0],
])
_FLU_TO_FRD = np.diag([1.0, -1.0, -1.0])


def _frame_is_ned(frame_id: str) -> bool:
    return frame_id.strip('/').lower() in ('ned', 'map_ned', 'world_ned')


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
        q = np.array([
            0.25 * s,
            (r[2, 1] - r[1, 2]) / s,
            (r[0, 2] - r[2, 0]) / s,
            (r[1, 0] - r[0, 1]) / s,
        ])
    else:
        i = int(np.argmax(np.diag(r)))
        if i == 0:
            s = np.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
            q = np.array([
                (r[2, 1] - r[1, 2]) / s,
                0.25 * s,
                (r[0, 1] + r[1, 0]) / s,
                (r[0, 2] + r[2, 0]) / s,
            ])
        elif i == 1:
            s = np.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
            q = np.array([
                (r[0, 2] - r[2, 0]) / s,
                (r[0, 1] + r[1, 0]) / s,
                0.25 * s,
                (r[1, 2] + r[2, 1]) / s,
            ])
        else:
            s = np.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
            q = np.array([
                (r[1, 0] - r[0, 1]) / s,
                (r[0, 2] + r[2, 0]) / s,
                (r[1, 2] + r[2, 1]) / s,
                0.25 * s,
            ])
    return _normalize_quat(q)


def _enu_flu_quat_to_ned_frd(q: np.ndarray) -> np.ndarray:
    return _rot_to_quat(_ENU_TO_NED @ _quat_to_rot(q) @ _FLU_TO_FRD.T)


def _quat_body_axes_ned(q: np.ndarray) -> tuple:
    """
    Given a body->NED quaternion [w,x,y,z] (FRD body, NED world — see
    mpc/drone_model.py R_bn), return the (front, right, down) body unit
    axes expressed in NED, i.e. the columns of the body->NED rotation matrix.
    """
    qw, qx, qy, qz = q
    norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if norm < 1e-9:
        qw, qx, qy, qz = 1.0, 0.0, 0.0, 0.0
    else:
        qw, qx, qy, qz = qw / norm, qx / norm, qy / norm, qz / norm

    front = np.array([
        1 - 2 * (qy ** 2 + qz ** 2),
        2 * (qx * qy + qw * qz),
        2 * (qx * qz - qw * qy),
    ])
    right = np.array([
        2 * (qx * qy - qw * qz),
        1 - 2 * (qx ** 2 + qz ** 2),
        2 * (qy * qz + qw * qx),
    ])
    down = np.array([
        2 * (qx * qz + qw * qy),
        2 * (qy * qz - qw * qx),
        1 - 2 * (qx ** 2 + qy ** 2),
    ])
    return front, right, down


def _point(x: float, y: float, z: float) -> Point:
    p = Point()
    p.x, p.y, p.z = x, y, z
    return p


class WaypointVisualizer(Node):
    """
    Publishes RViz MarkerArray with:

    Gates mode (gates_config.yaml):
      - Gate rectangles       — green LINE_LIST frames sized to width × height
      - Gate direction arrows — cyan arrows showing flight-through direction
      - Gate labels           — gate name (G1..Gn)
      - Planned circuit       — cyan line through gate centres

    Waypoints mode (waypoints_config.yaml, legacy):
      - Gate spheres + labels (orange)
      - Planned circuit (cyan line)

    Both modes:
      - Current active target (green sphere) + body axes (desired yaw)
      - Drone position        (yellow cube)  + body axes (live attitude)
      - Drone trajectory trail (yellow line)
    """

    _MAX_TRAIL = 3000  # maximum trail points to keep
    _AXIS_COLORS = ((1.0, 0.0, 0.0, 0.95),   # X (front) — red
                    (0.0, 1.0, 0.0, 0.95),   # Y (right) — green
                    (0.0, 0.4, 1.0, 0.95))   # Z (down)  — blue

    def __init__(self):
        super().__init__('waypoint_visualizer')

        self._drone_pos_enu: tuple | None = None
        self._drone_quat_ned: np.ndarray | None = None
        self._target_enu: tuple | None = None
        self._target_quat_ned: np.ndarray | None = None
        self._trail: list[tuple] = []

        # ── Load gates or waypoints ───────────────────────────────────────
        self.declare_parameter('waypoints_file', '')
        wp_file = self.get_parameter('waypoints_file').get_parameter_value().string_value

        if not wp_file:
            self._gates = []
            self._mode = 'none'
        else:
            with open(wp_file) as f:
                raw = yaml.safe_load(f) or {}

            if 'tracks' in raw:
                active = raw.get('active_track', next(iter(raw['tracks'])))
                if active not in raw['tracks']:
                    raise RuntimeError(
                        f"Track '{active}' not found. Available: {list(raw['tracks'].keys())}")
                self._gates: list[dict] = raw['tracks'][active].get('gates', [])
                self._mode = 'gates'
            elif 'gates' in raw:
                self._gates: list[dict] = raw['gates']
                self._mode = 'gates'
            elif 'waypoints' in raw:
                # Legacy waypoints format — treat each point as a minimal gate dict
                self._gates = raw['waypoints']
                self._mode = 'waypoints'
            else:
                self._gates = []
                self._mode = 'none'

        # ── Publishers ────────────────────────────────────────────────────
        self._pub = self.create_publisher(MarkerArray, '/waypoints/markers', 10)

        # ── Subscribers ───────────────────────────────────────────────────
        self.create_subscription(
            Float64MultiArray, 'drone_state_sim', self._state_cb, 10)

        latching_qos = QoSProfile(
            depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            PoseStamped, '/drone_current_target', self._target_cb, latching_qos)

        # ── Threshold (drives target-sphere size) ────────────────────────
        self.declare_parameter('threshold.position', 0.5)
        self._threshold_pos = self.get_parameter('threshold.position').get_parameter_value().double_value

        # ── Publish at 10 Hz ──────────────────────────────────────────────
        self.create_timer(0.1, self._publish)

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _state_cb(self, msg: Float64MultiArray):
        data = msg.data
        if len(data) < 3:
            return
        pos = _ned_to_enu(data[0], data[1], data[2])
        self._drone_pos_enu = pos
        self._trail.append(pos)
        if len(self._trail) > self._MAX_TRAIL:
            self._trail.pop(0)
        if len(data) >= 10:
            self._drone_quat_ned = np.array(data[6:10])

    def _target_cb(self, msg: PoseStamped):
        p = msg.pose.position
        if _frame_is_ned(msg.header.frame_id):
            self._target_enu = _ned_to_enu(p.x, p.y, p.z)
        else:
            self._target_enu = (p.x, p.y, p.z)
        o = msg.pose.orientation
        q = np.array([o.w, o.x, o.y, o.z])
        self._target_quat_ned = q if _frame_is_ned(msg.header.frame_id) else _enu_flu_quat_to_ned_frd(q)

    # ── Marker construction helpers ────────────────────────────────────────

    def _base(self, ns: str, mid: int, stamp) -> Marker:
        m = Marker()
        m.header.frame_id = 'world'
        m.header.stamp = stamp
        m.ns = ns
        m.id = mid
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    def _sphere(self, ns, mid, stamp, x, y, z, r, g, b, a, scale=0.5) -> Marker:
        m = self._base(ns, mid, stamp)
        m.type = Marker.SPHERE
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
        m.scale.x = m.scale.y = m.scale.z = scale
        m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, a
        return m

    def _label(self, ns, mid, stamp, x, y, z, text: str, size=0.5) -> Marker:
        m = self._base(ns, mid, stamp)
        m.type = Marker.TEXT_VIEW_FACING
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
        m.scale.z = size
        m.color.r = m.color.g = m.color.b = m.color.a = 1.0
        m.text = text
        return m

    def _line(self, ns, mid, stamp, points, r, g, b, a, width=0.08) -> Marker:
        m = self._base(ns, mid, stamp)
        m.type = Marker.LINE_STRIP
        m.scale.x = width
        m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, a
        m.points = points
        return m

    def _gate_frame(self, ns: str, mid: int, stamp, gate: dict) -> Marker:
        """
        Render a gate as a rectangular LINE_LIST frame.

        The rectangle is centred at the gate position with:
          - width  along n = normalize([-sin(yaw), cos(yaw), 0])  (NED right)
          - height along b = [0, 0, -1]                           (NED up)

        All 4 corners are converted NED → ENU before publishing.
        """
        cx, cy, cz = gate['x'], gate['y'], gate['z']
        yaw = math.radians(gate.get('yaw_deg', 0.0))
        half_w = gate.get('width',  2.0) / 2.0
        half_h = gate.get('height', 2.0) / 2.0

        # Local gate frame in NED
        # n: horizontal right direction when facing yaw
        n_ned = np.array([-math.sin(yaw), math.cos(yaw), 0.0])
        # b: vertical up in NED (NED z is down, so up = -z)
        b_ned = np.array([0.0, 0.0, -1.0])

        centre = np.array([cx, cy, cz])
        corners_ned = [
            centre - half_w * n_ned - half_h * b_ned,   # bottom-left
            centre + half_w * n_ned - half_h * b_ned,   # bottom-right
            centre + half_w * n_ned + half_h * b_ned,   # top-right
            centre - half_w * n_ned + half_h * b_ned,   # top-left
        ]

        m = self._base(ns, mid, stamp)
        m.type = Marker.LINE_LIST
        m.scale.x = 0.10                                   # line width [m]
        m.color.r, m.color.g, m.color.b, m.color.a = 0.2, 1.0, 0.2, 0.95  # green

        # 4 edges: BL→BR, BR→TR, TR→TL, TL→BL
        for i, j in [(0, 1), (1, 2), (2, 3), (3, 0)]:
            for k in (i, j):
                ex, ey, ez = _ned_to_enu(*corners_ned[k])
                m.points.append(_point(ex, ey, ez))
        return m

    def _gate_arrow(self, ns: str, mid: int, stamp, gate: dict) -> Marker:
        """
        Render a short arrow showing the intended flight direction through a gate.

        Arrow spans 0.4 m on each side of the gate centre.
        Colour: cyan.
        """
        cx, cy, cz = gate['x'], gate['y'], gate['z']
        yaw = math.radians(gate.get('yaw_deg', 0.0))

        # Tangent = direction of travel through gate in NED
        t_ned = np.array([math.cos(yaw), math.sin(yaw), 0.0])

        start_ned = np.array([cx, cy, cz]) - 0.4 * t_ned
        end_ned   = np.array([cx, cy, cz]) + 0.4 * t_ned

        m = self._base(ns, mid, stamp)
        m.type = Marker.ARROW
        m.scale.x = 0.06    # shaft diameter [m]
        m.scale.y = 0.12    # arrowhead diameter [m]
        m.scale.z = 0.10    # arrowhead length [m]
        m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 0.8, 1.0, 0.95  # cyan

        sx, sy, sz = _ned_to_enu(*start_ned)
        ex, ey, ez = _ned_to_enu(*end_ned)
        m.points.append(_point(sx, sy, sz))
        m.points.append(_point(ex, ey, ez))
        return m

    def _axis_arrow(self, ns, mid, stamp, origin_enu, dir_enu, color, length,
                     shaft_d=0.04, head_d=0.09, head_l=0.12) -> Marker:
        m = self._base(ns, mid, stamp)
        m.type = Marker.ARROW
        m.scale.x = shaft_d
        m.scale.y = head_d
        m.scale.z = head_l
        m.color.r, m.color.g, m.color.b, m.color.a = color
        ox, oy, oz = origin_enu
        m.points.append(_point(ox, oy, oz))
        m.points.append(_point(
            ox + dir_enu[0] * length,
            oy + dir_enu[1] * length,
            oz + dir_enu[2] * length))
        return m

    def _body_axes_markers(self, ns: str, mid: int, stamp, origin_enu, quat_ned: np.ndarray,
                            length: float) -> tuple:
        """Build RGB (front/right/down) body-axis arrows from a body->NED quaternion."""
        axes_ned = _quat_body_axes_ned(quat_ned)
        markers = []
        for axis_ned, color in zip(axes_ned, self._AXIS_COLORS):
            markers.append(self._axis_arrow(
                ns, mid, stamp, origin_enu, _ned_vec_to_enu(axis_ned), color, length))
            mid += 1
        return markers, mid

    # ── Main publish ───────────────────────────────────────────────────────

    def _publish(self):
        now = self.get_clock().now().to_msg()
        markers = MarkerArray()
        mid = 0

        if self._gates:
            if self._mode == 'gates':
                # Gate frames + direction arrows + labels
                for i, gate in enumerate(self._gates):
                    x, y, z = _ned_to_enu(gate['x'], gate['y'], gate['z'])
                    name = gate.get('name', f'G{i + 1}')

                    markers.markers.append(
                        self._gate_frame('gate_frames', mid, now, gate))
                    mid += 1

                    markers.markers.append(
                        self._gate_arrow('gate_arrows', mid, now, gate))
                    mid += 1

                    markers.markers.append(
                        self._label('gate_labels', mid, now, x, y, z + 1.2, name))
                    mid += 1
            else:
                # Legacy: orange spheres + labels
                for i, wp in enumerate(self._gates):
                    x, y, z = _ned_to_enu(wp['x'], wp['y'], wp['z'])
                    markers.markers.append(
                        self._sphere('gates', mid, now, x, y, z, 1.0, 0.35, 0.0, 1.0))
                    mid += 1
                    markers.markers.append(
                        self._label('gate_labels', mid, now, x, y, z + 0.75, f'G{i + 1}'))
                    mid += 1

            # Planned circuit (closed loop through gate / waypoint centres)
            loop = self._gates + [self._gates[0]]
            pts = [_point(*_ned_to_enu(w['x'], w['y'], w['z'])) for w in loop]
            markers.markers.append(
                self._line('planned_circuit', mid, now, pts, 0.2, 0.8, 1.0, 0.9))
            mid += 1

        # Current active target — sphere diameter = 2 × position threshold
        if self._target_enu is not None:
            x, y, z = self._target_enu
            sphere_diam = 2.0 * self._threshold_pos
            markers.markers.append(
                self._sphere('target', mid, now, x, y, z, 0.0, 1.0, 0.2, 0.85, scale=sphere_diam))
            mid += 1
            markers.markers.append(
                self._label('target_label', mid, now, x, y, z + self._threshold_pos + 0.3, 'TARGET', size=0.4))
            mid += 1
            if self._target_quat_ned is not None:
                axis_markers, mid = self._body_axes_markers(
                    'target_axes', mid, now, self._target_enu, self._target_quat_ned, length=1.0)
                markers.markers.extend(axis_markers)

        # Drone position
        if self._drone_pos_enu is not None:
            x, y, z = self._drone_pos_enu
            m = self._base('drone', mid, now)
            m.type = Marker.CUBE
            m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
            m.scale.x = m.scale.y = m.scale.z = 0.3
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 1.0, 0.0, 1.0
            markers.markers.append(m)
            mid += 1
            if self._drone_quat_ned is not None:
                axis_markers, mid = self._body_axes_markers(
                    'drone_axes', mid, now, self._drone_pos_enu, self._drone_quat_ned, length=0.5)
                markers.markers.extend(axis_markers)

        # Drone trail
        if len(self._trail) > 1:
            pts = [_point(*p) for p in self._trail]
            markers.markers.append(
                self._line('drone_trail', mid, now, pts, 1.0, 0.9, 0.0, 0.5, width=0.05))
            mid += 1

        self._pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
