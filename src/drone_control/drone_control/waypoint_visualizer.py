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
      - Current active target (green sphere)
      - Drone position        (yellow cube)
      - Drone trajectory trail (yellow line)
    """

    _MAX_TRAIL = 3000  # maximum trail points to keep

    def __init__(self):
        super().__init__('waypoint_visualizer')

        self._drone_pos_enu: tuple | None = None
        self._target_enu: tuple | None = None
        self._trail: list[tuple] = []

        # ── Load gates or waypoints ───────────────────────────────────────
        self.declare_parameter('waypoints_file', '')
        wp_file = self.get_parameter('waypoints_file').get_parameter_value().string_value
        if not wp_file:
            wp_file = os.path.join(
                get_package_share_directory('drone_control'),
                'config', 'gates_config.yaml',
            )
        with open(wp_file) as f:
            raw = yaml.safe_load(f)

        if 'gates' in raw:
            self._gates: list[dict] = raw['gates']
            self._mode = 'gates'
        else:
            # Legacy waypoints format — treat each point as a minimal gate dict
            self._gates = raw['waypoints']
            self._mode = 'waypoints'

        # ── Publishers ────────────────────────────────────────────────────
        self._pub = self.create_publisher(MarkerArray, '/waypoints/markers', 10)

        # ── Subscribers ───────────────────────────────────────────────────
        self.create_subscription(
            Float64MultiArray, 'drone_state_sim', self._state_cb, 10)

        latching_qos = QoSProfile(
            depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            PoseStamped, '/drone_current_target', self._target_cb, latching_qos)

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

    def _target_cb(self, msg: PoseStamped):
        p = msg.pose.position
        self._target_enu = _ned_to_enu(p.x, p.y, p.z)

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

    # ── Main publish ───────────────────────────────────────────────────────

    def _publish(self):
        now = self.get_clock().now().to_msg()
        markers = MarkerArray()
        mid = 0

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

        # Current active target
        if self._target_enu is not None:
            x, y, z = self._target_enu
            markers.markers.append(
                self._sphere('target', mid, now, x, y, z, 0.0, 1.0, 0.2, 0.85, scale=0.65))
            mid += 1
            markers.markers.append(
                self._label('target_label', mid, now, x, y, z + 0.95, 'TARGET', size=0.4))
            mid += 1

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
