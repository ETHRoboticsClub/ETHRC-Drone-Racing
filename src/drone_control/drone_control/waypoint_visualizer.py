import os

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


def _point(x: float, y: float, z: float) -> Point:
    p = Point()
    p.x, p.y, p.z = x, y, z
    return p


class WaypointVisualizer(Node):
    """
    Publishes RViz MarkerArray with:
      - Gate spheres + labels (G1–G8)
      - Planned circuit (cyan line)
      - Current active target (green sphere)
      - Drone position (yellow cube)
      - Drone trajectory trail (yellow line)
    """

    _MAX_TRAIL = 3000  # maximum number of trail points to keep

    def __init__(self):
        super().__init__('waypoint_visualizer')

        self._drone_pos_enu: tuple | None = None
        self._target_enu: tuple | None = None
        self._trail: list[tuple] = []

        # ── Load waypoints ────────────────────────────────────────────────
        self.declare_parameter('waypoints_file', '')
        wp_file = self.get_parameter('waypoints_file').get_parameter_value().string_value
        if not wp_file:
            wp_file = os.path.join(
                get_package_share_directory('drone_control'),
                'config', 'waypoints_config.yaml',
            )
        with open(wp_file) as f:
            self._waypoints: list[dict] = yaml.safe_load(f)['waypoints']

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

    def _sphere(self, ns: str, mid: int, stamp, x, y, z, r, g, b, a, scale=0.5) -> Marker:
        m = self._base(ns, mid, stamp)
        m.type = Marker.SPHERE
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
        m.scale.x = m.scale.y = m.scale.z = scale
        m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, a
        return m

    def _label(self, ns: str, mid: int, stamp, x, y, z, text: str, size=0.5) -> Marker:
        m = self._base(ns, mid, stamp)
        m.type = Marker.TEXT_VIEW_FACING
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
        m.scale.z = size
        m.color.r = m.color.g = m.color.b = m.color.a = 1.0
        m.text = text
        return m

    def _line(self, ns: str, mid: int, stamp, points, r, g, b, a, width=0.08) -> Marker:
        m = self._base(ns, mid, stamp)
        m.type = Marker.LINE_STRIP
        m.scale.x = width
        m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, a
        m.points = points
        return m

    # ── Main publish ───────────────────────────────────────────────────────

    def _publish(self):
        now = self.get_clock().now().to_msg()
        markers = MarkerArray()
        mid = 0

        # Gate spheres + labels
        for i, wp in enumerate(self._waypoints):
            x, y, z = _ned_to_enu(wp['x'], wp['y'], wp['z'])
            markers.markers.append(
                self._sphere('gates', mid, now, x, y, z, 1.0, 0.35, 0.0, 1.0, scale=0.5))
            mid += 1
            markers.markers.append(
                self._label('gate_labels', mid, now, x, y, z + 0.75, f'G{i + 1}'))
            mid += 1

        # Planned circuit (closed loop)
        wps_loop = self._waypoints + [self._waypoints[0]]
        pts = [_point(*_ned_to_enu(w['x'], w['y'], w['z'])) for w in wps_loop]
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
