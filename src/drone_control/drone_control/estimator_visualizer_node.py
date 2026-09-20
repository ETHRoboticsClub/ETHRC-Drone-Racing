#!/usr/bin/env python3
"""
Graphical State Estimator Visualizer Node.

Renders a live 2D Top-Down & Elevation Flight Trajectory Map and Camera HUD overlay:
  - Estimated Drone Trajectory (Green Trail)
  - Visual PnP Gate Pose Corrections (Orange Markers & Rays)
  - Track Gates & Active Target Gate
  - Real-Time Telemetry Gauges (Position, Velocity, Attitude Tilt, PnP Update Rate)
"""

import os
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float64MultiArray, Int32
from cv_bridge import CvBridge


class EstimatorVisualizerNode(Node):
    def __init__(self):
        super().__init__('estimator_visualizer_node')

        self.declare_parameter('state_topic', '/drone_state_estimated')
        self.declare_parameter('pnp_topic', '/aigp_state_estimator/set_pose')
        self.declare_parameter('camera_topic', '/cam0/image_rgb')
        self.declare_parameter('viz_topic', '/estimator_viz/image_raw')

        state_topic = self.get_parameter('state_topic').value
        pnp_topic = self.get_parameter('pnp_topic').value
        camera_topic = self.get_parameter('camera_topic').value
        viz_topic = self.get_parameter('viz_topic').value

        self.bridge = CvBridge()
        self.est_traj = []  # List of (x, y, z) in NED
        self.pnp_updates = []  # List of (x, y, z, timestamp)
        self.current_state = None
        self.active_gate = 0
        self.pnp_count = 0
        self.last_pnp_time = None
        self.last_camera_frame = None

        # Track gate coordinates (AIGP competition track)
        self.gate_positions = np.array([
            [-23.298,  0.400,  0.032],
            [-18.500,  8.200, -1.200],
            [-10.200, 14.500, -2.500],
            [  2.100, 12.000, -1.800],
            [ 12.500,  3.400, -0.500],
            [ -5.000, -8.000,  0.200],
        ], dtype=np.float64)

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(Float64MultiArray, state_topic, self.state_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, pnp_topic, self.pnp_callback, 10)
        self.create_subscription(Int32, '/aigp/active_gate_index', self.active_gate_callback, 10)
        self.create_subscription(Image, camera_topic, self.camera_callback, sensor_qos)

        self.viz_pub = self.create_publisher(Image, viz_topic, 10)
        self.timer = self.create_timer(0.05, self.render_loop)  # 20 Hz render loop

        self.get_logger().info("Graphical State Estimator Visualizer Node Started.")

    def state_callback(self, msg: Float64MultiArray):
        if len(msg.data) >= 13:
            self.current_state = np.array(msg.data[:13], dtype=np.float64)
            pos = self.current_state[:3]
            self.est_traj.append(pos.copy())
            if len(self.est_traj) > 1000:
                self.est_traj.pop(0)

    def pnp_callback(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        self.pnp_updates.append((p.x, p.y, p.z, self.get_clock().now().nanoseconds * 1e-9))
        self.pnp_count += 1
        self.last_pnp_time = self.get_clock().now().nanoseconds * 1e-9
        if len(self.pnp_updates) > 100:
            self.pnp_updates.pop(0)

    def active_gate_callback(self, msg: Int32):
        self.active_gate = int(msg.data)

    def camera_callback(self, msg: Image):
        try:
            self.last_camera_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            pass

    def render_loop(self):
        # Create Canvas (Width 1280, Height 720)
        canvas = np.zeros((720, 1280, 3), dtype=np.uint8)

        # Panel 1: Top-Down XY Trajectory Map (Left 640x720)
        map_w, map_h = 640, 720
        map_bg = np.full((map_h, map_w, 3), 30, dtype=np.uint8)

        # Map scale & origin (NED: X North/Up, Y East/Right)
        scale = 12.0  # pixels per metre
        cx, cy = map_w // 2, map_h // 2 + 100

        # Draw grid lines
        for g in range(-50, 50, 10):
            gx = int(cx + g * scale)
            gy = int(cy - g * scale)
            cv2.line(map_bg, (gx, 0), (gx, map_h), (50, 50, 50), 1)
            cv2.line(map_bg, (0, gy), (map_w, gy), (50, 50, 50), 1)

        # Draw Gates on Map
        for idx, gpos in enumerate(self.gate_positions):
            gx = int(cx + gpos[1] * scale)  # Y East -> screen X
            gy = int(cy - gpos[0] * scale)  # X North -> screen Y
            color = (0, 255, 255) if idx == self.active_gate else (255, 100, 0)
            cv2.rectangle(map_bg, (gx - 12, gy - 12), (gx + 12, gy + 12), color, 2)
            cv2.putText(map_bg, f"G{idx}", (gx - 10, gy - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        # Draw Estimated Trajectory (Green Trail)
        if len(self.est_traj) > 1:
            pts = []
            for p in self.est_traj:
                px = int(cx + p[1] * scale)
                py = int(cy - p[0] * scale)
                pts.append((px, py))
            for i in range(len(pts) - 1):
                cv2.line(map_bg, pts[i], pts[i + 1], (0, 255, 0), 2)

        # Draw PnP Visual Pose Updates (Orange Dots)
        for (px, py, pz, t) in self.pnp_updates:
            gx = int(cx + py * scale)
            gy = int(cy - px * scale)
            cv2.circle(map_bg, (gx, gy), 4, (0, 140, 255), -1)

        # Draw Current Drone Position & Heading
        if self.current_state is not None:
            pos = self.current_state[:3]
            quat = self.current_state[6:10] # w, x, y, z
            rot = R.from_quat([quat[1], quat[2], quat[3], quat[0]])
            rpy = rot.as_euler('xyz', degrees=True)

            dx = int(cx + pos[1] * scale)
            dy = int(cy - pos[0] * scale)
            cv2.circle(map_bg, (dx, dy), 7, (0, 0, 255), -1)

            # Heading vector line
            yaw_rad = np.radians(-rpy[2])
            hx = int(dx + 25 * np.sin(yaw_rad))
            hy = int(dy - 25 * np.cos(yaw_rad))
            cv2.line(map_bg, (dx, dy), (hx, hy), (0, 0, 255), 2)

        # Map Title
        cv2.putText(map_bg, "2D TOP-DOWN FLIGHT TRAJECTORY (NED)", (15, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(map_bg, "Green: Estimator Trajectory | Orange: Visual PnP | Blue: Gates", (15, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)

        canvas[:720, :640] = map_bg

        # Panel 2: Camera Feed HUD (Right Top 640x360)
        cam_panel = np.zeros((360, 640, 3), dtype=np.uint8)
        if self.last_camera_frame is not None:
            cam_panel = cv2.resize(self.last_camera_frame, (640, 360))

        # Camera HUD overlay
        cv2.putText(cam_panel, "ONBOARD CAMERA FEED", (15, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        now_s = self.get_clock().now().nanoseconds * 1e-9
        if self.last_pnp_time is not None and (now_s - self.last_pnp_time) < 0.3:
            cv2.rectangle(cam_panel, (5, 5), (635, 355), (0, 140, 255), 3) # Orange flash on PnP reset
            cv2.putText(cam_panel, "[PNP GATE CORRECTION FUSED]", (380, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 140, 255), 2)

        canvas[:360, 640:] = cam_panel

        # Panel 3: Real-Time Telemetry Dashboard (Right Bottom 640x360)
        dash_panel = np.full((360, 640, 3), 20, dtype=np.uint8)
        cv2.putText(dash_panel, "STATE ESTIMATOR DASHBOARD", (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.line(dash_panel, (15, 45), (625, 45), (100, 100, 100), 1)

        if self.current_state is not None:
            pos = self.current_state[:3]
            vel = self.current_state[3:6]
            quat = self.current_state[6:10]
            ang_vel = self.current_state[10:13]

            rot = R.from_quat([quat[1], quat[2], quat[3], quat[0]])
            rpy = rot.as_euler('xyz', degrees=True)
            speed = np.linalg.norm(vel)

            cv2.putText(dash_panel, f"Position (NED):  X: {pos[0]:+6.2f}m | Y: {pos[1]:+6.2f}m | Z (Down): {pos[2]:+6.2f}m",
                        (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.putText(dash_panel, f"Velocity (NED):  Vx: {vel[0]:+5.2f}m/s | Vy: {vel[1]:+5.2f}m/s | Vz: {vel[2]:+5.2f}m/s",
                        (20, 115), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.putText(dash_panel, f"Speed:           {speed:5.2f} m/s",
                        (20, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
            cv2.putText(dash_panel, f"Attitude (RPY):  Roll: {rpy[0]:+5.1f}° | Pitch: {rpy[1]:+5.1f}° | Yaw: {rpy[2]:+5.1f}°",
                        (20, 185), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 255), 1)
            cv2.putText(dash_panel, f"Body Rates:      p: {ang_vel[0]:+5.2f} rad/s | q: {ang_vel[1]:+5.2f} rad/s | r: {ang_vel[2]:+5.2f} rad/s",
                        (20, 220), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 255), 1)
            cv2.putText(dash_panel, f"Active Target:   Gate {self.active_gate}",
                        (20, 260), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
            cv2.putText(dash_panel, f"Total PnP Pose Resets: {self.pnp_count}",
                        (20, 295), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 140, 255), 2)

        canvas[360:, 640:] = dash_panel

        # Publish merged visual image
        msg_out = self.bridge.cv2_to_imgmsg(canvas, encoding='bgr8')
        self.viz_pub.publish(msg_out)

        # Show GUI window if environment supports display
        try:
            cv2.imshow("AIGP State Estimator Graphical Dashboard", canvas)
            cv2.waitKey(1)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = EstimatorVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
