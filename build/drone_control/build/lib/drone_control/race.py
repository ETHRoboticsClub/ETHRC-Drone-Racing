#!/usr/bin/env python
"""
Race node — entry point for  ros2 run drone_control race

Runs the MPCC controller for time-optimal drone racing.
Subscribes to the drone state, solves the MPCC OCP at every control
step, and publishes per-rotor normalised thrust commands.

Parameters (ROS2)
─────────────────
  state_topic     (str)   topic for drone state  [default: drone_state_sim]
  rotor_cmd_topic (str)   topic for rotor cmds   [default: /drone_rotor_cmd]
  controller_rate (float) control loop rate [Hz] [default: 100.0]
  t_horizon       (float) OCP horizon [s]         [default: 1.0]
  n_nodes         (int)   OCP shooting nodes      [default: 20]
  waypoints_file  (str)   path to YAML waypoints  [default: '']
  mu              (float) progress reward weight   [default: 10.0]
  ql              (float) lag weight               [default: 10.0]

Waypoints YAML format
─────────────────────
  waypoints:
    - {x: 0.0, y: 0.0, z: -1.0}
    - {x: 2.0, y: 0.0, z: -1.0}
    ...
"""

import sys
import os

# Add workspace src to path so mpcc, mpc, interfaces can be imported
for _candidate in ['~/ros_ws/src', '~/ETHRC-Drone-Racing/src']:
    _p = os.path.expanduser(_candidate)
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

import threading
import time
import yaml
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64MultiArray

from mpcc.drone_mpcc_controller import DroneMpccController


# Fallback waypoints: 4-point square at 1 m altitude (NED z = -1 m)
_DEFAULT_WAYPOINTS = np.array([
    [0.0,  0.0, -1.0],
    [3.0,  0.0, -1.0],
    [3.0,  3.0, -1.0],
    [0.0,  3.0, -1.0],
], dtype=np.float64)


class RaceNode(Node):
    """
    Standalone ROS2 node for MPCC-based drone racing.

    Control loop runs in a background thread at `controller_rate` Hz.
    Publishes Float64MultiArray on `rotor_cmd_topic` with 4 values:
      [u0, u1, u2, u3]  — normalised per-rotor thrust in [0, 1].
    """

    def __init__(self):
        super().__init__('race_node')

        # ── Parameters ─────────────────────────────────────────────────
        self.declare_parameter('state_topic',     'drone_state_sim')
        self.declare_parameter('rotor_cmd_topic', '/drone_rotor_cmd')
        self.declare_parameter('controller_rate', 100.0)
        self.declare_parameter('t_horizon',        1.0)
        self.declare_parameter('n_nodes',          20)
        self.declare_parameter('waypoints_file',   '')
        self.declare_parameter('mu',               10.0)
        self.declare_parameter('ql',               10.0)

        state_topic   = self.get_parameter('state_topic').get_parameter_value().string_value
        cmd_topic     = self.get_parameter('rotor_cmd_topic').get_parameter_value().string_value
        self._rate_hz = self.get_parameter('controller_rate').get_parameter_value().double_value
        t_horizon     = self.get_parameter('t_horizon').get_parameter_value().double_value
        n_nodes       = self.get_parameter('n_nodes').get_parameter_value().integer_value
        wp_file       = self.get_parameter('waypoints_file').get_parameter_value().string_value
        mu            = self.get_parameter('mu').get_parameter_value().double_value
        ql            = self.get_parameter('ql').get_parameter_value().double_value

        # ── Waypoints ───────────────────────────────────────────────────
        waypoints = self._load_waypoints(wp_file)
        self.get_logger().info(
            f"Path: {len(waypoints)} waypoints, "
            f"length ≈ {_path_length(waypoints):.2f} m")

        # ── MPCC controller ─────────────────────────────────────────────
        weights = {'mu': mu, 'ql': ql}
        self._mpcc = DroneMpccController(waypoints, t_horizon=t_horizon,
                                         n_nodes=n_nodes, weights=weights)

        # ── ROS2 pub/sub ────────────────────────────────────────────────
        self._sub = self.create_subscription(
            Float64MultiArray, state_topic, self._state_cb, 10)
        self._pub = self.create_publisher(Float64MultiArray, cmd_topic, 10)

        # State storage
        self._state = np.zeros(13)
        self._state[6] = 1.0   # identity quaternion: qw = 1
        self._state_lock = threading.Lock()
        self._state_ready = threading.Event()

        self._running = False

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def initialize(self):
        """
        Build the ACADOS solver (slow, ~10–60 s on first run) then
        start the control loop thread.
        """
        self.get_logger().info("Building MPCC optimizer — please wait…")
        self._mpcc.initialize()
        self.get_logger().info("Optimizer ready. Starting control loop.")

        self._running = True
        self._thread  = threading.Thread(target=self._control_loop, daemon=True)
        self._thread.start()

    # ------------------------------------------------------------------
    # State callback
    # ------------------------------------------------------------------

    def _state_cb(self, msg: Float64MultiArray):
        with self._state_lock:
            data = np.array(msg.data)
            n = min(len(data), 13)
            self._state[:n] = data[:n]
        self._state_ready.set()

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------

    def _control_loop(self):
        self.get_logger().info("Waiting for drone state…")
        if not self._state_ready.wait(timeout=30.0):
            self.get_logger().error("Timed out waiting for state. Aborting race.")
            return

        self.get_logger().info(
            f"Race loop started at {self._rate_hz:.0f} Hz  "
            f"(path length = {self._mpcc.path_length:.2f} m)")

        dt = 1.0 / self._rate_hz
        cmd_msg = Float64MultiArray()

        while rclpy.ok() and self._running:
            t_start = time.monotonic()

            with self._state_lock:
                x = self._state.copy()

            try:
                u_norm = self._mpcc.mpcc_update(x)

                cmd_msg.data = [float(u_norm[0]), float(u_norm[1]),
                                float(u_norm[2]), float(u_norm[3])]
                self._pub.publish(cmd_msg)

            except Exception as exc:
                self.get_logger().error(f"MPCC update error: {exc}")

            # Throttled status log (~1 Hz)
            if int(time.monotonic()) % 2 == 0:
                self.get_logger().info(
                    f"θ = {self._mpcc.current_theta:.2f} / "
                    f"{self._mpcc.path_length:.2f} m  "
                    f"lap = {self._mpcc.lap_count}",
                    throttle_duration_sec=2.0)

            elapsed = time.monotonic() - t_start
            remaining = dt - elapsed
            if remaining > 0.0:
                time.sleep(remaining)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _load_waypoints(self, filepath: str) -> np.ndarray:
        if filepath and os.path.exists(filepath):
            try:
                with open(filepath, 'r') as fh:
                    data = yaml.safe_load(fh)
                pts = data.get('waypoints', data)
                wps = np.array([[p['x'], p['y'], p['z']] for p in pts],
                               dtype=np.float64)
                self.get_logger().info(f"Waypoints loaded from {filepath}")
                return wps
            except Exception as exc:
                self.get_logger().warn(
                    f"Could not load {filepath}: {exc}. Using defaults.")
        return _DEFAULT_WAYPOINTS.copy()


def _path_length(wps: np.ndarray) -> float:
    return float(np.sum(np.linalg.norm(np.diff(wps, axis=0), axis=1)))


def main(args=None):
    rclpy.init(args=args)
    node = RaceNode()
    node.initialize()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._running = False
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
