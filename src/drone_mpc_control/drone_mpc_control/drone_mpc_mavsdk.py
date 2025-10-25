#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Thin ROS2 node wrapper that composes the modular ACADOS and MAVSDK components.

This file keeps the same entrypoint as before but delegates heavy logic to
`acados_ocp.AcadosController` and `mavsdk_client.MavsdkClient` for clarity.
"""

import asyncio
import numpy as np
from threading import Lock

import rclpy
from rclpy.node import Node

from .acados_ocp import AcadosController
from .mavsdk_client import MavsdkClient


class DroneMPCController(Node):
    def __init__(self):
        super().__init__('drone_mpc_controller')

        # parameters
        self.declare_parameter('control_rate', 50)
        self.declare_parameter('t_horizon', 1.0)
        self.declare_parameter('n_nodes', 20)

        self.declare_parameter('weights.position', [200.0, 200.0, 500.0])
        self.declare_parameter('weights.velocity', [1.0, 1.0, 1.0])
        self.declare_parameter('weights.quaternion', [5.0, 5.0, 200.0, 1.0])
        self.declare_parameter('weights.angular_velocity', [1.0, 1.0, 1.0])
        self.declare_parameter('weights.control', [6.0, 6.0, 6.0, 6.0])

        self.declare_parameter('target.x', 0.0)
        self.declare_parameter('target.y', 0.0)
        self.declare_parameter('target.z', -2.0)

        self.declare_parameter('mass', 0.5)
        self.declare_parameter('gravity', 9.8)
        self.declare_parameter('inertia.I_xx', 0.00232)
        self.declare_parameter('inertia.I_yy', 0.00232)
        self.declare_parameter('inertia.I_zz', 0.00400)

        self.declare_parameter('connection_string', 'udpin://0.0.0.0:14540')

        self.control_rate = self.get_parameter('control_rate').value
        self.t_horizon = self.get_parameter('t_horizon').value
        self.n_nodes = self.get_parameter('n_nodes').value

        weights = {
            'position': self.get_parameter('weights.position').value,
            'velocity': self.get_parameter('weights.velocity').value,
            'quaternion': self.get_parameter('weights.quaternion').value,
            'angular_velocity': self.get_parameter('weights.angular_velocity').value,
            'control': self.get_parameter('weights.control').value,
        }

        # ACADOS controller
        self.get_logger().info('Initializing ACADOS controller...')
        self.acados = AcadosController(self.t_horizon, self.n_nodes, weights=weights)

        # MAVSDK client
        self.get_logger().info('Initializing MAVSDK client...')
        self.mavsdk = MavsdkClient(self.get_parameter('connection_string').value)

        # State access lock (we'll use mavsdk.state_lock when reading state)
        self.state_lock = Lock()

        # Target state vector
        tx = self.get_parameter('target.x').value
        ty = self.get_parameter('target.y').value
        tz = self.get_parameter('target.z').value
        self.target = np.array([tx, ty, tz, 1.0])  # simplified: position + q_w

        # timer
        self.solve_times = []
        self.iteration_count = 0
        self.arm_attempted = False

        self.control_timer = self.create_timer(1.0 / self.control_rate, self.control_loop_callback)

        self.get_logger().info('Drone MPC Controller initialized.')

    def control_loop_callback(self):
        # wait for connection/health
        if not self.mavsdk.is_connected:
            return
        if not self.mavsdk.health_ok:
            return

        # read current state
        with self.mavsdk.state_lock:
            state_vec = self.mavsdk.current_state.to_vector()

        if not np.isfinite(state_vec).all():
            return

        # arm/offboard if needed
        if (not self.mavsdk.is_offboard) and (not self.arm_attempted):
            self.arm_attempted = True
            self.mavsdk.run_coroutine(self.mavsdk.arm_and_start_offboard())
            return

        # Solve MPC
        # create a target full state vector compatible with ACADOS nx
        # Here we expand the simple target to match expected dims by ACADOS (caller can refine)
        x_target = np.zeros(self.acados.nx)
        x_target[0:3] = np.array([self.get_parameter('target.x').value,
                                   self.get_parameter('target.y').value,
                                   self.get_parameter('target.z').value])
        x_target[6] = 1.0

        u_opt, t_solve = self.acados.solve(state_vec, x_target)
        if u_opt is None:
            return

        thrust_raw = u_opt[0]
        moments = u_opt[1:4]

        mass = self.get_parameter('mass').value
        g = self.get_parameter('gravity').value
        hover_thrust = mass * g
        thrust_normalized = np.clip((thrust_raw + hover_thrust) / (2 * hover_thrust), 0.0, 1.0)

        I_xx = self.get_parameter('inertia.I_xx').value
        I_yy = self.get_parameter('inertia.I_yy').value
        I_zz = self.get_parameter('inertia.I_zz').value
        I_inv = np.diag([1.0 / I_xx, 1.0 / I_yy, 1.0 / I_zz])

        with self.mavsdk.state_lock:
            w_current = self.mavsdk.current_state.angular_velocity

        w_dot = I_inv @ moments
        dt = 1.0 / self.control_rate
        w_desired = w_current + w_dot * dt
        w_desired = np.zeros_like(w_desired)

        # send command
        self.mavsdk.run_coroutine(self.mavsdk.send_attitude_rate(thrust_normalized, w_desired))

        # stats
        self.solve_times.append(t_solve)
        self.iteration_count += 1
        if self.iteration_count >= 50:
            avg = np.mean(self.solve_times) if self.solve_times else 0
            self.get_logger().info(f'MPC solve avg {avg*1000:.2f} ms')
            self.solve_times = []
            self.iteration_count = 0

    def destroy_node(self):
        try:
            self.mavsdk.run_coroutine(self.mavsdk.drone.offboard.stop())
        except Exception:
            pass
        try:
            self.mavsdk.loop.call_soon_threadsafe(self.mavsdk.loop.stop)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = DroneMPCController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
