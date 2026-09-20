#!/usr/bin/env python3
"""ROS 2 node that simulates the exact mpcc_pp augmented drone dynamics."""

from __future__ import annotations

import math
from typing import Iterable

import casadi as ca
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from .drone_model import DroneMpccPpModel
from .parameters import get_section, load_mpcc_pp_parameters


def _as_float_array(values: Iterable[float], length: int, name: str) -> np.ndarray:
    arr = np.asarray(list(values), dtype=np.float64)
    if arr.shape != (length,):
        raise ValueError(f'{name} must contain {length} values, got {arr.shape}')
    return arr


def _normalize_quaternion(q: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(q))
    if norm < 1e-12 or not np.isfinite(norm):
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    return q / norm


def _quat_to_rotmat_wxyz(q: np.ndarray) -> np.ndarray:
    qw, qx, qy, qz = _normalize_quaternion(q)
    return np.array([
        [1.0 - 2.0 * (qy * qy + qz * qz),
         2.0 * (qx * qy - qw * qz),
         2.0 * (qx * qz + qw * qy)],
        [2.0 * (qx * qy + qw * qz),
         1.0 - 2.0 * (qx * qx + qz * qz),
         2.0 * (qy * qz - qw * qx)],
        [2.0 * (qx * qz - qw * qy),
         2.0 * (qy * qz + qw * qx),
         1.0 - 2.0 * (qx * qx + qy * qy)],
    ], dtype=np.float64)


def _rotmat_to_quat_wxyz(r: np.ndarray) -> np.ndarray:
    trace = float(np.trace(r))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (r[2, 1] - r[1, 2]) / s
        qy = (r[0, 2] - r[2, 0]) / s
        qz = (r[1, 0] - r[0, 1]) / s
    else:
        i = int(np.argmax(np.diag(r)))
        if i == 0:
            s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
            qw = (r[2, 1] - r[1, 2]) / s
            qx = 0.25 * s
            qy = (r[0, 1] + r[1, 0]) / s
            qz = (r[0, 2] + r[2, 0]) / s
        elif i == 1:
            s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
            qw = (r[0, 2] - r[2, 0]) / s
            qx = (r[0, 1] + r[1, 0]) / s
            qy = 0.25 * s
            qz = (r[1, 2] + r[2, 1]) / s
        else:
            s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
            qw = (r[1, 0] - r[0, 1]) / s
            qx = (r[0, 2] + r[2, 0]) / s
            qy = (r[1, 2] + r[2, 1]) / s
            qz = 0.25 * s
    return _normalize_quaternion(np.array([qw, qx, qy, qz], dtype=np.float64))


def _ned_frd_quat_to_enu_flu(q_ned_frd: np.ndarray) -> np.ndarray:
    """Convert body FRD -> NED quaternion into ROS body FLU -> ENU quaternion."""
    ned_to_enu = np.array([
        [0.0, 1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0],
    ], dtype=np.float64)
    flu_to_frd = np.diag([1.0, -1.0, -1.0])
    r_enu_flu = ned_to_enu @ _quat_to_rotmat_wxyz(q_ned_frd) @ flu_to_frd
    return _rotmat_to_quat_wxyz(r_enu_flu)


class DroneMpccPpDynamicsSimulator(Node):
    """Integrate the mpcc_pp 19-state model using incoming 5-dim MPC actions."""

    def __init__(self):
        super().__init__('mpcc_pp_dynamics_simulator')

        self.declare_parameter('state_input_topic', '/mpcc_pp/mpc_state')
        self.declare_parameter('action_input_topic', '/mpcc_pp/mpc_action')
        self.declare_parameter('augmented_state_output_topic', '/mpcc_pp/sim_state')
        self.declare_parameter('external_state_output_topic', 'drone_state_sim')
        self.declare_parameter('pose_output_topic', '/mpcc_pp/sim_pose')
        self.declare_parameter('dt', 0.01)
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('publish_external_state', True)
        self.declare_parameter('publish_pose', True)
        self.declare_parameter('pose_ned_to_enu', True)
        self.declare_parameter('initialize_from_parameters', True)
        self.declare_parameter('periodic_publish', True)
        self.declare_parameter('hold_initial_until_first_action', True)
        self.declare_parameter('integration_steps', 0)
        self.declare_parameter('debug_dynamics', True)
        self.declare_parameter('debug_print_every', 1)
        self.declare_parameter('initial_position_ned', [0.0, 0.0, -1.0])
        self.declare_parameter('initial_velocity_ned', [0.0, 0.0, 0.0])
        self.declare_parameter('initial_quaternion_wxyz', [1.0, 0.0, 0.0, 0.0])
        self.declare_parameter('initial_body_rates_rad_s', [0.0, 0.0, 0.0])
        self.declare_parameter('initial_theta', 0.0)
        self.declare_parameter('initial_vtheta', 0.0)

        params = load_mpcc_pp_parameters()
        solver_cfg = get_section(params, 'solver')

        self._dt = float(self.get_parameter('dt').value)
        if self._dt <= 0.0:
            raise ValueError('dt must be positive')

        configured_steps = int(self.get_parameter('integration_steps').value)
        if configured_steps > 0:
            self._integration_steps = configured_steps
        else:
            self._integration_steps = max(
                1, int(solver_cfg.get('sim_method_num_steps', 1)))

        self._frame_id = str(self.get_parameter('frame_id').value)
        self._publish_external = bool(
            self.get_parameter('publish_external_state').value)
        self._publish_pose_enabled = bool(self.get_parameter('publish_pose').value)
        self._pose_ned_to_enu = bool(self.get_parameter('pose_ned_to_enu').value)
        self._periodic_publish = bool(self.get_parameter('periodic_publish').value)
        self._hold_initial_until_first_action = bool(
            self.get_parameter('hold_initial_until_first_action').value)
        self._debug_dynamics = bool(self.get_parameter('debug_dynamics').value)
        self._debug_print_every = max(
            1, int(self.get_parameter('debug_print_every').value))
        self._step_count = 0

        self._model = DroneMpccPpModel(model_params=get_section(params, 'model'))
        self._f_expl = ca.Function(
            'mpcc_pp_f_expl',
            [self._model.model.x, self._model.model.u],
            [self._model.model.f_expl_expr],
        )

        state_topic = str(self.get_parameter('state_input_topic').value)
        action_topic = str(self.get_parameter('action_input_topic').value)
        augmented_output_topic = str(
            self.get_parameter('augmented_state_output_topic').value)
        external_output_topic = str(
            self.get_parameter('external_state_output_topic').value)
        pose_output_topic = str(self.get_parameter('pose_output_topic').value)

        self._state_sub = self.create_subscription(
            Float64MultiArray, state_topic, self._state_cb, 10)
        self._action_sub = self.create_subscription(
            Float64MultiArray, action_topic, self._action_cb, 10)
        self._augmented_state_pub = self.create_publisher(
            Float64MultiArray, augmented_output_topic, 10)
        self._external_state_pub = self.create_publisher(
            Float64MultiArray, external_output_topic, 10)
        self._pose_pub = self.create_publisher(PoseStamped, pose_output_topic, 10)

        self._latest_u = np.zeros(self._model.nu, dtype=np.float64)
        self._received_action = False
        self._x_aug: np.ndarray | None = None
        if bool(self.get_parameter('initialize_from_parameters').value):
            self._x_aug = self._initial_state_from_parameters()
            self._publish_state()
        self._sim_timer = None
        if self._periodic_publish:
            self._sim_timer = self.create_timer(self._dt, self._timer_step)

        self.get_logger().info(
            'mpcc_pp dynamics simulator ready: '
            f'state_in={state_topic}, action_in={action_topic}, '
            f'dt={self._dt:.4f}s, integration_steps={self._integration_steps}, '
            'input_hold=zero_order, '
            f'hold_initial_until_first_action={self._hold_initial_until_first_action}')

    def _initial_state_from_parameters(self) -> np.ndarray:
        x_aug = np.zeros(self._model.nx, dtype=np.float64)
        x_aug[0:3] = _as_float_array(
            self.get_parameter('initial_position_ned').value,
            3,
            'initial_position_ned',
        )
        x_aug[3:7] = _normalize_quaternion(_as_float_array(
            self.get_parameter('initial_quaternion_wxyz').value,
            4,
            'initial_quaternion_wxyz',
        ))
        x_aug[7:10] = _as_float_array(
            self.get_parameter('initial_velocity_ned').value,
            3,
            'initial_velocity_ned',
        )
        x_aug[10:13] = _as_float_array(
            self.get_parameter('initial_body_rates_rad_s').value,
            3,
            'initial_body_rates_rad_s',
        )
        x_aug[13:17] = self._model.f_hover.copy()
        x_aug[17] = float(self.get_parameter('initial_theta').value)
        x_aug[18] = float(self.get_parameter('initial_vtheta').value)
        return x_aug

    def _state_cb(self, msg: Float64MultiArray) -> None:
        data = np.asarray(msg.data, dtype=np.float64)
        if data.shape == (self._model.nx,):
            self._x_aug = data.copy()
            self._x_aug[3:7] = _normalize_quaternion(self._x_aug[3:7])
            return
        if data.shape == (13,):
            self._x_aug = self._external_to_augmented(data)
            return
        self.get_logger().warn(
            f'Ignoring state message with {data.size} values; expected 19 or 13.')

    def _action_cb(self, msg: Float64MultiArray) -> None:
        u = np.asarray(msg.data, dtype=np.float64)
        if u.shape != (self._model.nu,):
            self.get_logger().warn(
                f'Ignoring action message with {u.size} values; expected 5.')
            return
        if not np.isfinite(u).all():
            self.get_logger().warn('Ignoring non-finite simulator input command.')
            return

        self._latest_u = u.copy()
        self._received_action = True

        if not self._periodic_publish:
            self._step_with_latest_input()

    def _timer_step(self) -> None:
        self._step_with_latest_input()

    def _step_with_latest_input(self) -> None:
        if self._x_aug is None:
            if self._received_action:
                self.get_logger().warn('Ignoring action before an initial state is available.')
            return
        if not np.isfinite(self._x_aug).all():
            self.get_logger().warn('Ignoring non-finite simulator state.')
            return

        if self._hold_initial_until_first_action and not self._received_action:
            self._publish_state()
            return

        u = self._latest_u.copy()
        x_current = self._x_aug.copy()
        x_next = self._rk4_step(x_current, u, self._dt)
        x_next[3:7] = _normalize_quaternion(x_next[3:7])
        self._step_count += 1

        if self._should_print_debug():
            self._print_transition_debug(x_current, u, x_next)

        self._x_aug = x_next
        self._publish_state()

    def _rk4_step(self, x: np.ndarray, u: np.ndarray, dt: float) -> np.ndarray:
        x_next = x.copy()
        sub_dt = dt / self._integration_steps
        for _ in range(self._integration_steps):
            k1 = self._dynamics(x_next, u)
            k2 = self._dynamics(x_next + 0.5 * sub_dt * k1, u)
            k3 = self._dynamics(x_next + 0.5 * sub_dt * k2, u)
            k4 = self._dynamics(x_next + sub_dt * k3, u)
            x_next = x_next + sub_dt * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0
        return x_next

    def _dynamics(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        return np.asarray(self._f_expl(x, u), dtype=np.float64).reshape(-1)

    def _should_print_debug(self) -> bool:
        return (self._debug_dynamics
                and self._step_count % self._debug_print_every == 0)

    def _print_transition_debug(self, x_current: np.ndarray, u: np.ndarray,
                                x_next: np.ndarray) -> None:
        self.get_logger().info(
            '[DynamicsDebug] '
            f'step={self._step_count} dt={self._dt:.6f}\n'
            f'  x_current_aug={self._fmt(x_current)}\n'
            f'  x_current_ext={self._fmt(self._augmented_to_external_array(x_current))}\n'
            f'  u={self._fmt(u)}\n'
            f'  x_next_aug={self._fmt(x_next)}\n'
            f'  x_next_ext={self._fmt(self._augmented_to_external_array(x_next))}')

    @staticmethod
    def _fmt(values: np.ndarray) -> str:
        return np.array2string(
            np.asarray(values, dtype=np.float64),
            precision=6,
            suppress_small=False,
            separator=', ',
            max_line_width=240,
        )

    @staticmethod
    def _augmented_to_external_array(x_aug: np.ndarray) -> np.ndarray:
        x_ext = np.zeros(13, dtype=np.float64)
        x_ext[0:3] = x_aug[0:3]
        x_ext[3:6] = x_aug[7:10]
        x_ext[6:10] = x_aug[3:7]
        x_ext[10:13] = x_aug[10:13]
        return x_ext

    def _external_to_augmented(self, x_ext: np.ndarray) -> np.ndarray:
        x_aug = self._x_aug.copy() if self._x_aug is not None else np.zeros(
            self._model.nx, dtype=np.float64)
        x_aug[0:3] = x_ext[0:3]
        x_aug[3:7] = _normalize_quaternion(x_ext[6:10])
        x_aug[7:10] = x_ext[3:6]
        x_aug[10:13] = x_ext[10:13]
        if self._x_aug is None:
            x_aug[13:17] = self._model.f_hover.copy()
        return x_aug

    def _augmented_to_external(self) -> np.ndarray:
        return self._augmented_to_external_array(self._x_aug)

    def _publish_state(self) -> None:
        if self._x_aug is None:
            return

        augmented_msg = Float64MultiArray()
        augmented_msg.data = [float(v) for v in self._x_aug]
        self._augmented_state_pub.publish(augmented_msg)

        if self._publish_external:
            external_msg = Float64MultiArray()
            external_msg.data = [float(v) for v in self._augmented_to_external()]
            self._external_state_pub.publish(external_msg)

        if self._publish_pose_enabled:
            self._pose_pub.publish(self._pose_msg())

    def _pose_msg(self) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id

        p_ned = self._x_aug[0:3]
        q_wxyz = self._x_aug[3:7]
        if self._pose_ned_to_enu:
            p = np.array([p_ned[1], p_ned[0], -p_ned[2]], dtype=np.float64)
            q_wxyz = _ned_frd_quat_to_enu_flu(q_wxyz)
        else:
            p = p_ned

        msg.pose.position.x = float(p[0])
        msg.pose.position.y = float(p[1])
        msg.pose.position.z = float(p[2])
        msg.pose.orientation.w = float(q_wxyz[0])
        msg.pose.orientation.x = float(q_wxyz[1])
        msg.pose.orientation.y = float(q_wxyz[2])
        msg.pose.orientation.z = float(q_wxyz[3])
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = DroneMpccPpDynamicsSimulator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
