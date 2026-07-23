#!/usr/bin/env python3
"""Validate an RL or MPC controller against the complete Betaflight SITL path."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

# Ubuntu's Gazebo Python messages are generated for the protobuf compatibility
# runtime. Set this before importing rclpy or gz message modules.
os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray

try:
    import gz.transport13 as gz_transport
    import gz.msgs10.double_v_pb2 as gz_double_v_msg
except Exception:
    gz_transport = None
    gz_double_v_msg = None


ROOT = Path(__file__).resolve().parents[2]


class ControllerProbe(Node):
    """Collect controller and state traffic while optionally holding ARM."""

    def __init__(self, arm: bool, arm_delay: float,
                 goal: tuple[float, float, float],
                 goal_delay_after_arm: float) -> None:
        """Create subscriptions and the optional arming heartbeat."""
        super().__init__("sitl_controller_validation")
        self.arm = arm
        self.arm_delay = arm_delay
        self.goal = goal
        self.goal_delay_after_arm = goal_delay_after_arm
        self.started_at = time.monotonic()
        self.controller_ready_at: float | None = None
        self.goal_published = False
        self.states: list[list[float]] = []
        self.commands: list[list[float]] = []
        self.altitudes: list[float] = []
        self.positions: list[tuple[float, list[float]]] = []
        self.observations: list[list[float]] = []
        self.motor_commands: list[list[float]] = []
        self.trace: list[dict] = []
        self.joy_publisher = self.create_publisher(Joy, "/joy", 10)
        self.goal_publisher = self.create_publisher(Point, "/drone_goal", 10)
        self.create_subscription(
            Float64MultiArray, "/drone_state_sim", self._state_callback, 10
        )
        self.create_subscription(
            Float64MultiArray, "/drone_attitude_cmd", self._command_callback, 10
        )
        self.create_subscription(
            Float64MultiArray, "/drone_observation", self._observation_callback, 10
        )
        self.create_subscription(
            Float64MultiArray,
            "/drone/sim/motor_command",
            self._ros_motor_command_callback,
            10,
        )
        self.create_subscription(Odometry, "/odometry", self._odometry_callback, 10)
        self.create_timer(0.02, self._publish_joy)
        self.create_timer(0.1, self._publish_goal)
        self.gz_node = None
        if gz_transport is not None and gz_double_v_msg is not None:
            self.gz_node = gz_transport.Node()
            self.gz_node.subscribe(
                gz_double_v_msg.Double_V,
                "/drone/sim/motor_command",
                self._motor_command_callback,
            )

    def _motor_command_callback(self, message) -> None:
        self.motor_commands.append([float(value) for value in message.data])

    def _ros_motor_command_callback(self, message: Float64MultiArray) -> None:
        self.motor_commands.append(list(message.data))

    def _state_callback(self, message: Float64MultiArray) -> None:
        self.states.append(list(message.data))

    def _command_callback(self, message: Float64MultiArray) -> None:
        if self.controller_ready_at is None:
            self.controller_ready_at = time.monotonic()
        command = list(message.data)
        self.commands.append(command)
        if self.states and self.observations:
            self.trace.append({
                "step": len(self.trace),
                "state": self.states[-1],
                "observation": self.observations[-1],
                "command": command,
                "motor_command": self.motor_commands[-1] if self.motor_commands else None,
            })

    def _observation_callback(self, message: Float64MultiArray) -> None:
        self.observations.append(list(message.data))

    def _odometry_callback(self, message: Odometry) -> None:
        position = message.pose.pose.position
        xyz = [float(position.x), float(position.y), float(position.z)]
        self.altitudes.append(xyz[2])
        self.positions.append((time.monotonic() - self.started_at, xyz))

    def _publish_joy(self) -> None:
        arm_active = (
            self.arm
            and self.controller_ready_at is not None
            and time.monotonic() - self.controller_ready_at >= self.arm_delay
        )
        message = Joy()
        message.header.stamp = self.get_clock().now().to_msg()
        message.axes = [0.0, 0.0, 0.0, -1.0]
        message.buttons = [0, 0, 0, 0, int(arm_active), 1, 0, 0]
        self.joy_publisher.publish(message)

    def _publish_goal(self) -> None:
        if self.controller_ready_at is None or self.goal_published:
            return
        # Always publish the requested validator goal. Relying on launch
        # defaults when no delay was requested made --goal X Y Z silently use
        # a stale installed/default goal (typically 0, 0, 0.5). For delayed
        # step tests, retain the initial launch goal until the requested time.
        publish_after = (
            self.arm_delay + self.goal_delay_after_arm
            if self.goal_delay_after_arm > 0.0
            else 0.0
        )
        if time.monotonic() - self.controller_ready_at < publish_after:
            return
        message = Point(x=self.goal[0], y=self.goal[1], z=self.goal[2])
        self.goal_publisher.publish(message)
        self.goal_published = True


def _all_finite(rows: list[list[float]], width: int) -> bool:
    return bool(rows) and all(
        len(row) == width and all(math.isfinite(value) for value in row)
        for row in rows
    )


def run(controller: str, startup_timeout: float, sample_seconds: float, arm: bool,
        trace: Path | None = None, arm_delay: float = 5.5,
        goal: tuple[float, float, float] = (0.0, 0.0, 1.5),
        max_final_goal_error: float | None = None,
        min_goal_error_reduction: float | None = None,
        goal_delay_after_arm: float = 0.0) -> dict:
    """Launch racing bringup and collect one controller validation report."""
    log_path = Path(f"/tmp/ethrc_{controller}_sitl_validation.log")
    log_handle = log_path.open("w", encoding="utf-8")
    command = [
        "ros2",
        "launch",
        "drone_control",
        "racing_bringup.launch.py",
        "sitl:=true",
        f"controller_type:={controller}",
        "view_stream:=false",
        "publish_raw:=false",
        "publish_rgb:=false",
        "use_vio:=false",
        "use_ekf:=false",
        f"rl_default_goal_x:={0.0 if goal_delay_after_arm > 0.0 else goal[0]}",
        f"rl_default_goal_y:={0.0 if goal_delay_after_arm > 0.0 else goal[1]}",
        f"rl_default_goal_z:={goal[2]}",
    ]
    process = subprocess.Popen(
        command,
        cwd=ROOT,
        stdout=log_handle,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )

    rclpy.init()
    probe = ControllerProbe(
        arm=arm, arm_delay=arm_delay, goal=goal,
        goal_delay_after_arm=goal_delay_after_arm,
    )
    try:
        deadline = time.monotonic() + startup_timeout
        while time.monotonic() < deadline and not probe.commands:
            if process.poll() is not None:
                raise RuntimeError(f"bringup exited early with code {process.returncode}")
            rclpy.spin_once(probe, timeout_sec=0.05)
        # A stale controller can publish on the same ROS topics before the
        # newly launched bringup finishes preflight. Never treat that traffic
        # as proof that our child process started successfully.
        if process.poll() is not None:
            raise RuntimeError(f"bringup exited early with code {process.returncode}")
        if not probe.commands:
            raise TimeoutError("controller produced no command before startup timeout")

        sample_deadline = time.monotonic() + sample_seconds
        if arm:
            assert probe.controller_ready_at is not None
            sample_deadline = max(
                sample_deadline,
                probe.controller_ready_at + arm_delay + sample_seconds,
            )
        while time.monotonic() < sample_deadline:
            rclpy.spin_once(probe, timeout_sec=0.02)

        state_ok = _all_finite(probe.states, 13)
        command_ok = _all_finite(probe.commands, 4)
        thrust_values = [row[3] for row in probe.commands if len(row) == 4]
        rate_rows = [row[:3] for row in probe.commands if len(row) == 4]
        thrust_ok = bool(thrust_values) and min(thrust_values) >= 0.0 and max(
            thrust_values
        ) <= 1.0
        saturated_rate_rows = sum(
            any(abs(rate) >= 495.0 for rate in rates) for rates in rate_rows
        )
        rate_saturation_fraction = (
            saturated_rate_rows / len(rate_rows) if rate_rows else 1.0
        )
        rate_behavior_ok = rate_saturation_fraction < 0.10
        arm_activation_elapsed = (
            probe.controller_ready_at - probe.started_at + arm_delay
            if probe.controller_ready_at is not None
            else math.inf
        )
        armed_positions = [
            xyz for elapsed, xyz in probe.positions
            if elapsed >= arm_activation_elapsed
        ]
        initial_position = armed_positions[0] if armed_positions else None
        final_position = armed_positions[-1] if armed_positions else None

        def _distance(point: list[float] | None) -> float | None:
            if point is None:
                return None
            return math.sqrt(sum((point[i] - goal[i]) ** 2 for i in range(3)))

        initial_goal_error = _distance(initial_position)
        final_goal_error = _distance(final_position)
        goal_error_reduction = (
            initial_goal_error - final_goal_error
            if initial_goal_error is not None and final_goal_error is not None
            else None
        )
        goal_tracking_ok = (
            (max_final_goal_error is None or (
                final_goal_error is not None
                and final_goal_error <= max_final_goal_error
            ))
            and (min_goal_error_reduction is None or (
                goal_error_reduction is not None
                and goal_error_reduction >= min_goal_error_reduction
            ))
        )
        passed = (
            state_ok
            and command_ok
            and thrust_ok
            and rate_behavior_ok
            and goal_tracking_ok
            and len(probe.states) >= 20
            and len(probe.commands) >= 20
            and len(probe.altitudes) >= 20
            and (not arm or any(max(row, default=0.0) > 0.10 for row in probe.motor_commands))
        )
        if trace is not None:
            trace.parent.mkdir(parents=True, exist_ok=True)
            trace.write_text("".join(json.dumps(row) + "\n" for row in probe.trace), encoding="utf-8")
        return {
            "passed": passed,
            "controller": controller,
            "armed": arm,
            "state_messages": len(probe.states),
            "command_messages": len(probe.commands),
            "odometry_messages": len(probe.altitudes),
            "observation_messages": len(probe.observations),
            "trace_rows": len(probe.trace),
            "motor_command_messages": len(probe.motor_commands),
            "betaflight_motors_active": any(
                max(row, default=0.0) > 0.10 for row in probe.motor_commands
            ),
            "state_finite_and_shaped": state_ok,
            "command_finite_and_shaped": command_ok,
            "thrust_in_range": thrust_ok,
            "rate_not_persistently_saturated": rate_behavior_ok,
            "rate_saturation_fraction": rate_saturation_fraction,
            "thrust_min": min(thrust_values) if thrust_values else None,
            "thrust_max": max(thrust_values) if thrust_values else None,
            "altitude_min_m": min(probe.altitudes) if probe.altitudes else None,
            "altitude_max_m": max(probe.altitudes) if probe.altitudes else None,
            "goal_m": list(goal),
            "goal_delay_after_arm_s": goal_delay_after_arm,
            "initial_armed_position_m": initial_position,
            "final_position_m": final_position,
            "initial_goal_error_m": initial_goal_error,
            "final_goal_error_m": final_goal_error,
            "goal_error_reduction_m": goal_error_reduction,
            "goal_tracking_passed": goal_tracking_ok,
            "bringup_log": str(log_path),
        }
    finally:
        probe.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=8.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3.0)
        log_handle.close()


def main() -> int:
    """Run one full-stack controller validation from the command line."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("controller", choices=("rl", "mpc"))
    parser.add_argument("--startup-timeout", type=float, default=60.0)
    parser.add_argument("--sample-seconds", type=float, default=3.0)
    parser.add_argument("--arm", action="store_true")
    parser.add_argument(
        "--arm-delay", type=float, default=5.5,
        help="Keep AUX1 low after startup so Betaflight boot grace sees an arm-switch cycle",
    )
    parser.add_argument("--report", type=Path)
    parser.add_argument("--trace", type=Path, help="Write synchronized state/observation/command JSONL")
    parser.add_argument(
        "--goal", type=float, nargs=3, metavar=("X", "Y", "Z"),
        default=(0.0, 0.0, 1.5), help="Default SITL goal in metres",
    )
    parser.add_argument(
        "--max-final-goal-error", type=float,
        help="Fail unless final 3-D goal error is at most this many metres",
    )
    parser.add_argument(
        "--min-goal-error-reduction", type=float,
        help="Fail unless armed flight reduces 3-D goal error by this many metres",
    )
    parser.add_argument(
        "--goal-delay-after-arm", type=float, default=0.0,
        help="Hold x=y=0 for this many seconds after arming, then publish --goal",
    )
    args = parser.parse_args()

    try:
        report = run(
            args.controller, args.startup_timeout, args.sample_seconds, args.arm,
            args.trace, args.arm_delay, tuple(args.goal),
            args.max_final_goal_error, args.min_goal_error_reduction,
            args.goal_delay_after_arm,
        )
    except (OSError, RuntimeError, TimeoutError) as error:
        print(f"SITL controller validation FAILED: {error}", file=sys.stderr)
        return 1

    rendered = json.dumps(report, indent=2, sort_keys=True)
    print(rendered)
    if args.report:
        args.report.write_text(rendered + "\n", encoding="utf-8")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
