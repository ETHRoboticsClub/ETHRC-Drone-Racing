#!/usr/bin/env python3

from __future__ import annotations

from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32
from std_srvs.srv import Trigger

from rl.aigp_policy_runtime import (
    AIGP_OURS_105,
    LEGACY_CORNERS_25,
    AigpOursObservationBuilder,
    TorchScriptPolicy,
    build_observation,
    load_track,
    policy_action_to_attitude_cmd,
)


class AigpPolicyNode(Node):
    def __init__(self):
        super().__init__("aigp_policy_node")

        try:
            share_dir = Path(get_package_share_directory("rl"))
            default_policy_path = share_dir / "models" / "nicola" / "policy.pt"
            default_track_path = share_dir / "config" / "aigp_competition_track.yaml"
        except Exception:
            pkg_dir = Path(__file__).resolve().parent.parent
            repo_root = pkg_dir.parent.parent
            default_policy_path = repo_root / "ext" / "nicola" / "policy.pt"
            default_track_path = pkg_dir / "config" / "aigp_competition_track.yaml"

        self.declare_parameter("policy_path", str(default_policy_path))
        self.declare_parameter("track_path", str(default_track_path))
        self.declare_parameter("state_topic", "/drone_state_sim")
        self.declare_parameter("active_gate_topic", "/aigp/active_gate_index")
        self.declare_parameter("attitude_command_topic", "/drone_attitude_cmd")
        self.declare_parameter("policy_rate_hz", 60.0)
        self.declare_parameter("rate_command_scale_rad_s", 4.0)
        self.declare_parameter("startup_hold_s", 4.0)
        self.declare_parameter("velocity_frame", "world_ned")
        self.declare_parameter("velocity_limit", 15.0)
        self.declare_parameter("observation_contract", "auto")
        self.declare_parameter("torch_device", "cpu")

        self.policy_path = Path(self.get_parameter("policy_path").value)
        self.track_path = Path(self.get_parameter("track_path").value)
        self.rate_command_scale_rad_s = float(self.get_parameter("rate_command_scale_rad_s").value)
        self.startup_hold_s = float(self.get_parameter("startup_hold_s").value)
        self.velocity_frame = str(self.get_parameter("velocity_frame").value)
        self.velocity_limit = float(self.get_parameter("velocity_limit").value)
        self.last_action = np.zeros(4, dtype=np.float32)
        self.latest_state: np.ndarray | None = None
        self.active_gate_index = 0
        self.hold_until = self.get_clock().now().nanoseconds * 1e-9 + self.startup_hold_s

        if not self.policy_path.exists():
            raise FileNotFoundError(f"Policy file not found: {self.policy_path}")
        if not self.track_path.exists():
            raise FileNotFoundError(f"Track file not found: {self.track_path}")

        self.gates = load_track(self.track_path)
        requested_contract = str(self.get_parameter("observation_contract").value)
        if requested_contract == "auto":
            requested_contract = None
        self.policy = TorchScriptPolicy(
            self.policy_path,
            device=str(self.get_parameter("torch_device").value),
            observation_contract=requested_contract,
        )
        if self.policy.observation_contract == AIGP_OURS_105:
            self.observation_builder = AigpOursObservationBuilder(
                velocity_frame=self.velocity_frame
            )
        elif self.policy.observation_contract == LEGACY_CORNERS_25:
            self.observation_builder = None
        else:
            raise ValueError(
                f"No runtime observation builder for {self.policy.observation_dim}D policy"
            )

        state_topic = str(self.get_parameter("state_topic").value)
        active_gate_topic = str(self.get_parameter("active_gate_topic").value)
        attitude_command_topic = str(self.get_parameter("attitude_command_topic").value)

        self.state_sub = self.create_subscription(
            Float64MultiArray,
            state_topic,
            self.state_callback,
            10,
        )
        self.active_gate_sub = self.create_subscription(
            Int32,
            active_gate_topic,
            self.active_gate_callback,
            10,
        )
        self.cmd_pub = self.create_publisher(Float64MultiArray, attitude_command_topic, 10)
        self.reset_srv = self.create_service(Trigger, "/aigp_policy/reset", self.reset_callback)

        rate_hz = float(self.get_parameter("policy_rate_hz").value)
        if rate_hz <= 0.0:
            raise ValueError("policy_rate_hz must be positive.")
        self.timer = self.create_timer(1.0 / rate_hz, self.timer_callback)

        self.get_logger().info(
            f"Loaded AIGP policy from {self.policy_path}; track={self.track_path}; "
            f"gates={len(self.gates)}; rate={rate_hz:.1f} Hz; "
            f"contract={self.policy.observation_contract}"
        )

    def state_callback(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 13:
            self.get_logger().warning("Ignoring state message with fewer than 13 values.")
            return
        self.latest_state = np.asarray(msg.data[:13], dtype=np.float64)

    def active_gate_callback(self, msg: Int32) -> None:
        self.active_gate_index = int(np.clip(msg.data, 0, len(self.gates) - 1))

    def reset_callback(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.policy.reset()
        if self.observation_builder is not None:
            self.observation_builder.reset()
        self.last_action[:] = 0.0
        self.hold_until = self.get_clock().now().nanoseconds * 1e-9 + self.startup_hold_s
        response.success = True
        response.message = "AIGP policy recurrent state and previous action reset."
        return response

    def timer_callback(self) -> None:
        now_s = self.get_clock().now().nanoseconds * 1e-9
        if self.latest_state is None:
            return

        if now_s < self.hold_until:
            self.publish_attitude_command(np.zeros(4, dtype=np.float64))
            return

        try:
            if self.observation_builder is None:
                observation = build_observation(
                    self.latest_state,
                    self.gates,
                    self.active_gate_index,
                    self.last_action,
                    velocity_frame=self.velocity_frame,
                )
            else:
                observation = self.observation_builder.build(
                    self.latest_state,
                    self.gates,
                    self.active_gate_index,
                    self.last_action,
                    velocity_limit=self.velocity_limit,
                )
            action = self.policy(observation)
            cmd, clamped_action = policy_action_to_attitude_cmd(
                action,
                rate_command_scale_rad_s=self.rate_command_scale_rad_s,
            )
        except Exception as exc:
            self.get_logger().error(f"AIGP policy update failed: {exc}")
            self.publish_attitude_command(np.zeros(4, dtype=np.float64))
            return

        self.last_action = clamped_action
        self.publish_attitude_command(cmd)

    def publish_attitude_command(self, cmd: np.ndarray) -> None:
        msg = Float64MultiArray()
        msg.data = [float(cmd[0]), float(cmd[1]), float(cmd[2]), float(cmd[3])]
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AigpPolicyNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
