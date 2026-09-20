#!/usr/bin/env python3
"""VQ1 GT controller using freshly calibrated FlightSim body-rate targets."""

from __future__ import annotations

import math
from dataclasses import replace
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32
from std_srvs.srv import SetBool, Trigger

from rl.aigp_policy_runtime import load_track
from rl.aigp_vq1_controller_runtime import (
    RateControllerConfig,
    rate_thrust_command_frd,
    vehicle_tilt_rad,
)


class AigpVq1ControllerNode(Node):
    def __init__(self):
        super().__init__("aigp_vq1_controller_node")

        try:
            default_track = (
                Path(get_package_share_directory("rl"))
                / "config/aigp_competition_track.yaml"
            )
        except Exception:
            default_track = (
                Path(__file__).resolve().parent.parent
                / "config/aigp_competition_track.yaml"
            )

        self.declare_parameter("track_path", str(default_track))
        self.declare_parameter("gate_center_height_offset_m", 1.35)
        self.declare_parameter("state_topic", "/aigp/ground_truth/state")
        self.declare_parameter("active_gate_topic", "/aigp/active_gate_index")
        self.declare_parameter("race_status_topic", "/aigp/race_status")
        self.declare_parameter("collision_topic", "/aigp/collision")
        self.declare_parameter("attitude_command_topic", "/drone_attitude_cmd")
        self.declare_parameter("control_rate_hz", 50.0)
        self.declare_parameter("enabled", False)
        self.declare_parameter("startup_hold_s", 0.0)
        self.declare_parameter("leveling_duration_s", 0.0)
        self.declare_parameter("leveling_thrust", 0.15)
        self.declare_parameter("state_timeout_s", 0.15)
        self.declare_parameter("target_speed_m_s", 1.0)
        self.declare_parameter("position_gain_per_s", 0.3)
        self.declare_parameter("lateral_speed_limit_m_s", 0.5)
        self.declare_parameter("max_command_speed_m_s", 1.2)
        self.declare_parameter("velocity_gain_per_s", 0.8)
        self.declare_parameter("vertical_position_gain_per_s", 1.0)
        self.declare_parameter("vertical_speed_limit_m_s", 0.5)
        self.declare_parameter("vertical_velocity_gain_per_s", 2.0)
        self.declare_parameter("acceleration_limit_m_s2", 0.5)
        self.declare_parameter("attitude_gain_per_s", 1.0)
        self.declare_parameter("body_rate_limit_rad_s", 0.5)
        self.declare_parameter("roll_attitude_sign", -1.0)
        self.declare_parameter("roll_rate_sign", 1.0)
        self.declare_parameter("pitch_rate_sign", -1.0)
        self.declare_parameter("thrust_fit_intercept_ned_m_s2", 8.419816438065174)
        self.declare_parameter("thrust_fit_slope_ned_m_s2", -30.33559184948186)
        self.declare_parameter("thrust_calibration_body_up", 0.9521197)
        self.declare_parameter("minimum_thrust", 0.15)
        self.declare_parameter("maximum_thrust", 0.45)
        self.declare_parameter("max_measured_speed_m_s", 5.0)
        self.declare_parameter("max_tilt_deg", 35.0)
        self.declare_parameter("max_centering_error_m", 20.0)

        track_path = Path(str(self.get_parameter("track_path").value))
        gate_center_height_offset_m = float(
            self.get_parameter("gate_center_height_offset_m").value
        )
        self.gates = [
            replace(
                gate,
                position=np.asarray(gate.position, dtype=np.float64)
                + np.array([0.0, 0.0, gate_center_height_offset_m]),
            )
            for gate in load_track(track_path)
        ]
        self.config = RateControllerConfig(
            target_speed_m_s=float(self.get_parameter("target_speed_m_s").value),
            position_gain_per_s=float(self.get_parameter("position_gain_per_s").value),
            lateral_speed_limit_m_s=float(
                self.get_parameter("lateral_speed_limit_m_s").value
            ),
            max_command_speed_m_s=float(
                self.get_parameter("max_command_speed_m_s").value
            ),
            velocity_gain_per_s=float(self.get_parameter("velocity_gain_per_s").value),
            vertical_position_gain_per_s=float(
                self.get_parameter("vertical_position_gain_per_s").value
            ),
            vertical_speed_limit_m_s=float(
                self.get_parameter("vertical_speed_limit_m_s").value
            ),
            vertical_velocity_gain_per_s=float(
                self.get_parameter("vertical_velocity_gain_per_s").value
            ),
            acceleration_limit_m_s2=float(
                self.get_parameter("acceleration_limit_m_s2").value
            ),
            attitude_gain_per_s=float(self.get_parameter("attitude_gain_per_s").value),
            body_rate_limit_rad_s=float(
                self.get_parameter("body_rate_limit_rad_s").value
            ),
            roll_attitude_sign=float(
                self.get_parameter("roll_attitude_sign").value
            ),
            roll_rate_sign=float(self.get_parameter("roll_rate_sign").value),
            pitch_rate_sign=float(self.get_parameter("pitch_rate_sign").value),
            thrust_fit_intercept_ned_m_s2=float(
                self.get_parameter("thrust_fit_intercept_ned_m_s2").value
            ),
            thrust_fit_slope_ned_m_s2=float(
                self.get_parameter("thrust_fit_slope_ned_m_s2").value
            ),
            thrust_calibration_body_up=float(
                self.get_parameter("thrust_calibration_body_up").value
            ),
            minimum_thrust=float(self.get_parameter("minimum_thrust").value),
            maximum_thrust=float(self.get_parameter("maximum_thrust").value),
        )
        self.state_timeout_s = float(self.get_parameter("state_timeout_s").value)
        self.startup_hold_s = float(self.get_parameter("startup_hold_s").value)
        self.leveling_duration_s = float(
            self.get_parameter("leveling_duration_s").value
        )
        self.leveling_thrust = float(self.get_parameter("leveling_thrust").value)
        self.max_measured_speed_m_s = float(
            self.get_parameter("max_measured_speed_m_s").value
        )
        self.max_tilt_rad = math.radians(float(self.get_parameter("max_tilt_deg").value))
        self.max_centering_error_m = float(
            self.get_parameter("max_centering_error_m").value
        )

        self.enabled = bool(self.get_parameter("enabled").value)
        self.ever_enabled = self.enabled
        self.fault_reason: str | None = None
        self.complete = False
        self.latest_state: np.ndarray | None = None
        self.latest_state_time_s = -math.inf
        self.active_gate_index = 0
        self.have_gate_status = False
        self.previous_gate_index = 0
        self.hold_until_s = self._now_s() + self.startup_hold_s
        self.level_until_s = self.hold_until_s + self.leveling_duration_s
        self.last_log_s = -math.inf
        self.reset_requested = False

        self.command_pub = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter("attitude_command_topic").value),
            10,
        )
        self.state_sub = self.create_subscription(
            Float64MultiArray,
            str(self.get_parameter("state_topic").value),
            self.state_callback,
            20,
        )
        self.gate_sub = self.create_subscription(
            Int32,
            str(self.get_parameter("active_gate_topic").value),
            self.gate_callback,
            20,
        )
        self.race_sub = self.create_subscription(
            Float64MultiArray,
            str(self.get_parameter("race_status_topic").value),
            self.race_callback,
            20,
        )
        self.collision_sub = self.create_subscription(
            Float64MultiArray,
            str(self.get_parameter("collision_topic").value),
            self.collision_callback,
            20,
        )
        self.enable_service = self.create_service(
            SetBool, "/aigp_vq1_controller/enable", self.enable_callback
        )
        self.reset_service = self.create_service(
            Trigger, "/aigp_vq1_controller/reset", self.reset_callback
        )
        self.sim_reset_client = self.create_client(Trigger, "/reset_sim")

        rate_hz = float(self.get_parameter("control_rate_hz").value)
        if not 0.0 < rate_hz < 100.0:
            raise ValueError("control_rate_hz must be between 0 and 100 Hz")
        self.timer = self.create_timer(1.0 / rate_hz, self.timer_callback)
        self.get_logger().info(
            f"VQ1 GT rate controller ready: gates={len(self.gates)}, "
            f"enabled={self.enabled}, target_speed={self.config.target_speed_m_s:.2f} m/s"
        )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def state_callback(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 13:
            self._latch_fault("short ground-truth state")
            return
        state = np.asarray(msg.data[:13], dtype=np.float64)
        if not np.all(np.isfinite(state)):
            self._latch_fault("non-finite ground-truth state")
            return
        self.latest_state = state
        self.latest_state_time_s = self._now_s()

    def gate_callback(self, msg: Int32) -> None:
        index = int(msg.data)
        if index == len(self.gates):
            if not self.complete:
                self.get_logger().info("All VQ1 gates passed; awaiting race finish status.")
            self.complete = True
            self.enabled = False
            self.publish_command(np.zeros(4))
            return
        if not 0 <= index < len(self.gates):
            self._latch_fault(f"invalid gate index {index}")
            return
        if self.have_gate_status and index != self.active_gate_index:
            self.get_logger().info(f"Gate advanced: {self.active_gate_index} -> {index}")
        self.previous_gate_index = self.active_gate_index
        self.active_gate_index = index
        self.have_gate_status = True

    def race_callback(self, msg: Float64MultiArray) -> None:
        if len(msg.data) >= 3 and float(msg.data[2]) >= 0.0:
            if not self.complete:
                self.get_logger().info("VQ1 race finish reported; cutting thrust.")
            self.complete = True
            self.enabled = False
            self.publish_command(np.zeros(4))

    def collision_callback(self, msg: Float64MultiArray) -> None:
        collision_id = int(msg.data[0]) if msg.data else -1
        threat_level = float(msg.data[1]) if len(msg.data) >= 2 else 0.0
        impact = (
            float(msg.data[2]) if len(msg.data) >= 3 else math.nan
        )
        # V4 repurposes horizontal_minimum_delta as impulse magnitude. Threat-1
        # packets occur on the start pad and at valid gate transitions (the
        # race index still advances); follow the simulator's severity contract
        # and stop only on a threat-2 collision.
        if threat_level >= 2.0:
            self._latch_fault(
                f"collision {collision_id} threat={threat_level:.0f} "
                f"impact={impact:.3f}"
            )

    def enable_callback(self, request: SetBool.Request, response: SetBool.Response):
        if request.data:
            now = self._now_s()
            state_fresh = now - self.latest_state_time_s <= self.state_timeout_s
            if self.fault_reason is not None or self.complete:
                response.success = False
                response.message = self.fault_reason or "race already complete"
                return response
            if not state_fresh or not self.have_gate_status:
                response.success = False
                response.message = "fresh ground truth and gate status required"
                return response
            self.enabled = True
            self.ever_enabled = True
            self.hold_until_s = now + self.startup_hold_s
            self.level_until_s = self.hold_until_s + self.leveling_duration_s
            self.reset_requested = False
            response.message = (
                f"enabled; hold={self.startup_hold_s:.1f}s, "
                f"low-thrust leveling={self.leveling_duration_s:.1f}s"
            )
        else:
            self.enabled = False
            self.publish_command(np.zeros(4))
            response.message = "disabled and commanding zero thrust/rates"
        response.success = True
        return response

    def reset_callback(self, _request: Trigger.Request, response: Trigger.Response):
        self.enabled = False
        self.fault_reason = None
        self.complete = False
        self.have_gate_status = False
        self.active_gate_index = 0
        self.previous_gate_index = 0
        self.hold_until_s = self._now_s() + self.startup_hold_s
        self.level_until_s = self.hold_until_s + self.leveling_duration_s
        self.reset_requested = False
        self.publish_command(np.zeros(4))
        response.success = True
        response.message = "controller state reset; disabled"
        return response

    def _latch_fault(self, reason: str) -> None:
        if self.fault_reason is None:
            self.fault_reason = reason
            self.get_logger().error(f"VQ1 controller fault latched: {reason}")
            if self.sim_reset_client.service_is_ready():
                self.sim_reset_client.call_async(Trigger.Request())
                self.reset_requested = True
                self.get_logger().warning("Requested simulator reset after fault.")
        self.enabled = False

    def timer_callback(self) -> None:
        now = self._now_s()
        if not self.enabled:
            return
        if self.complete or self.fault_reason is not None:
            self.publish_command(np.zeros(4))
            return
        if now - self.latest_state_time_s > self.state_timeout_s:
            self._latch_fault("stale ground-truth state")
            self.publish_command(np.zeros(4))
            return
        if not self.have_gate_status or self.latest_state is None:
            return
        if now < self.hold_until_s:
            return

        if now < self.level_until_s:
            try:
                command, telemetry = rate_thrust_command_frd(
                    self.latest_state,
                    self.gates[self.active_gate_index],
                    self.config,
                    acceleration_override_nwu=np.zeros(3),
                )
            except Exception as exc:
                self._latch_fault(f"leveling update failed: {exc}")
                return
            command[3] = self.leveling_thrust
            self.publish_command(command)
            if now - self.last_log_s >= 0.5:
                self.last_log_s = now
                self.get_logger().info(
                    f"leveling tilt={math.degrees(vehicle_tilt_rad(self.latest_state)):.1f}deg "
                    f"rates_frd={np.round(command[:3], 1).tolist()}deg/s "
                    f"thrust={command[3]:.3f}"
                )
            return

        measured_speed = float(np.linalg.norm(self.latest_state[3:6]))
        tilt = vehicle_tilt_rad(self.latest_state)
        if measured_speed > self.max_measured_speed_m_s:
            self._latch_fault(f"speed limit exceeded: {measured_speed:.2f} m/s")
            self.publish_command(np.zeros(4))
            return
        if tilt > self.max_tilt_rad:
            self._latch_fault(f"tilt limit exceeded: {math.degrees(tilt):.1f} deg")
            self.publish_command(np.zeros(4))
            return

        try:
            command, telemetry = rate_thrust_command_frd(
                self.latest_state,
                self.gates[self.active_gate_index],
                self.config,
            )
        except Exception as exc:
            self._latch_fault(f"controller update failed: {exc}")
            self.publish_command(np.zeros(4))
            return
        if telemetry.centering_error_m > self.max_centering_error_m:
            self._latch_fault(
                f"track corridor exceeded: {telemetry.centering_error_m:.2f} m"
            )
            self.publish_command(np.zeros(4))
            return

        self.publish_command(command)
        if now - self.last_log_s >= 1.0:
            self.last_log_s = now
            self.get_logger().info(
                f"gate={self.active_gate_index} forward={telemetry.forward_distance_m:.2f}m "
                f"center={telemetry.centering_error_m:.2f}m speed={measured_speed:.2f}m/s "
                f"p_ned={np.round(self.latest_state[:3], 2).tolist()} "
                f"v_ned={np.round(self.latest_state[3:6], 2).tolist()} "
                f"a_nwu={np.round(telemetry.desired_acceleration_nwu, 2).tolist()} "
                f"rpy={np.round(np.degrees(telemetry.current_euler_ned_frd), 1).tolist()} "
                f"rpy_d={np.round(np.degrees(telemetry.desired_euler_ned_frd), 1).tolist()} "
                f"rates_frd={np.round(command[:3], 1).tolist()}deg/s "
                f"thrust={command[3]:.3f}"
            )

    def publish_command(self, command: np.ndarray) -> None:
        msg = Float64MultiArray()
        msg.data = [float(x) for x in command]
        self.command_pub.publish(msg)

    def destroy_node(self):
        if self.ever_enabled and rclpy.ok():
            self.publish_command(np.zeros(4))
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AigpVq1ControllerNode()
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
