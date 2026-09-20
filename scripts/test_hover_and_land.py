#!/usr/bin/env python3
"""
Test script: Straight hover test from spawn position (0, 0, 0) up to 1.0m hover altitude,
holds stationary hover, and then descends back to (0, 0, 0) land position while tracking state fusion.
"""

import sys
import time
import math
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from pymavlink import mavutil


class StraightHoverTestNode(Node):
    def __init__(self):
        super().__init__('straight_hover_test_node')

        self.declare_parameter('device', 'udpin:0.0.0.0:14550')
        self.declare_parameter('state_topic', '/drone_state_estimated')
        self.declare_parameter('hover_height_m', 1.0)
        self.declare_parameter('hover_duration_s', 4.0)

        device = self.get_parameter('device').value
        state_topic = self.get_parameter('state_topic').value
        self.hover_height_m = float(self.get_parameter('hover_height_m').value)
        self.hover_duration_s = float(self.get_parameter('hover_duration_s').value)

        self.get_logger().info(f"Connecting to MAVLink simulator on {device}...")
        self.master = mavutil.mavlink_connection(device)
        self.master.wait_heartbeat(timeout=10)
        self.get_logger().info(f"Connected! Target system: {self.master.target_system}")

        self.latest_state = None
        self.state_history = []

        self.sub = self.create_subscription(
            Float64MultiArray,
            state_topic,
            self.state_callback,
            10
        )

        # Start test execution in timer
        self.system_boot_ms = int(time.time() * 1000)
        self.start_time = time.time()
        self.phase = "ARMING"
        self.timer = self.create_timer(0.05, self.control_loop)  # 20Hz control loop

    def state_callback(self, msg: Float64MultiArray):
        if len(msg.data) >= 13:
            self.latest_state = np.array(msg.data[:13], dtype=np.float64)
            self.state_history.append((time.time() - self.start_time, self.latest_state.copy()))

    def arm_drone(self, arm=True):
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1 if arm else 0,
                0, 0, 0, 0, 0, 0
            )
        except Exception as e:
            self.get_logger().error(f"Arm command error: {e}")

    def send_position_setpoint(self, x, y, z_ned, yaw_deg=0.0):
        now_ms = int(time.time() * 1000) - self.system_boot_ms
        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )
        try:
            self.master.mav.set_position_target_local_ned_send(
                now_ms,
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                type_mask,
                x, y, z_ned,
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0,
                math.radians(yaw_deg),
                0.0
            )
        except Exception as e:
            self.get_logger().error(f"Setpoint error: {e}")

    def control_loop(self):
        elapsed = time.time() - self.start_time

        if self.phase == "ARMING":
            self.arm_drone(True)
            self.send_position_setpoint(0.0, 0.0, 0.0)
            if elapsed > 1.0:
                self.phase = "TAKEOFF"
                self.get_logger().info(f"Phase -> TAKEOFF: ascending straight up to {self.hover_height_m}m")

        elif self.phase == "TAKEOFF":
            target_z = -self.hover_height_m
            self.send_position_setpoint(0.0, 0.0, target_z)
            if self.latest_state is not None:
                curr_z = self.latest_state[2]
                self.get_logger().info(
                    f"[TAKEOFF] Estimated Position: NED=[{self.latest_state[0]:.3f}, {self.latest_state[1]:.3f}, {curr_z:.3f}]",
                    throttle_duration_sec=0.5
                )
                if abs(curr_z - target_z) < 0.15 or elapsed > 4.0:
                    self.phase = "HOVER"
                    self.hover_start_time = time.time()
                    self.get_logger().info(f"Phase -> HOVER: holding stationary hover at 0,0,{target_z:.1f}m for {self.hover_duration_s}s")

        elif self.phase == "HOVER":
            target_z = -self.hover_height_m
            self.send_position_setpoint(0.0, 0.0, target_z)
            hover_elapsed = time.time() - self.hover_start_time
            if self.latest_state is not None:
                self.get_logger().info(
                    f"[HOVER] Estimated Position: NED=[{self.latest_state[0]:.3f}, {self.latest_state[1]:.3f}, {self.latest_state[2]:.3f}]",
                    throttle_duration_sec=1.0
                )
            if hover_elapsed >= self.hover_duration_s:
                self.phase = "LANDING"
                self.get_logger().info("Phase -> LANDING: descending straight back to (0, 0, 0)")

        elif self.phase == "LANDING":
            self.send_position_setpoint(0.0, 0.0, 0.0)
            if self.latest_state is not None:
                curr_z = self.latest_state[2]
                self.get_logger().info(
                    f"[LANDING] Estimated Position: NED=[{self.latest_state[0]:.3f}, {self.latest_state[1]:.3f}, {curr_z:.3f}]",
                    throttle_duration_sec=0.5
                )
                if abs(curr_z - 0.0) < 0.1 or elapsed > (5.0 + self.hover_duration_s + 4.0):
                    self.phase = "COMPLETE"
                    self.get_logger().info("Phase -> COMPLETE: disarming drone")
                    self.arm_drone(False)

        elif self.phase == "COMPLETE":
            self.get_logger().info("Hover and land test finished successfully!")
            self.print_summary()
            rclpy.shutdown()

    def print_summary(self):
        if not self.state_history:
            self.get_logger().warn("No state telemetry received during test.")
            return

        positions = np.array([s[1][:3] for s in self.state_history])
        final_pos = positions[-1]

        self.get_logger().info("=== STRAIGHT HOVER TEST SUMMARY ===")
        self.get_logger().info(f"Start Position: NED=[{positions[0][0]:.3f}, {positions[0][1]:.3f}, {positions[0][2]:.3f}]")
        self.get_logger().info(f"Max Altitude: {-np.min(positions[:, 2]):.3f} m")
        self.get_logger().info(f"Final Land Position: NED=[{final_pos[0]:.3f}, {final_pos[1]:.3f}, {final_pos[2]:.3f}]")
        self.get_logger().info(f"Land Drift from (0,0,0): {np.linalg.norm(final_pos):.3f} m")
        self.get_logger().info("====================================")


def main(args=None):
    rclpy.init(args=args)
    node = StraightHoverTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
