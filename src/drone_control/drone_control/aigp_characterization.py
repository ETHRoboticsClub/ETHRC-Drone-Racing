#!/usr/bin/env python3
"""
Black-box AIGP simulator characterization.

The node sends scripted attitude-rate/thrust commands through the same topic
used by the controllers and estimates the effective vertical thrust map,
thrust-to-weight ratio, thrust lag, and body-rate response from telemetry.
It works with HIGHRES_IMU alone; legacy state telemetry is used only when a
simulator build happens to expose it.
"""

from __future__ import annotations

import csv
import json
import math
import re
import ast
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool, Trigger


G = 9.80665


@dataclass
class Sample:
    t: float
    phase: str
    cmd_roll_deg_s: float
    cmd_pitch_deg_s: float
    cmd_yaw_deg_s: float
    cmd_thrust: float
    cmd_actuators: np.ndarray
    state: np.ndarray
    imu_accel: np.ndarray | None
    imu_gyro: np.ndarray | None


class AigpCharacterization(Node):
    def __init__(self):
        super().__init__("aigp_characterization")

        self.declare_parameter("state_topic", "/drone_state_sim")
        self.declare_parameter("command_mode", "actuator")
        self.declare_parameter("command_topic", "/drone_attitude_cmd")
        self.declare_parameter("actuator_command_topic", "/drone_actuator_cmd")
        self.declare_parameter("imu_topic", "/imu0")
        self.declare_parameter("arm_service", "/arm_drone")
        self.declare_parameter("reset_service", "/reset_sim")
        self.declare_parameter("output_dir", "aigp_characterization_runs")
        self.declare_parameter(
            "thrust_levels",
            [0.25, 0.35, 0.45, 0.55, 0.65, 0.75, 0.85, 0.95, 1.0],
        )
        self.declare_parameter("command_rate_hz", 50.0)
        self.declare_parameter("start_delay_s", 4.0)
        self.declare_parameter("pre_test_wait_s", 0.6)
        self.declare_parameter("takeoff_thrust", 0.75)
        self.declare_parameter("takeoff_duration_s", 0.6)
        self.declare_parameter("thrust_test_duration_s", 1.2)
        self.declare_parameter("settle_duration_s", 0.35)
        self.declare_parameter("rate_step_deg_s", 120.0)
        self.declare_parameter("actuator_step_delta", 0.08)
        self.declare_parameter("rate_step_duration_s", 0.7)
        # Build 3390 applies command changes after roughly 0.9 s. Excluding
        # this interval prevents the previous phase from contaminating fits.
        self.declare_parameter("response_settle_s", 1.1)
        self.declare_parameter("run_thrust_sweep", True)
        self.declare_parameter("run_rate_tests", True)
        self.declare_parameter("run_thrust_step_tests", True)
        # Negative selects the hover estimate from the thrust sweep. Set this
        # explicitly for rate-only runs so a poor vertical fit cannot invalidate
        # all body-rate phases.
        self.declare_parameter("rate_test_thrust_override", -1.0)

        self.state_topic = self.param_string("state_topic")
        self.command_mode = self.param_string("command_mode")
        self.command_topic = self.param_string("command_topic")
        self.actuator_command_topic = self.param_string("actuator_command_topic")
        self.imu_topic = self.param_string("imu_topic")
        self.output_dir = Path(self.param_string("output_dir")).expanduser()
        self.thrust_levels = self.param_double_list("thrust_levels")
        self.command_rate_hz = self.param_float("command_rate_hz")
        self.start_delay_s = self.param_float("start_delay_s")
        self.pre_test_wait_s = self.param_float("pre_test_wait_s")
        self.takeoff_thrust = self.param_float("takeoff_thrust")
        self.takeoff_duration_s = self.param_float("takeoff_duration_s")
        self.thrust_test_duration_s = self.param_float("thrust_test_duration_s")
        self.settle_duration_s = self.param_float("settle_duration_s")
        self.rate_step_deg_s = self.param_float("rate_step_deg_s")
        self.actuator_step_delta = self.param_float("actuator_step_delta")
        self.rate_step_duration_s = self.param_float("rate_step_duration_s")
        self.response_settle_s = self.param_float("response_settle_s")
        self.run_thrust_sweep = self.param_bool("run_thrust_sweep")
        self.run_rate_tests = self.param_bool("run_rate_tests")
        self.run_thrust_step_tests = self.param_bool("run_thrust_step_tests")
        self.rate_test_thrust_override = self.param_float(
            "rate_test_thrust_override"
        )

        self.command_pub = self.create_publisher(
            Float64MultiArray, self.command_topic, 10
        )
        self.actuator_pub = self.create_publisher(
            Float64MultiArray, self.actuator_command_topic, 10
        )
        self.state_sub = self.create_subscription(
            Float64MultiArray, self.state_topic, self.state_cb, 50
        )
        self.imu_sub = self.create_subscription(Imu, self.imu_topic, self.imu_cb, 50)
        self.arm_client = self.create_client(SetBool, self.param_string("arm_service"))
        self.reset_client = self.create_client(Trigger, self.param_string("reset_service"))

        self.state: np.ndarray | None = None
        self.imu_accel: np.ndarray | None = None
        self.imu_gyro: np.ndarray | None = None
        self.samples: list[Sample] = []
        self.current_phase = "idle"
        self.current_cmd = (0.0, 0.0, 0.0, 0.0)
        self.current_actuators = np.zeros(4)
        self.start_time = time.monotonic()

    def param_string(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def param_float(self, name: str) -> float:
        return self.get_parameter(name).get_parameter_value().double_value

    def param_bool(self, name: str) -> bool:
        return self.get_parameter(name).get_parameter_value().bool_value

    def param_double_list(self, name: str) -> list[float]:
        value = self.get_parameter(name).get_parameter_value()
        if value.double_array_value:
            return [float(x) for x in value.double_array_value]
        if value.integer_array_value:
            return [float(x) for x in value.integer_array_value]
        if value.string_value:
            parsed = ast.literal_eval(value.string_value)
            if isinstance(parsed, (list, tuple)):
                return [float(x) for x in parsed]
        raise ValueError(f"Parameter {name} must be a list of numbers.")

    def state_cb(self, msg: Float64MultiArray):
        if len(msg.data) < 13:
            self.get_logger().warn(
                f"Ignoring short state message on {self.state_topic}: {len(msg.data)}"
            )
            return

        self.state = np.array(msg.data[:13], dtype=float)

    def append_sample(self):
        roll, pitch, yaw, thrust = self.current_cmd
        self.samples.append(
            Sample(
                t=time.monotonic() - self.start_time,
                phase=self.current_phase,
                cmd_roll_deg_s=roll,
                cmd_pitch_deg_s=pitch,
                cmd_yaw_deg_s=yaw,
                cmd_thrust=thrust,
                cmd_actuators=self.current_actuators.copy(),
                state=(
                    np.full(13, np.nan, dtype=float)
                    if self.state is None
                    else self.state.copy()
                ),
                imu_accel=None if self.imu_accel is None else self.imu_accel.copy(),
                imu_gyro=None if self.imu_gyro is None else self.imu_gyro.copy(),
            )
        )

    def imu_cb(self, msg: Imu):
        self.imu_accel = np.array(
            [
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
            ],
            dtype=float,
        )
        self.imu_gyro = np.array(
            [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
            dtype=float,
        )
        self.append_sample()

    def run(self) -> tuple[Path, Path]:
        self.output_dir.mkdir(parents=True, exist_ok=True)
        run_id = time.strftime("%Y%m%d-%H%M%S")
        csv_path = self.output_dir / f"aigp_characterization_{run_id}.csv"
        metrics_path = self.output_dir / f"aigp_characterization_{run_id}.json"

        self.get_logger().info("Waiting for AIGP HIGHRES_IMU telemetry...")
        self.wait_for_imu(timeout_s=10.0)

        if self.run_thrust_sweep:
            self.get_logger().info("Running vertical thrust sweep.")
            for thrust in self.thrust_levels:
                self.reset_and_arm()
                self.command_phase(
                    "pre_test_wait", 0.0, 0.0, 0.0, 0.0, self.pre_test_wait_s
                )
                self.command_phase(
                    f"takeoff_for_thrust_{thrust:.2f}",
                    0.0,
                    0.0,
                    0.0,
                    self.takeoff_thrust,
                    self.takeoff_duration_s,
                )
                self.command_phase(
                    f"thrust_{thrust:.2f}",
                    0.0,
                    0.0,
                    0.0,
                    thrust,
                    self.thrust_test_duration_s,
                )
                self.command_phase(
                    "settle_zero", 0.0, 0.0, 0.0, 0.0, self.settle_duration_s
                )

        metrics = self.analyze()
        hover_thrust = metrics.get("hover_thrust")
        if self.rate_test_thrust_override >= 0.0:
            hover_thrust = self.rate_test_thrust_override
            metrics["rate_test_thrust"] = hover_thrust

        if hover_thrust is None:
            hover_thrust = 0.5
            self.get_logger().warn(
                "Could not estimate hover thrust from sweep; using 0.5 for step tests."
            )

        if self.run_thrust_step_tests:
            self.run_thrust_steps(float(hover_thrust))
            metrics = self.analyze()

        if self.run_rate_tests:
            self.run_body_rate_steps(float(hover_thrust))
            metrics = self.analyze()
        metrics["rate_test_thrust"] = float(hover_thrust)

        self.command_phase("finished_zero", 0.0, 0.0, 0.0, 0.0, 0.3)
        self.call_arm(False)
        self.write_csv(csv_path)
        self.write_metrics(metrics_path, metrics)

        self.get_logger().info(f"Wrote raw samples to {csv_path}")
        self.get_logger().info(f"Wrote fitted metrics to {metrics_path}")
        return csv_path, metrics_path

    def wait_for_imu(self, timeout_s: float):
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and self.imu_accel is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.imu_accel is None:
            raise RuntimeError(f"No IMU messages received on {self.imu_topic}")

    def reset_and_arm(self):
        self.call_trigger(self.reset_client, "reset")
        self.command_phase("post_reset_zero", 0.0, 0.0, 0.0, 0.0, self.start_delay_s)
        self.call_arm(True)
        self.command_phase("post_arm_zero", 0.0, 0.0, 0.0, 0.0, 0.2)

    def call_trigger(self, client, name: str, timeout_s: float = 2.0):
        if not client.wait_for_service(timeout_sec=timeout_s):
            self.get_logger().warn(f"{name} service unavailable; continuing.")
            return
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        if not future.done() or future.result() is None:
            self.get_logger().warn(f"{name} service call timed out or failed.")

    def call_arm(self, arm: bool, timeout_s: float = 2.0):
        if not self.arm_client.wait_for_service(timeout_sec=timeout_s):
            self.get_logger().warn("arm service unavailable; continuing.")
            return
        req = SetBool.Request()
        req.data = arm
        future = self.arm_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        if not future.done() or future.result() is None:
            self.get_logger().warn("arm service call timed out or failed.")

    def command_phase(
        self,
        phase: str,
        roll_deg_s: float,
        pitch_deg_s: float,
        yaw_deg_s: float,
        thrust: float,
        duration_s: float,
    ):
        self.current_phase = phase
        self.current_cmd = (roll_deg_s, pitch_deg_s, yaw_deg_s, thrust)
        msg = Float64MultiArray()
        if self.command_mode == "actuator":
            actuators = np.full(4, thrust, dtype=float)
            self.current_actuators = actuators
            msg.data = actuators.tolist()
            publisher = self.actuator_pub
        else:
            msg.data = [roll_deg_s, pitch_deg_s, yaw_deg_s, thrust]
            self.current_actuators = np.full(4, np.nan)
            publisher = self.command_pub

        period_s = 1.0 / self.command_rate_hz
        deadline = time.monotonic() + duration_s
        while rclpy.ok() and time.monotonic() < deadline:
            publisher.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period_s)

    def actuator_phase(
        self,
        phase: str,
        actuators: np.ndarray,
        duration_s: float,
        roll_cmd: float = 0.0,
        pitch_cmd: float = 0.0,
        yaw_cmd: float = 0.0,
    ):
        actuators = np.clip(np.asarray(actuators, dtype=float), 0.0, 1.0)
        self.current_phase = phase
        self.current_cmd = (roll_cmd, pitch_cmd, yaw_cmd, float(np.mean(actuators)))
        self.current_actuators = actuators
        msg = Float64MultiArray()
        msg.data = actuators.tolist()

        period_s = 1.0 / self.command_rate_hz
        deadline = time.monotonic() + duration_s
        while rclpy.ok() and time.monotonic() < deadline:
            self.actuator_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period_s)

    def actuator_mix(
        self,
        collective: float,
        roll_delta: float = 0.0,
        pitch_delta: float = 0.0,
        yaw_delta: float = 0.0,
    ) -> np.ndarray:
        # Motor order used by the sim model in this repo:
        # 0 front-right, 1 back-right, 2 back-left, 3 front-left.
        base = np.full(4, collective, dtype=float)
        roll = np.array([-1.0, -1.0, 1.0, 1.0]) * roll_delta
        pitch = np.array([1.0, -1.0, -1.0, 1.0]) * pitch_delta
        yaw = np.array([1.0, -1.0, 1.0, -1.0]) * yaw_delta
        return np.clip(base + roll + pitch + yaw, 0.0, 1.0)

    def run_thrust_steps(self, hover_thrust: float):
        self.get_logger().info("Running thrust step response tests.")
        base = float(np.clip(hover_thrust, 0.15, 0.85))
        up = float(np.clip(base + 0.15, 0.0, 1.0))
        down = float(np.clip(base - 0.15, 0.0, 1.0))

        self.reset_and_arm()
        self.command_phase(
            "thrust_step_takeoff",
            0.0,
            0.0,
            0.0,
            max(self.takeoff_thrust, up),
            self.takeoff_duration_s,
        )
        self.command_phase("thrust_step_base", 0.0, 0.0, 0.0, base, 0.8)
        self.command_phase("thrust_step_up", 0.0, 0.0, 0.0, up, 1.0)
        self.command_phase("thrust_step_base_after_up", 0.0, 0.0, 0.0, base, 0.8)
        self.command_phase("thrust_step_down", 0.0, 0.0, 0.0, down, 1.0)

    def run_body_rate_steps(self, hover_thrust: float):
        if self.command_mode == "actuator":
            self.get_logger().info("Running differential actuator step response tests.")
        else:
            self.get_logger().info("Running body-rate step response tests.")
        for axis in ("roll", "pitch", "yaw"):
            self.reset_and_arm()
            self.command_phase(
                f"rate_{axis}_takeoff",
                0.0,
                0.0,
                0.0,
                max(self.takeoff_thrust, hover_thrust + 0.1),
                self.takeoff_duration_s,
            )
            self.command_phase(f"rate_{axis}_base", 0.0, 0.0, 0.0, hover_thrust, 0.5)

            roll = self.rate_step_deg_s if axis == "roll" else 0.0
            pitch = self.rate_step_deg_s if axis == "pitch" else 0.0
            yaw = self.rate_step_deg_s if axis == "yaw" else 0.0
            if self.command_mode == "actuator":
                # Empirically verified against AIGP: these motor patterns map
                # yaw_delta -> roll rate, roll_delta -> pitch rate,
                # pitch_delta -> yaw rate.
                actuators = self.actuator_mix(
                    hover_thrust,
                    roll_delta=self.actuator_step_delta if axis == "pitch" else 0.0,
                    pitch_delta=self.actuator_step_delta if axis == "yaw" else 0.0,
                    yaw_delta=self.actuator_step_delta if axis == "roll" else 0.0,
                )
                self.actuator_phase(
                    f"rate_{axis}_step",
                    actuators,
                    self.rate_step_duration_s,
                    roll_cmd=roll,
                    pitch_cmd=pitch,
                    yaw_cmd=yaw,
                )
            else:
                self.command_phase(
                    f"rate_{axis}_step",
                    roll,
                    pitch,
                    yaw,
                    hover_thrust,
                    self.rate_step_duration_s,
                )
            self.command_phase(f"rate_{axis}_zero", 0.0, 0.0, 0.0, hover_thrust, 0.4)

    def analyze(self) -> dict:
        by_phase = self.samples_by_phase()
        thrust_points = self.estimate_thrust_points(by_phase)
        metrics = {
            "sample_count": len(self.samples),
            "command_mode": self.command_mode,
            "thrust_points": thrust_points,
        }

        fit = self.fit_thrust_map(thrust_points)
        metrics.update(fit)
        metrics["thrust_step_response"] = self.estimate_thrust_step_response(by_phase)
        metrics["body_rate_response"] = self.estimate_body_rate_response(by_phase)
        metrics["imu_noise"] = self.estimate_imu_noise(by_phase)
        return metrics

    def samples_by_phase(self) -> dict[str, list[Sample]]:
        phases: dict[str, list[Sample]] = {}
        for sample in self.samples:
            phases.setdefault(sample.phase, []).append(sample)
        return phases

    def estimate_thrust_points(self, by_phase: dict[str, list[Sample]]) -> list[dict]:
        points = []
        pattern = re.compile(r"^thrust_([0-9.]+)$")
        for phase, samples in by_phase.items():
            match = pattern.match(phase)
            if not match:
                continue
            thrust = float(match.group(1))
            response_settle_s = float(getattr(self, "response_settle_s", 0.15))
            estimate = self.velocity_slope(
                samples, axis=2, trim_start_s=response_settle_s
            )
            if estimate is not None:
                t, z, vz, accel = estimate
                z_span = float(np.max(z) - np.min(z))
                vz_span = float(np.max(vz) - np.min(vz))
                source = "state_velocity"
            else:
                imu_samples = [
                    sample for sample in samples
                    if sample.imu_accel is not None
                    and sample.t - samples[0].t >= response_settle_s
                ]
                if len(imu_samples) < 5:
                    continue
                # During the vertical sweep the vehicle is kept approximately
                # level. HIGHRES_IMU is specific force in body FRD, so adding
                # +g recovers NED vertical acceleration.
                accel = float(np.median([sample.imu_accel[2] for sample in imu_samples]) + G)
                z_span = math.nan
                vz_span = math.nan
                source = "imu_specific_force"
            points.append(
                {
                    "phase": phase,
                    "thrust": thrust,
                    "accel_z_state_units_m_s2": accel,
                    "z_span_m": z_span,
                    "vz_span_m_s": vz_span,
                    "sample_count": int(len(samples)),
                    "source": source,
                }
            )
        return sorted(points, key=lambda p: p["thrust"])

    def fit_thrust_map(self, thrust_points: list[dict]) -> dict:
        valid = [
            point
            for point in thrust_points
            if point["sample_count"] >= 5
            and (
                point.get("source") == "imu_specific_force"
                or point["vz_span_m_s"] > 0.03
            )
        ]
        if len(valid) < 3:
            return {
                "hover_thrust": None,
                "thrust_to_weight": None,
                "vertical_axis_convention": "unknown",
                "thrust_fit": None,
                "fit_warning": "Need at least 3 airborne thrust points.",
            }

        u = np.array([point["thrust"] for point in valid], dtype=float)
        az = np.array([point["accel_z_state_units_m_s2"] for point in valid], dtype=float)
        slope, intercept = np.polyfit(u, az, 1)
        predicted = slope * u + intercept
        rmse = float(np.sqrt(np.mean((predicted - az) ** 2)))

        hover = None
        if abs(slope) > 1e-6:
            hover = float(-intercept / slope)

        axis_convention = "ned_z_positive_down" if slope < 0.0 else "z_positive_up"
        az_at_full = float(slope + intercept)
        if slope < 0.0:
            twr = float((G - az_at_full) / G)
        else:
            twr = float((G + az_at_full) / G)

        return {
            "hover_thrust": hover,
            "thrust_to_weight": twr,
            "vertical_axis_convention": axis_convention,
            "thrust_fit": {
                "accel_z_per_thrust_slope": float(slope),
                "accel_z_intercept": float(intercept),
                "rmse_m_s2": rmse,
                "used_point_count": len(valid),
            },
        }

    def estimate_thrust_step_response(
        self, by_phase: dict[str, list[Sample]]
    ) -> dict[str, dict | None]:
        return {
            "step_up": self.estimate_accel_tau(by_phase.get("thrust_step_up", [])),
            "step_down": self.estimate_accel_tau(by_phase.get("thrust_step_down", [])),
        }

    def estimate_body_rate_response(
        self, by_phase: dict[str, list[Sample]]
    ) -> dict[str, dict | None]:
        result = {}
        for axis_name, axis_index in (("roll", 0), ("pitch", 1), ("yaw", 2)):
            samples = by_phase.get(f"rate_{axis_name}_step", [])
            result[axis_name] = self.estimate_rate_tau(samples, axis_index)
        return result

    def estimate_imu_noise(self, by_phase: dict[str, list[Sample]]) -> dict | None:
        static_samples = by_phase.get("pre_test_wait", [])
        accel = [s.imu_accel for s in static_samples if s.imu_accel is not None]
        gyro = [s.imu_gyro for s in static_samples if s.imu_gyro is not None]
        if len(accel) < 5 or len(gyro) < 5:
            return None
        accel_arr = np.vstack(accel)
        gyro_arr = np.vstack(gyro)
        return {
            "accel_mean_m_s2": accel_arr.mean(axis=0).tolist(),
            "accel_std_m_s2": accel_arr.std(axis=0).tolist(),
            "gyro_mean_rad_s": gyro_arr.mean(axis=0).tolist(),
            "gyro_std_rad_s": gyro_arr.std(axis=0).tolist(),
            "sample_count": len(accel),
        }

    def estimate_accel_tau(self, samples: list[Sample]) -> dict | None:
        series = self.accel_series(samples, axis=2, trim_start_s=0.0)
        if series is None:
            return None
        t, accel = series
        return self.step_tau_from_series(t, accel)

    def estimate_rate_tau(self, samples: list[Sample], axis_index: int) -> dict | None:
        if len(samples) < 5:
            return None
        t = np.array([s.t for s in samples], dtype=float)
        t -= t[0]
        state_omega = np.array([s.state[10 + axis_index] for s in samples], dtype=float)
        if np.all(np.isfinite(state_omega)):
            omega = state_omega
        else:
            gyro = [s.imu_gyro for s in samples if s.imu_gyro is not None]
            if len(gyro) != len(samples):
                return None
            omega = np.array([value[axis_index] for value in gyro], dtype=float)
        if self.command_mode == "actuator":
            cmd_rad_s = None
        else:
            cmd_rad_s = math.radians(
                float(
                    np.median(
                        [
                            (s.cmd_roll_deg_s, s.cmd_pitch_deg_s, s.cmd_yaw_deg_s)[
                                axis_index
                            ]
                            for s in samples
                        ]
                    )
                )
            )
        tau = self.step_tau_from_series(t, omega)
        if tau is None:
            return None
        tau["command_rad_s"] = cmd_rad_s
        if self.command_mode == "actuator":
            tau["command_actuator_delta"] = self.actuator_step_delta
        tau["steady_state_gain"] = (
            None
            if cmd_rad_s is None or abs(cmd_rad_s) < 1e-6
            else tau["final_value"] / cmd_rad_s
        )
        tau["max_abs_rate_rad_s"] = float(np.max(np.abs(omega)))
        return tau

    def step_tau_from_series(self, t: np.ndarray, y: np.ndarray) -> dict | None:
        if len(t) < 5 or t[-1] <= 0.0:
            return None
        initial_count = max(2, int(0.15 * len(y)))
        final_count = max(2, int(0.25 * len(y)))
        initial = float(np.mean(y[:initial_count]))
        final = float(np.mean(y[-final_count:]))
        delta = final - initial
        if abs(delta) < 1e-3:
            return {
                "tau_63_s": None,
                "initial_value": initial,
                "final_value": final,
                "delta": delta,
                "warning": "Step response too small to estimate tau.",
            }
        target = initial + 0.632 * delta
        if delta > 0.0:
            indices = np.where(y >= target)[0]
        else:
            indices = np.where(y <= target)[0]
        tau = None if len(indices) == 0 else float(t[indices[0]])
        return {
            "tau_63_s": tau,
            "initial_value": initial,
            "final_value": final,
            "delta": delta,
        }

    def velocity_slope(
        self, samples: list[Sample], axis: int, trim_start_s: float
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, float] | None:
        if len(samples) < 5:
            return None
        t = np.array([s.t for s in samples], dtype=float)
        t -= t[0]
        mask = t >= trim_start_s
        if np.count_nonzero(mask) < 5:
            return None
        t = t[mask]
        pos = np.array([s.state[axis] for s in samples], dtype=float)[mask]
        vel = np.array([s.state[3 + axis] for s in samples], dtype=float)[mask]
        if not np.all(np.isfinite(pos)) or not np.all(np.isfinite(vel)):
            return None
        slope, _ = np.polyfit(t, vel, 1)
        return t, pos, vel, float(slope)

    def accel_series(
        self, samples: list[Sample], axis: int, trim_start_s: float
    ) -> tuple[np.ndarray, np.ndarray] | None:
        if len(samples) < 6:
            return None
        t = np.array([s.t for s in samples], dtype=float)
        t -= t[0]
        vel = np.array([s.state[3 + axis] for s in samples], dtype=float)
        if not np.all(np.isfinite(vel)):
            accel_values = [
                s.imu_accel[axis] if s.imu_accel is not None else math.nan
                for s in samples
            ]
            accel = np.asarray(accel_values, dtype=float)
            if not np.all(np.isfinite(accel)):
                return None
            if axis == 2:
                accel = accel + G
            mask = t >= trim_start_s
            if np.count_nonzero(mask) < 6:
                return None
            return t[mask], accel[mask]
        mask = t >= trim_start_s
        t = t[mask]
        vel = vel[mask]
        if len(t) < 6:
            return None
        accel = np.gradient(vel, t)
        if len(accel) >= 5:
            kernel = np.ones(5) / 5.0
            accel = np.convolve(accel, kernel, mode="same")
        return t, accel

    def write_csv(self, path: Path):
        with path.open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    "t_s",
                    "phase",
                    "cmd_roll_deg_s",
                    "cmd_pitch_deg_s",
                    "cmd_yaw_deg_s",
                    "cmd_thrust",
                    "cmd_actuator_0",
                    "cmd_actuator_1",
                    "cmd_actuator_2",
                    "cmd_actuator_3",
                    "x",
                    "y",
                    "z",
                    "vx",
                    "vy",
                    "vz",
                    "qw",
                    "qx",
                    "qy",
                    "qz",
                    "wx",
                    "wy",
                    "wz",
                    "imu_ax",
                    "imu_ay",
                    "imu_az",
                    "imu_gx",
                    "imu_gy",
                    "imu_gz",
                ]
            )
            for sample in self.samples:
                imu_accel = sample.imu_accel if sample.imu_accel is not None else [math.nan] * 3
                imu_gyro = sample.imu_gyro if sample.imu_gyro is not None else [math.nan] * 3
                writer.writerow(
                    [
                        sample.t,
                        sample.phase,
                        sample.cmd_roll_deg_s,
                        sample.cmd_pitch_deg_s,
                        sample.cmd_yaw_deg_s,
                        sample.cmd_thrust,
                        *sample.cmd_actuators.tolist(),
                        *sample.state.tolist(),
                        *imu_accel,
                        *imu_gyro,
                    ]
                )

    def write_metrics(self, path: Path, metrics: dict):
        with path.open("w") as f:
            json.dump(metrics, f, indent=2, sort_keys=True)
            f.write("\n")


def print_summary(metrics_path: Path):
    metrics = json.loads(metrics_path.read_text())
    print("\nAIGP characterization summary")
    print(f"  command_mode: {metrics.get('command_mode')}")
    print(f"  hover_thrust: {metrics.get('hover_thrust')}")
    print(f"  thrust_to_weight: {metrics.get('thrust_to_weight')}")
    print(f"  vertical_axis: {metrics.get('vertical_axis_convention')}")
    fit = metrics.get("thrust_fit")
    if fit:
        print(
            "  thrust_fit: "
            f"az = {fit['accel_z_per_thrust_slope']:.3f} * u "
            f"+ {fit['accel_z_intercept']:.3f}, "
            f"rmse={fit['rmse_m_s2']:.3f}"
        )
    print(f"  metrics_json: {metrics_path}")


def main(args: Iterable[str] | None = None):
    rclpy.init(args=args)
    node = AigpCharacterization()
    try:
        _, metrics_path = node.run()
        print_summary(metrics_path)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
