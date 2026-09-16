#!/usr/bin/env python3

import csv
import math
import os
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from std_msgs.msg import Float64MultiArray

from px4_msgs.msg import (
    ActuatorOutputs,
    ActuatorMotors,
    VehicleAngularVelocity,
    VehicleOdometry,
    VehicleRatesSetpoint,
    VehicleThrustSetpoint,
)


def _default_data_dir() -> Path:
    env_data_dir = os.environ.get("PID_TRACKING_DATA_DIR")
    if env_data_dir:
        return Path(env_data_dir).expanduser()

    candidate_workspaces = [
        Path.cwd(),
        Path.home() / "ros_ws",
        Path("/home/mattia/ETHRC-Drone-Racing"),
    ]
    for workspace in candidate_workspaces:
        drone_control_dir = workspace / "src" / "drone_control"
        if drone_control_dir.exists():
            return drone_control_dir / "data"

    return Path.cwd() / "src" / "drone_control" / "data"


DEFAULT_DATA_DIR = _default_data_dir()
DEFAULT_CSV_PATH = DEFAULT_DATA_DIR / "pid_tracking.csv"
ZEPHYROS_MOTOR_CONSTANT_N_PER_RAD2_S2 = 3.422e-6
ZEPHYROS_MOTOR_SPEED_MIN_RADPS = 100.0
ZEPHYROS_MOTOR_SPEED_MAX_RADPS = 1500.0
ZEPHYROS_MOTOR_TAU_UP_S = 0.0125
ZEPHYROS_MOTOR_TAU_DOWN_S = 0.025


class PidTrackingMonitorNode(Node):
    """
    Publish and optionally save PX4 rate-controller tracking data.

    Output topic /pid_tracking:
      [t_s,
       roll_sp, pitch_sp, yaw_sp,
       roll, pitch, yaw,
       roll_err, pitch_err, yaw_err,
       thrust_sp_x, thrust_sp_y, thrust_sp_z,
       actuator_mean, actuator_min, actuator_max]

    Rates are rad/s. Thrust setpoint is normalized body-frame thrust [-1, 1].
    Actuator values are normalized motor commands after control allocation.
    """

    def __init__(self) -> None:
        super().__init__("pid_tracking_monitor")

        self.declare_parameter("rate_hz", 200.0)
        self.declare_parameter("csv_path", str(DEFAULT_CSV_PATH))
        self.declare_parameter("plot_path", "")
        self.declare_parameter("publish_topic", "/pid_tracking")
        self.declare_parameter("ros_command_topic", "/drone_attitude_cmd")
        self.declare_parameter("ros_state_topic", "drone_state_sim")
        self.declare_parameter("mpcc_rotor_thrust_topic", "/mpcc/optimized_rotor_thrust")
        self.declare_parameter("allow_ros_fallback", False)
        self.declare_parameter("print_every_n", 0)
        self.declare_parameter("diagnostic_every_sec", 2.0)

        rate_hz = self.get_parameter("rate_hz").get_parameter_value().double_value
        csv_path = self.get_parameter("csv_path").get_parameter_value().string_value
        plot_path = self.get_parameter("plot_path").get_parameter_value().string_value
        publish_topic = self.get_parameter("publish_topic").get_parameter_value().string_value
        ros_command_topic = (
            self.get_parameter("ros_command_topic").get_parameter_value().string_value
        )
        ros_state_topic = (
            self.get_parameter("ros_state_topic").get_parameter_value().string_value
        )
        mpcc_rotor_thrust_topic = (
            self.get_parameter("mpcc_rotor_thrust_topic")
            .get_parameter_value()
            .string_value
        )
        diagnostic_every_sec = (
            self.get_parameter("diagnostic_every_sec").get_parameter_value().double_value
        )
        self._allow_ros_fallback = (
            self.get_parameter("allow_ros_fallback").get_parameter_value().bool_value
        )
        self._print_every_n = (
            self.get_parameter("print_every_n").get_parameter_value().integer_value
        )

        px4_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._rates_sp: Optional[VehicleRatesSetpoint] = None
        self._angular_velocity: Optional[VehicleAngularVelocity] = None
        self._vehicle_odometry: Optional[VehicleOdometry] = None
        self._thrust_sp: Optional[VehicleThrustSetpoint] = None
        self._actuator_motors: Optional[ActuatorMotors] = None
        self._actuator_outputs_sim: Optional[ActuatorOutputs] = None
        self._ros_rates_sp: Optional[list[float]] = None
        self._ros_angular_velocity: Optional[list[float]] = None
        self._mpcc_rotor_thrust: Optional[list[float]] = None
        self._ros_thrust_sp = math.nan
        self._gazebo_motor_omega: Optional[list[float]] = None
        self._last_motor_update_ns: Optional[int] = None
        self._sample_count = 0
        self._t0_ns: Optional[int] = None
        self._last_sample_source: Optional[str] = None
        self._warned_waiting_for_samples = False

        self._pub = self.create_publisher(Float64MultiArray, publish_topic, 10)

        self.create_subscription(
            VehicleRatesSetpoint,
            "/fmu/out/vehicle_rates_setpoint",
            self._rates_sp_cb,
            px4_qos,
        )
        self.create_subscription(
            VehicleAngularVelocity,
            "/fmu/out/vehicle_angular_velocity",
            self._angular_velocity_cb,
            px4_qos,
        )
        self.create_subscription(
            VehicleOdometry,
            "/fmu/out/vehicle_odometry",
            self._vehicle_odometry_cb,
            px4_qos,
        )
        self.create_subscription(
            VehicleThrustSetpoint,
            "/fmu/out/vehicle_thrust_setpoint",
            self._thrust_sp_cb,
            px4_qos,
        )
        self.create_subscription(
            ActuatorMotors,
            "/fmu/out/actuator_motors",
            self._actuator_motors_cb,
            px4_qos,
        )
        self.create_subscription(
            ActuatorOutputs,
            "/fmu/out/actuator_outputs_sim",
            self._actuator_outputs_sim_cb,
            px4_qos,
        )
        self.create_subscription(
            Float64MultiArray,
            ros_command_topic,
            self._ros_command_cb,
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            ros_state_topic,
            self._ros_state_cb,
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            mpcc_rotor_thrust_topic,
            self._mpcc_rotor_thrust_cb,
            10,
        )

        self._csv_file = None
        self._csv_writer = None
        self._csv_path: Optional[Path] = None
        self._plot_path: Optional[Path] = None
        if csv_path:
            path = Path(csv_path).expanduser()
            path.parent.mkdir(parents=True, exist_ok=True)
            self._csv_path = path
            self._plot_path = (
                Path(plot_path).expanduser()
                if plot_path
                else path.with_name(f"{path.stem}_error_plot.png")
            )
            self._plot_path.parent.mkdir(parents=True, exist_ok=True)
            self._csv_file = path.open("w", newline="")
            self._csv_writer = csv.writer(self._csv_file)
            self._csv_writer.writerow(
                [
                    "t_s",
                    "roll_sp_rad_s",
                    "pitch_sp_rad_s",
                    "yaw_sp_rad_s",
                    "roll_rad_s",
                    "pitch_rad_s",
                    "yaw_rad_s",
                    "roll_err_rad_s",
                    "pitch_err_rad_s",
                    "yaw_err_rad_s",
                    "thrust_sp_x",
                    "thrust_sp_y",
                    "thrust_sp_z",
                    "actuator_mean",
                    "actuator_min",
                    "actuator_max",
                    "total_thrust_n",
                    "data_source_code",
                    "mpcc_f0_n",
                    "mpcc_f1_n",
                    "mpcc_f2_n",
                    "mpcc_f3_n",
                    "gazebo_f0_n",
                    "gazebo_f1_n",
                    "gazebo_f2_n",
                    "gazebo_f3_n",
                    "rotor_f_err0_n",
                    "rotor_f_err1_n",
                    "rotor_f_err2_n",
                    "rotor_f_err3_n",
                    "mpcc_total_thrust_n",
                    "gazebo_motor_total_thrust_n",
                ]
            )
            self.get_logger().info(f"Writing PID tracking CSV to {path}")
            self.get_logger().info(f"Will save PID error plot to {self._plot_path}")

        self.create_timer(1.0 / max(rate_hz, 1.0), self._timer_cb)
        if diagnostic_every_sec > 0.0:
            self.create_timer(diagnostic_every_sec, self._diagnostic_cb)
        self.get_logger().info(
            "Monitoring PX4 PID tracking on /fmu/out/vehicle_rates_setpoint, "
            "/fmu/out/vehicle_angular_velocity, /fmu/out/vehicle_thrust_setpoint, "
            "and /fmu/out/actuator_motors."
        )
        self.get_logger().info(
            f"Fallback monitoring ROS/MAVSDK tracking on {ros_command_topic} "
            f"and {ros_state_topic}; enabled={self._allow_ros_fallback}."
        )
        self.get_logger().info(
            f"Tracking MPCC++ optimized rotor thrust on {mpcc_rotor_thrust_topic}."
        )

    def destroy_node(self) -> bool:
        if self._csv_file is not None:
            self._csv_file.close()
            self._csv_file = None
            self._make_error_plot()
        return super().destroy_node()

    def _make_error_plot(self) -> None:
        if self._csv_path is None or self._plot_path is None:
            return

        try:
            import matplotlib

            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except ImportError as exc:
            self.get_logger().error(
                "Could not create PID error plot because matplotlib is missing: "
                f"{exc}. Install it with `sudo apt install python3-matplotlib`."
            )
            return

        t_s = []
        roll_err = []
        pitch_err = []
        yaw_err = []

        try:
            with self._csv_path.open(newline="") as csv_file:
                reader = csv.DictReader(csv_file)
                for row in reader:
                    t_s.append(float(row["t_s"]))
                    roll_err.append(math.degrees(float(row["roll_err_rad_s"])))
                    pitch_err.append(math.degrees(float(row["pitch_err_rad_s"])))
                    yaw_err.append(math.degrees(float(row["yaw_err_rad_s"])))
        except (OSError, KeyError, ValueError) as exc:
            self.get_logger().error(f"Could not read PID tracking CSV for plot: {exc}")
            return

        if not t_s:
            self.get_logger().warn("No PID tracking samples recorded; skipping plot.")
            return

        fig, ax = plt.subplots(figsize=(11, 6), constrained_layout=True)
        ax.plot(t_s, roll_err, label="roll error")
        ax.plot(t_s, pitch_err, label="pitch error")
        ax.plot(t_s, yaw_err, label="yaw error")
        ax.axhline(0.0, color="black", linewidth=0.8, alpha=0.4)
        ax.set_title("PX4 rate-controller tracking error")
        ax.set_xlabel("time [s]")
        ax.set_ylabel("rate error [deg/s]")
        ax.grid(True, alpha=0.3)
        ax.legend()

        try:
            fig.savefig(self._plot_path, dpi=160)
        except OSError as exc:
            self.get_logger().error(f"Could not save PID error plot: {exc}")
        else:
            self.get_logger().info(f"Saved PID error plot to {self._plot_path}")
        finally:
            plt.close(fig)

    def _rates_sp_cb(self, msg: VehicleRatesSetpoint) -> None:
        self._rates_sp = msg

    def _angular_velocity_cb(self, msg: VehicleAngularVelocity) -> None:
        self._angular_velocity = msg

    def _vehicle_odometry_cb(self, msg: VehicleOdometry) -> None:
        self._vehicle_odometry = msg

    def _thrust_sp_cb(self, msg: VehicleThrustSetpoint) -> None:
        self._thrust_sp = msg

    def _actuator_motors_cb(self, msg: ActuatorMotors) -> None:
        self._actuator_motors = msg

    def _actuator_outputs_sim_cb(self, msg: ActuatorOutputs) -> None:
        self._actuator_outputs_sim = msg

    def _ros_command_cb(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 4:
            return

        # drone_attitude_cmd is a MAVSDK AttitudeRate command: deg/s, deg/s, deg/s, thrust.
        self._ros_rates_sp = [
            math.radians(float(msg.data[0])),
            math.radians(float(msg.data[1])),
            math.radians(float(msg.data[2])),
        ]
        self._ros_thrust_sp = float(msg.data[3])

    def _ros_state_cb(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 13:
            return

        # drone_state_sim is [p(3), v(3), q(4), w(3)], with w in rad/s.
        self._ros_angular_velocity = [
            float(msg.data[10]),
            float(msg.data[11]),
            float(msg.data[12]),
        ]

    def _mpcc_rotor_thrust_cb(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 4:
            return

        values = [float(v) for v in msg.data[:4]]
        if not all(math.isfinite(v) for v in values):
            return
        self._mpcc_rotor_thrust = values

    def _diagnostic_cb(self) -> None:
        if self._sample_count == 0 and not self._warned_waiting_for_samples:
            self.get_logger().warn(
                "No PID tracking samples yet. Waiting for either PX4 DDS topics "
                "or enabled ROS/MAVSDK fallback topics."
            )
            self._warned_waiting_for_samples = True

        self.get_logger().info(
            "topic status: "
            f"px4_rates_sp={self._rates_sp is not None}, "
            f"px4_angular_velocity={self._angular_velocity is not None}, "
            f"px4_vehicle_odometry={self._vehicle_odometry is not None}, "
            f"px4_actuator_outputs_sim={self._actuator_outputs_sim is not None}, "
            f"ros_attitude_cmd={self._ros_rates_sp is not None}, "
            f"ros_state={self._ros_angular_velocity is not None}, "
            f"mpcc_rotor_thrust={self._mpcc_rotor_thrust is not None}, "
            f"samples={self._sample_count}, "
            f"source={self._last_sample_source or 'none'}"
        )

    def _actuator_outputs_to_target_omega(self) -> Optional[list[float]]:
        if self._actuator_outputs_sim is None:
            return None

        outputs = [
            float(v)
            for v in self._actuator_outputs_sim.output[:4]
            if math.isfinite(float(v))
        ]
        if len(outputs) != 4:
            return None

        omega = []
        for control in outputs:
            control = min(max(control, 0.0), 1.0)
            omega.append(
                ZEPHYROS_MOTOR_SPEED_MIN_RADPS
                + control
                * (ZEPHYROS_MOTOR_SPEED_MAX_RADPS - ZEPHYROS_MOTOR_SPEED_MIN_RADPS)
            )
        return omega

    def _gazebo_motor_thrusts_n(self, now_ns: int) -> list[float]:
        target_omega = self._actuator_outputs_to_target_omega()
        if target_omega is None:
            return [math.nan] * 4

        if self._gazebo_motor_omega is None or self._last_motor_update_ns is None:
            self._gazebo_motor_omega = target_omega.copy()
            self._last_motor_update_ns = now_ns
        else:
            dt_s = max((now_ns - self._last_motor_update_ns) * 1e-9, 0.0)
            for i, target in enumerate(target_omega):
                current = self._gazebo_motor_omega[i]
                tau = (
                    ZEPHYROS_MOTOR_TAU_UP_S
                    if target > current
                    else ZEPHYROS_MOTOR_TAU_DOWN_S
                )
                alpha = 1.0 - math.exp(-dt_s / max(tau, 1e-9))
                self._gazebo_motor_omega[i] = current + alpha * (target - current)
            self._last_motor_update_ns = now_ns

        return [
            ZEPHYROS_MOTOR_CONSTANT_N_PER_RAD2_S2 * omega * omega
            for omega in self._gazebo_motor_omega
        ]

    def _timer_cb(self) -> None:
        sample_source = None
        data_source_code = math.nan
        thrust = [math.nan, math.nan, math.nan]

        if self._rates_sp is not None and (
            self._angular_velocity is not None or self._vehicle_odometry is not None
        ):
            sample_source = "PX4 DDS"
            data_source_code = 2.0
            roll_sp = float(self._rates_sp.roll)
            pitch_sp = float(self._rates_sp.pitch)
            yaw_sp = float(self._rates_sp.yaw)
            if self._angular_velocity is not None:
                roll = float(self._angular_velocity.xyz[0])
                pitch = float(self._angular_velocity.xyz[1])
                yaw = float(self._angular_velocity.xyz[2])
            else:
                roll = float(self._vehicle_odometry.angular_velocity[0])
                pitch = float(self._vehicle_odometry.angular_velocity[1])
                yaw = float(self._vehicle_odometry.angular_velocity[2])
            if self._thrust_sp is not None:
                thrust = [float(v) for v in self._thrust_sp.xyz]
        elif self._allow_ros_fallback and self._ros_rates_sp is not None and (
            self._vehicle_odometry is not None or self._ros_angular_velocity is not None
        ):
            roll_sp, pitch_sp, yaw_sp = self._ros_rates_sp
            if self._vehicle_odometry is not None:
                sample_source = "ROS command + PX4 odometry"
                data_source_code = 1.0
                roll = float(self._vehicle_odometry.angular_velocity[0])
                pitch = float(self._vehicle_odometry.angular_velocity[1])
                yaw = float(self._vehicle_odometry.angular_velocity[2])
            else:
                sample_source = "ROS/MAVSDK fallback"
                data_source_code = 0.0
                roll, pitch, yaw = self._ros_angular_velocity
            thrust[2] = self._ros_thrust_sp
        else:
            return

        now_ns = self.get_clock().now().nanoseconds
        if self._t0_ns is None:
            self._t0_ns = now_ns
        t_s = (now_ns - self._t0_ns) * 1e-9

        actuator_values = []
        if self._actuator_motors is not None:
            actuator_values = [
                float(v) for v in self._actuator_motors.control if math.isfinite(float(v))
            ]

        if actuator_values:
            actuator_mean = sum(actuator_values) / len(actuator_values)
            actuator_min = min(actuator_values)
            actuator_max = max(actuator_values)
        else:
            actuator_mean = math.nan
            actuator_min = math.nan
            actuator_max = math.nan
        gazebo_motor_thrusts = self._gazebo_motor_thrusts_n(now_ns)
        gazebo_motor_total_thrust_n = (
            sum(gazebo_motor_thrusts)
            if all(math.isfinite(v) for v in gazebo_motor_thrusts)
            else math.nan
        )
        total_thrust_n = gazebo_motor_total_thrust_n
        mpcc_rotor_thrust = (
            self._mpcc_rotor_thrust.copy()
            if self._mpcc_rotor_thrust is not None
            else [math.nan] * 4
        )
        rotor_force_error = [
            mpcc - gazebo
            if math.isfinite(mpcc) and math.isfinite(gazebo)
            else math.nan
            for mpcc, gazebo in zip(mpcc_rotor_thrust, gazebo_motor_thrusts)
        ]
        mpcc_total_thrust_n = (
            sum(mpcc_rotor_thrust)
            if all(math.isfinite(v) for v in mpcc_rotor_thrust)
            else math.nan
        )

        sample = [
            t_s,
            roll_sp,
            pitch_sp,
            yaw_sp,
            roll,
            pitch,
            yaw,
            roll_sp - roll,
            pitch_sp - pitch,
            yaw_sp - yaw,
            float(thrust[0]),
            float(thrust[1]),
            float(thrust[2]),
            actuator_mean,
            actuator_min,
            actuator_max,
            total_thrust_n,
            data_source_code,
            *mpcc_rotor_thrust,
            *gazebo_motor_thrusts,
            *rotor_force_error,
            mpcc_total_thrust_n,
            gazebo_motor_total_thrust_n,
        ]

        msg = Float64MultiArray()
        msg.data = sample
        self._pub.publish(msg)

        if self._csv_writer is not None:
            self._csv_writer.writerow(sample)

        self._sample_count += 1
        if sample_source != self._last_sample_source:
            self.get_logger().info(f"Recording PID tracking samples from {sample_source}.")
            self._last_sample_source = sample_source
        if self._print_every_n > 0 and self._sample_count % self._print_every_n == 0:
            self.get_logger().info(
                "rate error [deg/s] "
                f"roll={math.degrees(sample[7]):+.2f}, "
                f"pitch={math.degrees(sample[8]):+.2f}, "
                f"yaw={math.degrees(sample[9]):+.2f}; "
                f"thrust_z_sp={sample[12]:+.3f}; "
                f"actuator_mean={actuator_mean:.3f}"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PidTrackingMonitorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
