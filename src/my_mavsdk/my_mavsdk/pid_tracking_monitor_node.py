#!/usr/bin/env python3

import csv
import math
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from std_msgs.msg import Float64MultiArray

from px4_msgs.msg import (
    ActuatorMotors,
    VehicleAngularVelocity,
    VehicleRatesSetpoint,
    VehicleThrustSetpoint,
)


DEFAULT_DATA_DIR = Path("/home/mattia/ETHRC-Drone-Racing/src/drone_control/data")
DEFAULT_CSV_PATH = DEFAULT_DATA_DIR / "pid_tracking.csv"


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

        self.declare_parameter("rate_hz", 100.0)
        self.declare_parameter("csv_path", str(DEFAULT_CSV_PATH))
        self.declare_parameter("plot_path", "")
        self.declare_parameter("publish_topic", "/pid_tracking")
        self.declare_parameter("print_every_n", 100)

        rate_hz = self.get_parameter("rate_hz").get_parameter_value().double_value
        csv_path = self.get_parameter("csv_path").get_parameter_value().string_value
        plot_path = self.get_parameter("plot_path").get_parameter_value().string_value
        publish_topic = self.get_parameter("publish_topic").get_parameter_value().string_value
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
        self._thrust_sp: Optional[VehicleThrustSetpoint] = None
        self._actuator_motors: Optional[ActuatorMotors] = None
        self._sample_count = 0
        self._t0_ns: Optional[int] = None

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
                ]
            )
            self.get_logger().info(f"Writing PID tracking CSV to {path}")
            self.get_logger().info(f"Will save PID error plot to {self._plot_path}")

        self.create_timer(1.0 / max(rate_hz, 1.0), self._timer_cb)
        self.get_logger().info(
            "Monitoring PX4 PID tracking on /fmu/out/vehicle_rates_setpoint, "
            "/fmu/out/vehicle_angular_velocity, /fmu/out/vehicle_thrust_setpoint, "
            "and /fmu/out/actuator_motors."
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

    def _thrust_sp_cb(self, msg: VehicleThrustSetpoint) -> None:
        self._thrust_sp = msg

    def _actuator_motors_cb(self, msg: ActuatorMotors) -> None:
        self._actuator_motors = msg

    def _timer_cb(self) -> None:
        if self._rates_sp is None or self._angular_velocity is None:
            return

        now_ns = self.get_clock().now().nanoseconds
        if self._t0_ns is None:
            self._t0_ns = now_ns
        t_s = (now_ns - self._t0_ns) * 1e-9

        roll_sp = float(self._rates_sp.roll)
        pitch_sp = float(self._rates_sp.pitch)
        yaw_sp = float(self._rates_sp.yaw)

        roll = float(self._angular_velocity.xyz[0])
        pitch = float(self._angular_velocity.xyz[1])
        yaw = float(self._angular_velocity.xyz[2])

        thrust = self._thrust_sp.xyz if self._thrust_sp is not None else [math.nan] * 3

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
        ]

        msg = Float64MultiArray()
        msg.data = sample
        self._pub.publish(msg)

        if self._csv_writer is not None:
            self._csv_writer.writerow(sample)

        self._sample_count += 1
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
        rclpy.shutdown()


if __name__ == "__main__":
    main()
