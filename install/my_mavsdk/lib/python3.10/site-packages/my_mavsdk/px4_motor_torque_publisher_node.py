#!/usr/bin/env python3

import asyncio
import threading
from typing import List, Optional

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray

from mavsdk import System


class Px4MotorTorquePublisherNode(Node):
    """
    Publish PX4 motor actuator controls as a normalized 0..1 array.

    Source: MAVSDK Telemetry `actuator_control_target()`.
    - This typically represents actuator controls after mixing.
    - Values are usually in [-1, +1] for generic actuators; for multicopter motor outputs
      you will usually observe [0, 1].
    """

    def __init__(self) -> None:
        super().__init__("px4_motor_torque_publisher")

        self.declare_parameter("system_address", "serial:///dev/ttyACM0:115200")
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("publish_raw", False)

        self._system_address = (
            self.get_parameter("system_address").get_parameter_value().string_value
        )
        self._rate_hz = self.get_parameter("rate_hz").get_parameter_value().double_value
        self._publish_raw = (
            self.get_parameter("publish_raw").get_parameter_value().bool_value
        )

        self._motor_count = 4
        self._topic = "/px4/motor_torques"
        self._source = "output_status" # control_target | output_status

        # Calibration: two known (raw, normalized) points define the linear mapping.
        # raw=109 at idle → 0.000429, raw=1195 at ~50% thrust → 0.499589
        self._cal_raw_min = 109.0
        self._cal_raw_max = 1195.0
        self._cal_norm_min = 0.000429
        self._cal_norm_max = 0.499589


        if self._source not in ("control_target", "output_status"):
            self.get_logger().warn(
                "source must be 'control_target' or 'output_status'. Using 'control_target'."
            )
            self._source = "control_target"

        self._pub = self.create_publisher(Float32MultiArray, self._topic, 10)
        self._drone = System()

        # MAVSDK async loop thread
        self._asyncio_loop = asyncio.new_event_loop()
        self._loop_thread = threading.Thread(
            target=self._run_event_loop, name="mavsdk_motor_torque_loop", daemon=True
        )
        self._loop_thread.start()

        self._run_async(self._initialize_and_stream())

        self.get_logger().info(
            f"Publishing motor torques to '{self._topic}' (motor_count={self._motor_count}, source={self._source})."
        )

    def _run_event_loop(self) -> None:
        asyncio.set_event_loop(self._asyncio_loop)
        self._asyncio_loop.run_forever()

    def _run_async(self, coro):
        return asyncio.run_coroutine_threadsafe(coro, self._asyncio_loop)

    async def _initialize_and_stream(self) -> None:
        self.get_logger().info(f"Connecting to '{self._system_address}' ...")
        await self._drone.connect(system_address=self._system_address)

        async for state in self._drone.core.connection_state():
            if state.is_connected:
                self.get_logger().info("MAVSDK connected to vehicle.")
                break
            await asyncio.sleep(0.1)

        # Set telemetry rate where supported
        try:
            if self._source == "control_target":
                await self._drone.telemetry.set_rate_actuator_control_target(self._rate_hz)
            else:
                await self._drone.telemetry.set_rate_actuator_output_status(self._rate_hz)
            self.get_logger().info(
                f"Requested telemetry rate {self._rate_hz} Hz for source '{self._source}'."
            )
        except Exception as e:
            self.get_logger().warn(
                f"Could not set telemetry rate for '{self._source}': {e}. "
                "PX4 may not stream this message type on this link."
            )

        self.get_logger().info(f"Subscribing to MAVSDK stream '{self._source}' ...")

        if self._source == "control_target":
            await self._stream_control_target()
        else:
            await self._stream_output_status()

    def _publish(self, values: List[float]) -> None:
        msg = Float32MultiArray()
        msg.data = [float(v) for v in values]
        self._pub.publish(msg)

    @staticmethod
    def _clamp01(v: float) -> float:
        if v != v:  # NaN
            return 0.0
        if v < 0.0:
            return 0.0
        if v > 1.0:
            return 1.0
        return v

    def _normalize_output_value(self, v: float) -> float:
        """
        Normalize a PX4 actuator/motor output into [0, 1] using linear calibration.

        Two measured points define the line:
          (cal_raw_min, cal_norm_min)  and  (cal_raw_max, cal_norm_max)
        Values outside the calibrated range are extrapolated and clamped to [0, 1].
        """
        if v != v:  # NaN
            return 0.0

        raw_span = self._cal_raw_max - self._cal_raw_min
        if raw_span == 0.0:
            return 0.0

        norm_span = self._cal_norm_max - self._cal_norm_min
        normalized = (v - self._cal_raw_min) / raw_span * norm_span + self._cal_norm_min
        return self._clamp01(normalized)

    async def _stream_control_target(self) -> None:
        got_first = False
        async for act in self._drone.telemetry.actuator_control_target():
            if not got_first:
                self.get_logger().info(
                    f"Receiving 'control_target' data (sample: {act})."
                )
                got_first = True
            controls = list(getattr(act, "controls", []))
            if not controls:
                continue

            if self._publish_raw:
                out = [float(x) for x in controls[: self._motor_count]]
            else:
                out = [self._normalize_output_value(float(x)) for x in controls[: self._motor_count]]
            while len(out) < self._motor_count:
                out.append(0.0)

            self._publish(out)

    async def _stream_output_status(self) -> None:
        got_first = False
        async for act in self._drone.telemetry.actuator_output_status():
            if not got_first:
                self.get_logger().info(
                    f"Receiving 'output_status' data (sample: {act})."
                )
                got_first = True
            outputs = list(getattr(act, "actuator", []))
            if not outputs:
                continue

            if self._publish_raw:
                out = [float(x) for x in outputs[: self._motor_count]]
            else:
                out = [self._normalize_output_value(float(x)) for x in outputs[: self._motor_count]]
            while len(out) < self._motor_count:
                out.append(0.0)

            self._publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Px4MotorTorquePublisherNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

