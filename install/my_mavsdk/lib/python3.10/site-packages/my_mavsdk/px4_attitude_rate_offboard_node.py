#!/usr/bin/env python3

import asyncio
import math
import threading
from typing import Optional, Tuple

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from std_srvs.srv import SetBool

from mavsdk import System
from mavsdk.offboard import AttitudeRate, OffboardError


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class Px4AttitudeRateOffboardNode(Node):
    """
    Stream PX4 OFFBOARD attitude-rate setpoints from a ROS topic.

    Subscribes to a Float32MultiArray with:
      [ roll_rate_deg_s, pitch_rate_deg_s, yaw_rate_deg_s, thrust_0_to_1 ]

    Streams MAVSDK Offboard AttitudeRate at a fixed rate (default 50 Hz).
    This is a "real" OFFBOARD setpoint stream, so QGC's "No offboard signal"
    warning should disappear once this node is running and connected.
    """

    def __init__(self) -> None:
        super().__init__("px4_attitude_rate_offboard")

        self.declare_parameter("system_address", "serial:///dev/ttyACM0:115200")
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("arm_service", "/arm_drone")

        self._system_address = (
            self.get_parameter("system_address").get_parameter_value().string_value
        )

        self._rate_hz = self.get_parameter("rate_hz").get_parameter_value().double_value
        self._arm_service_name = (
            self.get_parameter("arm_service").get_parameter_value().string_value
        )
        if not math.isfinite(self._rate_hz) or self._rate_hz <= 0.0:
            self.get_logger().warn("rate_hz must be > 0. Using 50.0.")
            self._rate_hz = 50.0

        self._drone = System()
        self._connected = False

        # Last commanded setpoint (roll, pitch, yaw, thrust)
        self._last_cmd: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
        self._cmd_lock = threading.Lock()

        self.create_subscription(
            Float32MultiArray, "/offboard/attitude_rate_cmd", self._on_cmd, 10
        )

        self.create_service(SetBool, self._arm_service_name, self._on_arm_srv)

        # Run MAVSDK async work in its own event loop thread.
        self._asyncio_loop = asyncio.new_event_loop()
        self._loop_thread = threading.Thread(
            target=self._run_event_loop,
            name="mavsdk_att_rate_offboard_loop",
            daemon=True,
        )
        self._loop_thread.start()

        self._run_async(self._initialize_and_stream())

        self.get_logger().info(
            "px4_attitude_rate_offboard started. "
            f"Subscribing to /offboard/attitude_rate_cmd and streaming OFFBOARD at {self._rate_hz:.1f} Hz."
        )

    def _run_event_loop(self) -> None:
        asyncio.set_event_loop(self._asyncio_loop)
        self._asyncio_loop.run_forever()

    def _run_async(self, coro) -> asyncio.Future:
        return asyncio.run_coroutine_threadsafe(coro, self._asyncio_loop)

    def _on_cmd(self, msg: Float32MultiArray) -> None:
        data = list(msg.data)
        if len(data) != 4:
            self.get_logger().warn(
                f"Expected 4 floats [roll,pitch,yaw,thrust], got {len(data)}. Ignoring."
            )
            return

        roll, pitch, yaw, thrust = (float(data[0]), float(data[1]), float(data[2]), float(data[3]))

        max_rate = 180.0
        if not math.isfinite(max_rate) or max_rate <= 0.0:
            max_rate = 180.0

        roll = _clamp(roll, -max_rate, max_rate)
        pitch = _clamp(pitch, -max_rate, max_rate)
        yaw = _clamp(yaw, -max_rate, max_rate)
        thrust = _clamp(thrust, 0.0, 1.0)

        with self._cmd_lock:
            self._last_cmd = (roll, pitch, yaw, thrust)

    async def _initialize_and_stream(self) -> None:
        try:
            await self._drone.connect(system_address=self._system_address)
        except Exception as exc:
            self.get_logger().error(f"Failed to connect via MAVSDK: {exc}")
            return

        self.get_logger().info(f"Connecting to PX4 using MAVSDK at '{self._system_address}'...")
        async for state in self._drone.core.connection_state():
            if state.is_connected:
                break
            await asyncio.sleep(0.1)

        self._connected = True
        self.get_logger().info("MAVSDK connected. Starting OFFBOARD stream.")

        # PX4 requires at least one setpoint before starting offboard.
        await self._send_current_setpoint()

        try:
            await self._drone.offboard.start()
            self.get_logger().info("OFFBOARD started.")
        except OffboardError as exc:
            self.get_logger().error(f"Failed to start OFFBOARD: {exc}")
            return
        except Exception as exc:
            self.get_logger().error(f"Unexpected error starting OFFBOARD: {exc}")
            return

        period = 1.0 / self._rate_hz

        try:
            while rclpy.ok():
                await self._send_current_setpoint()
                await asyncio.sleep(period)
        finally:
            try:
                await self._drone.offboard.stop()
            except Exception:
                pass

    def _on_arm_srv(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        if not self._connected:
            response.success = False
            response.message = "Not connected to PX4 yet"
            return response

        if request.data:
            fut = self._run_async(self._drone.action.arm())
            try:
                fut.result(timeout=10.0)
                response.success = True
                response.message = "Arm command sent"
            except Exception as exc:
                response.success = False
                response.message = f"Arming failed: {exc}"
        else:
            fut = self._run_async(self._drone.action.disarm())
            try:
                fut.result(timeout=5.0)
                response.success = True
                response.message = "Disarm command sent"
            except Exception as exc:
                response.success = False
                response.message = f"Disarming failed: {exc}"

        return response

    async def _send_current_setpoint(self) -> None:
        with self._cmd_lock:
            roll, pitch, yaw, thrust = self._last_cmd

        sp = AttitudeRate(
            roll_deg_s=roll,
            pitch_deg_s=pitch,
            yaw_deg_s=yaw,
            thrust_value=thrust,
        )
        await self._drone.offboard.set_attitude_rate(sp)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Px4AttitudeRateOffboardNode()

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
