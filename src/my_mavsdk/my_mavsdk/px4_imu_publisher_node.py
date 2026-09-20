#!/usr/bin/env python3

import asyncio
import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from sensor_msgs.msg import Imu, MagneticField

from mavsdk import System


class Px4ImuPublisherNode(Node):
    def __init__(self):
        super().__init__("px4_imu_publisher")

        self.declare_parameter("system_address", "serial:///dev/ttyACM0:115200")
        self.declare_parameter("rate_hz", 50.0)
        self._rate_hz = self.get_parameter("rate_hz").get_parameter_value().double_value
        self._system_address = (
            self.get_parameter("system_address").get_parameter_value().string_value
        )
        self._frame_id = "base_link"
        self._publish_mag = True

        # Use absolute topic names so users can `ros2 topic echo /px4/imu`.
        self._imu_pub = self.create_publisher(Imu, "/px4/imu", 10)
        self._mag_pub = self.create_publisher(MagneticField, "/px4/mag", 10)

        self._drone = System()

        # Run MAVSDK async telemetry in its own event loop thread
        self._asyncio_loop = asyncio.new_event_loop()
        self._loop_thread = threading.Thread(
            target=self._run_event_loop, name="mavsdk_imu_loop", daemon=True
        )
        self._loop_thread.start()

        self._run_async(self._initialize_and_stream())

    def _run_event_loop(self):
        asyncio.set_event_loop(self._asyncio_loop)
        self._asyncio_loop.run_forever()

    def _run_async(self, coro):
        return asyncio.run_coroutine_threadsafe(coro, self._asyncio_loop)

    async def _initialize_and_stream(self):
        self.get_logger().info(f"Connecting to '{self._system_address}' ...")
        await self._drone.connect(system_address=self._system_address)

        async for state in self._drone.core.connection_state():
            if state.is_connected:
                self.get_logger().info("MAVSDK connected to vehicle.")
                break
            await asyncio.sleep(0.1)

        try:
            await self._drone.telemetry.set_rate_imu(self._rate_hz)
            self.get_logger().info("Requested IMU telemetry rate 200 Hz.")
        except Exception as e:
            self.get_logger().warn(
                f"Could not set IMU rate: {e}. "
                "PX4 may not support telemetry.imu() on this link."
            )

        self.get_logger().info("Subscribing to MAVSDK imu() stream ...")
        await self._imu_listener()

    async def _imu_listener(self):
        got_first = False
        async for imu in self._drone.telemetry.imu():
            if not got_first:
                self.get_logger().info(f"Receiving IMU data (first sample: {imu}).")
                got_first = True
            stamp = self.get_clock().now().to_msg()

            imu_msg = Imu()
            imu_msg.header.stamp = stamp
            imu_msg.header.frame_id = self._frame_id

            # MAVSDK IMU provides accel (m/s^2), gyro (rad/s) in FRD.
            # Orientation is not provided by this stream.
            imu_msg.orientation_covariance[0] = -1.0

            imu_msg.angular_velocity.x = float(imu.angular_velocity_frd.forward_rad_s)
            imu_msg.angular_velocity.y = float(imu.angular_velocity_frd.right_rad_s)
            imu_msg.angular_velocity.z = float(imu.angular_velocity_frd.down_rad_s)

            imu_msg.linear_acceleration.x = float(imu.acceleration_frd.forward_m_s2)
            imu_msg.linear_acceleration.y = float(imu.acceleration_frd.right_m_s2)
            imu_msg.linear_acceleration.z = float(imu.acceleration_frd.down_m_s2)

            self._imu_pub.publish(imu_msg)

            if self._publish_mag:
                mag_msg = MagneticField()
                mag_msg.header.stamp = stamp
                mag_msg.header.frame_id = self._frame_id

                # MAVSDK exposes magnetic_field_frd in gauss.
                mag_msg.magnetic_field.x = float(imu.magnetic_field_frd.forward_gauss)
                mag_msg.magnetic_field.y = float(imu.magnetic_field_frd.right_gauss)
                mag_msg.magnetic_field.z = float(imu.magnetic_field_frd.down_gauss)

                self._mag_pub.publish(mag_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Px4ImuPublisherNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

