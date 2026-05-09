import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Image


PLACEHOLDER_MSG_TYPE = Imu  # TODO: replace with the real msg type for each non-IMU topic

DEFAULT_TOPICS = [
    (Imu, '/imu0'),
    (Image, '/image_raw')
    # (PLACEHOLDER_MSG_TYPE, '/some_other_topic'),
]


class TopicListener(Node):
    def __init__(self, topics=DEFAULT_TOPICS):
        super().__init__('imu_listener')
        self.subscriptions_ = [
            self.create_subscription(msg_type, topic, self.imu_callback, 10)
            for msg_type, topic in topics
        ]

    def imu_callback(self, msg):
        if msg == Imu:
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            o = msg.orientation
            w = msg.angular_velocity
            a = msg.linear_acceleration
            self.get_logger().info(
                f"[{t:.3f}] frame={msg.header.frame_id} "
                f"orient=({o.x:.3f}, {o.y:.3f}, {o.z:.3f}, {o.w:.3f}) "
                f"ang_vel=({w.x:.3f}, {w.y:.3f}, {w.z:.3f}) rad/s "
                f"lin_acc=({a.x:.3f}, {a.y:.3f}, {a.z:.3f}) m/s^2"
            )
        elif msg == Image:
            self.get_logger().info(
                f"Placeholder"
            )
            


def main(args=None):
    rclpy.init(args=args)
    node = TopicListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
