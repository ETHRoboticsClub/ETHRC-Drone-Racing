from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="camera_bridge",
            output="screen",
            arguments=[
                "/drone/camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
                "/drone/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
                "/model/x500_cam_0/pose@geometry_msgs/msg/Pose@gz.msgs.Pose",
            ],
        ),
    ])
