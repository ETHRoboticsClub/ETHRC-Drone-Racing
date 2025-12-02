from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    
    # Read CLI args (strings: "true"/"false")
    bridge_camera     = LaunchConfiguration("bridge_camera").perform(context)
    bridge_camera_imu = LaunchConfiguration("bridge_camera_imu").perform(context)
    bridge_camera_parameters = LaunchConfiguration("bridge_camera_parameters").perform(context)

    arguments = []

    if bridge_camera.lower() == "true":
        arguments.append(
            "/drone/camera/image@sensor_msgs/msg/Image@gz.msgs.Image"
        )

    if bridge_camera_imu.lower() == "true":
        arguments.append(
            "/world/walls/model/x500_cam_0/link/base_link/sensor/imu_sensor/imu"
            "@sensor_msgs/msg/Imu@gz.msgs.IMU"
        )

    if bridge_camera_parameters.lower() == "true":
        arguments.append(
            "/drone/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo"
        )

    print(f"[gz_sensors_bridge] Bridging topics: {arguments}")

    node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_sensors_bridge",
        output="screen",
        arguments=arguments,
    )

    return [node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "bridge_camera",
            default_value="true",
            description="Bridge /drone/camera/image",
        ),

        DeclareLaunchArgument(
            "bridge_camera_imu",
            default_value="true",
            description="Bridge camera_imu IMU topic",
        ),
        
        DeclareLaunchArgument(
            "bridge_camera_parameters",
            default_value="true",
            description="Bridge camera parameters topic",
        ),

        OpaqueFunction(function=launch_setup),
    ])
