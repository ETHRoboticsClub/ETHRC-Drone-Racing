import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_drone_control = get_package_share_directory('drone_control')
    
    # Path to AIGP camera driver configurations
    camera_config = PathJoinSubstitution([
        FindPackageShare('aigp_camera_driver'), 'config', 'camera_params.yaml'
    ])

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument('controller_type', default_value='rl',
                              description='Type of controller: "mpc", "rl", or "random"'),
        DeclareLaunchArgument('rate_error_scale', default_value='300.0',
                              description='Torque-to-rate scaling for RL controller'),
        DeclareLaunchArgument('device', default_value='udpin:0.0.0.0:14550',
                              description='MAVLink UDP connection string to AIGP simulator'),
        DeclareLaunchArgument('publish_imu', default_value='true',
                              description='Publish /imu0 from the bridge node for VIO'),
        
        # 1. AIGP MAVLink Bridge Node
        Node(
            package='betaflight_mavlink_bridge',
            executable='aigp_bridge_node',
            name='aigp_bridge_node',
            output='screen',
            parameters=[{
                'device': LaunchConfiguration('device'),
                'publish_imu': LaunchConfiguration('publish_imu'),
            }]
        ),

        # 2. AIGP Camera Driver Composable Node Container
        ComposableNodeContainer(
            name='aigp_camera_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            output='screen',
            composable_node_descriptions=[
                ComposableNode(
                    package='aigp_camera_driver',
                    plugin='aigp_camera_driver::VisionDriverNode',
                    name='aigp_vision_driver_node',
                    parameters=[camera_config],
                    remappings=[
                        ('image_raw', '/cam0/image_raw'),
                        ('image_rgb', '/cam0/image_rgb'),
                        ('camera_info', '/cam0/camera_info'),
                    ],
                    extra_arguments=[{'use_intra_process_comms': True}],
                )
            ]
        ),

        # 3. Drone Controller Launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_drone_control, 'launch', 'drone_controller.launch.py')
            ),
            launch_arguments={
                'plotjuggler': 'false',
                'use_mavsdk': 'false',
                'controller_type': LaunchConfiguration('controller_type'),
                'rate_error_scale': LaunchConfiguration('rate_error_scale')
            }.items()
        )
    ])
