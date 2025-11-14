from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/drone/camera/image',
        description='Camera image topic to subscribe to'
    )

    camera_calibration_file_arg = DeclareLaunchArgument(
        'camera_calibration_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('aruco_pose_estimation'),
            'config',
            'camera_calibration_simulation.yaml'
        ]),
        description='Path to camera calibration YAML file'
    )

    marker_map_file_arg = DeclareLaunchArgument(
        'marker_map_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('aruco_pose_estimation'),
            'config',
            'marker_map.yaml'
        ]),
        description='Path to marker map YAML file'
    )

    world_frame_arg = DeclareLaunchArgument(
        'world_frame',
        default_value='world',
        description='World/reference frame ID'
    )

    camera_frame_arg = DeclareLaunchArgument(
        'camera_frame',
        default_value='camera_link',
        description='Camera frame ID'
    )

    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf',
        default_value='true',
        description='Publish TF transforms'
    )

    publish_debug_image_arg = DeclareLaunchArgument(
        'publish_debug_image',
        default_value='true',
        description='Publish debug image with detected markers'
    )

    # Create node
    aruco_pose_estimation_node = Node(
        package='aruco_pose_estimation',
        executable='aruco_pose_estimation_node',
        name='aruco_pose_estimation',
        output='screen',
        parameters=[{
            'camera_topic': LaunchConfiguration('camera_topic'),
            'camera_calibration_file': LaunchConfiguration('camera_calibration_file'),
            'marker_map_file': LaunchConfiguration('marker_map_file'),
            'world_frame': LaunchConfiguration('world_frame'),
            'camera_frame': LaunchConfiguration('camera_frame'),
            'publish_tf': LaunchConfiguration('publish_tf'),
            'publish_debug_image': LaunchConfiguration('publish_debug_image'),
        }]
    )

    return LaunchDescription([
        camera_topic_arg,
        camera_calibration_file_arg,
        marker_map_file_arg,
        world_frame_arg,
        camera_frame_arg,
        publish_tf_arg,
        publish_debug_image_arg,
        aruco_pose_estimation_node,
    ])
