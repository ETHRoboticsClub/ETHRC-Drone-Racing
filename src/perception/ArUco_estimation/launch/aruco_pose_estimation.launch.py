from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # path to config directory
    config_dir = PathJoinSubstitution([FindPackageShare('aruco_pose_estimation'), 'config'])

    return LaunchDescription([
        # --- Arguments (Defaults) ---
        DeclareLaunchArgument('camera_topic', default_value='/cam0/image_raw'),
        DeclareLaunchArgument('world_frame', default_value='world'),
        DeclareLaunchArgument('camera_frame', default_value='front_camera_optical_link'),
        DeclareLaunchArgument('base_link_frame', default_value='base_link'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('publish_debug_image', default_value='true'),

        # Paths to config files
        DeclareLaunchArgument('marker_map_file',
            default_value=PathJoinSubstitution([config_dir, 'marker_map.yaml'])),
        DeclareLaunchArgument('camera_calibration_file',
            default_value=PathJoinSubstitution([config_dir, 'camera_calibration_simulation.yaml'])),

        # Static transform from base_link to front_camera_link
        # From x500_cam model.sdf: <pose>0.2 0 0.03 0 0 0</pose>
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_camera_tf',
            arguments=['0.2', '0', '0.03', '0', '0', '0', 'base_link', 'front_camera_link']
        ),

        # Static transform from front_camera_link to front_camera_optical_link
        # Converts from ROS camera frame convention to optical frame convention
        # This empirically determined transform gives excellent orientation accuracy (5.26° error)
        # roll=-90°, pitch=0°, yaw=-90°
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_to_optical_tf',
            arguments=['0', '0', '0', '-1.5707963267948966', '0', '-1.5707963267948966',
                      'front_camera_link', 'front_camera_optical_link']
        ),

        # --- Node ---
        Node(
            package='aruco_pose_estimation',
            executable='aruco_pose_estimation_node',
            name='aruco_pose_estimation',
            output='screen',
            parameters=[{
                'camera_topic': LaunchConfiguration('camera_topic'),
                'marker_map_file': LaunchConfiguration('marker_map_file'),
                'camera_calibration_file': LaunchConfiguration('camera_calibration_file'),
                'world_frame': LaunchConfiguration('world_frame'),
                'camera_frame': LaunchConfiguration('camera_frame'),
                'base_link_frame': LaunchConfiguration('base_link_frame'),
                'publish_tf': LaunchConfiguration('publish_tf'),
                'publish_debug_image': LaunchConfiguration('publish_debug_image'),
            }]
        )
    ])