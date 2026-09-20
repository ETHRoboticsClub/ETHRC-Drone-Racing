from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sensor_id = LaunchConfiguration('sensor_id')
    sensor_mode = LaunchConfiguration('sensor_mode')
    image_width = LaunchConfiguration('image_width')
    image_height = LaunchConfiguration('image_height')
    framerate = LaunchConfiguration('framerate')
    camera_info_url = LaunchConfiguration('camera_info_url')
    raw_output_encoding = LaunchConfiguration('raw_output_encoding')
    publish_mono = LaunchConfiguration('publish_mono')
    publish_rgb = LaunchConfiguration('publish_rgb')

    # 1. Native NITROS Argus Camera Node
    # Keep the camera-owned output on the high-performance NITROS path. Argus
    # negotiates RGB8 with downstream NITROS consumers on the A603.
    argus_mono_node = ComposableNode(
        name='argus_mono',
        package='isaac_ros_argus_camera',
        plugin='nvidia::isaac_ros::argus::ArgusMonoNode',
        namespace='cam0',
        parameters=[{
            'camera_id': ParameterValue(sensor_id, value_type=int),
            'module_id': -1,
            'mode': ParameterValue(sensor_mode, value_type=int),
            'sensor_mode': ParameterValue(sensor_mode, value_type=int),
            'output_encoding': raw_output_encoding,
            'fsync_type': 0,
            'camera_info_url': camera_info_url,
        }],
        remappings=[
            ('left/image_raw', 'image_raw_nitros'),
            ('left/camera_info', 'camera_info'),
        ]
    )

    # 2. Flip/relay: RGB8 -> RGB8 for the default color topic.
    # The flip corrects the A603 camera orientation and avoids the distorted
    # output produced by ImageFormatConverter on this branch.
    rgb_flip = ComposableNode(
        name='rgb_flip',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ImageFlipNode',
        namespace='cam0',
        parameters=[{
            'flip_mode': 'BOTH',
        }],
        remappings=[
            ('image', 'image_raw_nitros'),
            ('image_flipped', 'image_rgb')
        ],
        condition=IfCondition(publish_rgb),
    )

    # 3. Format Converter: RGB8 -> Mono8 (for VIO and mono ROS consumers)
    format_converter_mono = ComposableNode(
        name='format_converter_mono',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
        namespace='cam0',
        parameters=[{
            'encoding_desired': 'mono8',
            'image_width': ParameterValue(image_width, value_type=int),
            'image_height': ParameterValue(image_height, value_type=int),
        }],
        remappings=[
            ('image_raw', 'image_raw_nitros'),
            ('image', 'image_raw')
        ],
        condition=IfCondition(publish_mono),
    )

    # Put them all in a single hardware-accelerated container
    container = ComposableNodeContainer(
        name='nitros_camera_container',
        package='rclcpp_components',
        executable='component_container_mt',
        namespace='',
        composable_node_descriptions=[
            argus_mono_node,
            rgb_flip,
            format_converter_mono
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('sensor_id', default_value='0'),
        DeclareLaunchArgument('sensor_mode', default_value='4'),
        DeclareLaunchArgument('image_width', default_value='1280'),
        DeclareLaunchArgument('image_height', default_value='720'),
        DeclareLaunchArgument('framerate', default_value='60'),
        DeclareLaunchArgument(
            'camera_info_url',
            default_value='file:///home/aero/proj/ETHRC-Drone-Racing/src/perception/camera_calibration/camera_info.yaml',
        ),
        DeclareLaunchArgument('raw_output_encoding', default_value='nv12'),
        DeclareLaunchArgument(
            'publish_mono',
            default_value='true',
            description='Publish /cam0/image_raw as mono8 for VIO and other CPU ROS consumers',
        ),
        DeclareLaunchArgument(
            'publish_rgb',
            default_value='true',
            description='Publish /cam0/image_rgb as the default flipped NITROS RGB camera output.',
        ),
        container
    ])
