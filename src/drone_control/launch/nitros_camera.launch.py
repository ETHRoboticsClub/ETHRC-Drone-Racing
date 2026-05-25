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
    raw_output_encoding = LaunchConfiguration('raw_output_encoding')
    publish_mono = LaunchConfiguration('publish_mono')
    publish_rgb = LaunchConfiguration('publish_rgb')

    # 1. Native NITROS Argus Camera Node
    # Keep the camera-owned output as NV12/NITROS so the H.264 encoder can stay
    # on the hardware path. ROS Image branches are opt-in consumers below.
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
        }],
        remappings=[
            ('left/image_raw', 'image_raw_nitros'),
            ('left/camera_info', 'camera_info'),
        ]
    )

    # 2. Format Converter: NV12 -> RGB8 (only for color CPU consumers)
    format_converter_rgb = ComposableNode(
        name='format_converter_rgb',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
        namespace='cam0',
        parameters=[{
            'encoding_desired': 'rgb8',
            'image_width': ParameterValue(image_width, value_type=int),
            'image_height': ParameterValue(image_height, value_type=int),
        }],
        remappings=[
            ('image_raw', 'image_raw_nitros'),
            ('image', 'image_rgb')
        ],
        condition=IfCondition(publish_rgb),
    )

    # 3. Format Converter: NV12 -> Mono8 (for VIO and mono ROS consumers)
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
            format_converter_rgb,
            format_converter_mono
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('sensor_id', default_value='0'),
        DeclareLaunchArgument('sensor_mode', default_value='4'),
        DeclareLaunchArgument('image_width', default_value='1280'),
        DeclareLaunchArgument('image_height', default_value='720'),
        DeclareLaunchArgument('raw_output_encoding', default_value='nv12'),
        DeclareLaunchArgument(
            'publish_mono',
            default_value='true',
            description='Publish /cam0/image_raw as mono8 for VIO and other CPU ROS consumers',
        ),
        DeclareLaunchArgument(
            'publish_rgb',
            default_value='true',
            description='Publish /cam0/image_rgb as rgb8 for color CPU consumers. H.264 still uses /cam0/image_raw_nitros.',
        ),
        container
    ])
