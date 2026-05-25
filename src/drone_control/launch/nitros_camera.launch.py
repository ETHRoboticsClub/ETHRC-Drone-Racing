from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def _launch_setup(context):
    sensor_id = int(LaunchConfiguration('sensor_id').perform(context))
    sensor_mode = int(LaunchConfiguration('sensor_mode').perform(context))
    image_width = int(LaunchConfiguration('image_width').perform(context))
    image_height = int(LaunchConfiguration('image_height').perform(context))
    framerate = float(LaunchConfiguration('framerate').perform(context))
    raw_output_encoding = LaunchConfiguration('raw_output_encoding').perform(context)
    publish_mono = LaunchConfiguration('publish_mono').perform(context)
    publish_rgb = LaunchConfiguration('publish_rgb').perform(context)
    camera_info_url = LaunchConfiguration('camera_info_url').perform(context)

    # 1. Native NITROS Argus Camera Node
    argus_mono_node = ComposableNode(
        name='argus_mono',
        package='isaac_ros_argus_camera',
        plugin='nvidia::isaac_ros::argus::ArgusMonoNode',
        namespace='cam0',
        parameters=[{
            'camera_id': sensor_id,
            'module_id': -1,
            'mode': sensor_mode,
            'output_width': image_width,
            'output_height': image_height,
            'framerate': framerate,
            'fps': framerate,
            'output_encoding': raw_output_encoding,
            'camera_info_url': camera_info_url,
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
            'image_width': image_width,
            'image_height': image_height,
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
            'image_width': image_width,
            'image_height': image_height,
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

    return [container]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sensor_id', default_value='0'),
        DeclareLaunchArgument('sensor_mode', default_value='4'),
        DeclareLaunchArgument('image_width', default_value='1280'),
        DeclareLaunchArgument('image_height', default_value='720'),
        DeclareLaunchArgument('framerate', default_value='60.0'),
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
        DeclareLaunchArgument(
            'camera_info_url',
            default_value='file:///home/aero/proj/ETHRC-Drone-Racing/src/perception/camera_calibration/camera_info.yaml',
            description='Path to standard camera calibration YAML file',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
