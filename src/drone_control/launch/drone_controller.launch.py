from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import  PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch.actions import SetEnvironmentVariable
import os


def generate_launch_description():

    extended_pythonpath = os.path.expanduser('~/ros_ws/src')
    current_pythonpath = os.environ.get('PYTHONPATH', '')
    new_pythonpath_value = f"{current_pythonpath}:{extended_pythonpath}" if current_pythonpath else extended_pythonpath

    # rviz_config_file = PathJoinSubstitution(
    #     [FindPackageShare('drone_control'), "config", "assignement2.rviz"]
    # )   
    plotjuggler_config_file = PathJoinSubstitution(
        [FindPackageShare('drone_control'), "config", "control_plotjuggler.xml"]
    )   
    controller_params = PathJoinSubstitution(
        [FindPackageShare('drone_control'), "config", "drone_controller_config.yaml"]
    )   

    return LaunchDescription([
        SetEnvironmentVariable( # important, since we need to find the other packages
            name='PYTHONPATH',
            value=new_pythonpath_value
        ), 
        Node(
            package="drone_control",
            executable="drone_controller_node",
            name="drone_controller_node",
            parameters=[controller_params],
            output="screen",
        ),
        # Node(
        #     package="rviz2",
        #     executable="rviz2",
        #     name="rviz2",
        #     # arguments=["-d", rviz_config_file],
        #     # respawn=True,
        # ),
        Node(
        package="plotjuggler",
        executable="plotjuggler",
        output="screen",
        emulate_tty=True,
        arguments=['-l', plotjuggler_config_file]
        ),
        # Node(
        #     package="tf2_ros",
        #     executable="static_transform_publisher",
        #     name="static_world_map",
        #     arguments=["0","0","0","0","0","0","world","map"],
        # ),
    ])