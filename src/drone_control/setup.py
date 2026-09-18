from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'drone_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
    ],
    install_requires=['setuptools', 'mavsdk'],
    zip_safe=True,
    maintainer='papaveneti',
    maintainer_email='michael.d.papadakis@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'drone_controller_node = drone_control.drone_controller_node:main',
            'send_pose_goal = drone_control.send_pose_goal:main',
            'send_trajectory_goal = drone_control.send_trajectory_goal:main',
            'waypoint_visualizer = drone_control.waypoint_visualizer:main',
            'state_bridge_node = drone_control.state_bridge_node:main',
        ],
    },
)
