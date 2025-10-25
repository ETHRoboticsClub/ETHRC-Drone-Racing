from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'drone_mpc_control'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # RViz files
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mattia',
    maintainer_email='y',
    description='Drone MPC controller for ROS 2 using ACADOS',
    license='MIT',
    entry_points={
        'console_scripts': [
            'drone_mpc_node = drone_mpc_control.drone_mpc_node:main',
            'drone_mpc_mavsdk = drone_mpc_control.drone_mpc_mavsdk:main',
        ],
    },
)
