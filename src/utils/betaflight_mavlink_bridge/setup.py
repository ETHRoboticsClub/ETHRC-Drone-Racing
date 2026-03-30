from setuptools import setup
import os
from glob import glob

package_name = 'betaflight_mavlink_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dimetri Chau',
    maintainer_email='dimetri.chau@gmail.com',
    description='Bridge to send RC_CHANNELS_OVERRIDE MAVLink messages to Betaflight',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge_node = betaflight_mavlink_bridge.bridge_node:main'
        ],
    },
)
