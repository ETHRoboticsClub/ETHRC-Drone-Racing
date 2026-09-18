import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'betaflight_msp_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ETHRC',
    maintainer_email='aero@ethz.ch',
    description='Bridge ROS 2 attitude commands to Multiwii OverrideRCIn',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'msp_bridge = betaflight_msp_controller.msp_bridge:main',
            'load_bf_config = betaflight_msp_controller.load_bf_config:main',
            'msp_tx_emulator = betaflight_msp_controller.msp_tx_emulator:main'
        ],
    },
)
