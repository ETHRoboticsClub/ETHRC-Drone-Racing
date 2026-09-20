from setuptools import setup
import os
from glob import glob

package_name = 'gate_detection'

setup(
    name=package_name,
    version='0.1.0',
    packages=[
        package_name,
        f'{package_name}.models',
        f'{package_name}.utils',
        f'{package_name}.scripts',
        # v3.4 hybrid detector subpackage (DeepLabV3+ seg + kp + ng heads
        # + HybridSegNextGateTracker). Ported from DroneRacing_GateDetection
        # commit 14d21f3 — see handover.md for the dep-tree audit.
        f'{package_name}.v3_4',
        f'{package_name}.v3_4.data',
        f'{package_name}.v3_4.inference',
        f'{package_name}.v3_4.models',
        f'{package_name}.v3_4.models.heads',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'checkpoints'), glob('checkpoints/*.pth')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Xander Yap',
    maintainer_email='xander.yap@ethz.ch',
    description='Gate detection and PnP pose estimation for autonomous drone racing (WIP)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detector_node = gate_detection.detector_node:main',
            'detector_node_v34 = gate_detection.detector_node_v34:main',
            'inference_standalone = gate_detection.scripts.inference_standalone:main',
            'viewer_standalone = gate_detection.scripts.viewer_standalone:main',
        ],
    },
)
