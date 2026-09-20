from setuptools import setup
import os
from glob import glob

package_name = 'gate_detection'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, f'{package_name}.models', f'{package_name}.utils', f'{package_name}.scripts'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'checkpoints'), glob('checkpoints/*.pth')),
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
            'inference_standalone = gate_detection.scripts.inference_standalone:main',
            'viewer_standalone = gate_detection.scripts.viewer_standalone:main',
        ],
    },
)
