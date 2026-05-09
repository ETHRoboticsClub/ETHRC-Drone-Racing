from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rl'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include model files (only .pt files, not directories)
        (os.path.join('share', package_name, 'models'), glob('rl/models/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ETHRC Aeronautics Division',
    maintainer_email='aeronautics@ethz.ch',
    description='RL Controller and Policy Inference for ETHRC Drone Racing',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_rl_interface = rl.test_rl_interface:main'
        ],
    },
)
