from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rl'
nicola_policy = os.path.join('..', '..', 'ext', 'nicola', 'policy.pt')
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'models'), glob('rl/models/*.pt')),
    (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
]
if os.path.isfile(nicola_policy):
    data_files.append(
        (os.path.join('share', package_name, 'models', 'nicola'), [nicola_policy])
    )

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ETHRC Aeronautics Division',
    maintainer_email='aeronautics@ethz.ch',
    description='RL Controller and Policy Inference for ETHRC Drone Racing',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_rl_interface = rl.test_rl_interface:main',
            'aigp_policy_node = rl.aigp_policy_node:main',
            'aigp_vq1_controller_node = rl.aigp_vq1_controller_node:main',
            'aigp_state_estimator_node = rl.aigp_state_estimator_node:main',
        ],
    },
)
