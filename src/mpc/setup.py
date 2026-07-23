from glob import glob
from setuptools import find_packages, setup

package_name = 'mpc'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    package_data={
        'mpc': [
            'c_generated_code/**',
            'c_generated_code/**/*.so',
            'c_generated_code/**/*.h',
            'c_generated_code/**/*.c',
            'c_generated_code/**/*.json',
            'drone_acados_ocp.json',
            'drone_thrust_acados_ocp.json',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'scipy',
        'PyYAML',
        'mavsdk',
        'casadi',
    ],
    zip_safe=True,
    maintainer='papaveneti',
    maintainer_email='michael.d.papadakis@gmail.com',
    description='Model Predictive Controller for drone racing',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [],
    },
)
