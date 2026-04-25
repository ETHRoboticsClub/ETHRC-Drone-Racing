from setuptools import find_packages, setup

package_name = 'mpcc_pp'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    package_data={
        'mpcc_pp': [
            'parameters.yaml',
            'acados_models/**',
            'drone_mpcc_pp_acados_ocp.json',
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
        'casadi',
        'PyYAML',
    ],
    zip_safe=False,
    maintainer='todo',
    maintainer_email='todo@todo.com',
    description='mpcc_pp for drone racing',
    license='TODO',
    extras_require={'test': ['pytest']},
    entry_points={'console_scripts': []},
)
