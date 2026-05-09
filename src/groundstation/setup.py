import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'groundstation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'static'), glob('groundstation/static/*')),
        (os.path.join('share', package_name, 'static', 'images'), glob('groundstation/static/images/*')),
        (os.path.join('lib', package_name), glob('scripts/*')),
    ],
    install_requires=['setuptools', 'fastapi', 'uvicorn'],
    zip_safe=True,
    maintainer='Aaron Glaser',
    maintainer_email='aglaser@ethz.ch',
    description='Web-based groundstation for live drone telemetry visualization',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_server = groundstation.web_server:main',
        ],
    },
)
