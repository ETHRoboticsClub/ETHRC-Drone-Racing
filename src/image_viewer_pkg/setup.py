from setuptools import find_packages, setup

package_name = 'image_viewer_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='slahlou',
    maintainer_email='saad.lahlou@epfl.ch',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'image_viewer = image_viewer_pkg.image_view:main',
        ],
    },
)
