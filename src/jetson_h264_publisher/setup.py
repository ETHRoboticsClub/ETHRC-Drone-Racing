from setuptools import setup
from glob import glob

package_name = "jetson_h264_publisher"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Aaron",
    maintainer_email="aglaser@ethz.ch",
    description="Jetson NVENC H.264 ROS 2 publisher",
    license="MIT",
    entry_points={
        "console_scripts": [
            "encoder_node = jetson_h264_publisher.encoder_node:main",
        ],
    },
)
