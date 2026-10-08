from glob import glob
import os
from setuptools import find_packages, setup


package_name = "amr_stm32_bridge"


setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AMR Team",
    maintainer_email="amr@example.com",
    description=(
        "ROS 2 Humble USB CDC bridge for STM32F411 cmd_vel, odometry and TF"
    ),
    license="MIT",
    entry_points={
        "console_scripts": [
            "stm32_bridge = amr_stm32_bridge.bridge_node:main",
        ],
    },
)
