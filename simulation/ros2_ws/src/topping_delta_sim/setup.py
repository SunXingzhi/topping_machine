import os
from glob import glob

from setuptools import find_packages, setup

package_name = "topping_delta_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="topping_machine",
    maintainer_email="topping_machine@example.com",
    description="Delta 机械臂 ROS2 运动学仿真节点",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "delta_ik_node = topping_delta_sim.ik_node:main",
            "delta_viz_node = topping_delta_sim.viz_node:main",
            "delta_demo_node = topping_delta_sim.demo_node:main",
        ],
    },
)
