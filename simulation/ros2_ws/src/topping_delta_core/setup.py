import os
from glob import glob

from setuptools import find_packages, setup

package_name = "topping_delta_core"

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
    ],
    install_requires=["setuptools", "numpy", "PyYAML"],
    zip_safe=True,
    maintainer="topping_machine",
    maintainer_email="topping_machine@example.com",
    description="Delta 机械臂纯 Python 运动学内核与约束流水线",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "delta_core_cli = topping_delta_core.cli:main",
        ],
    },
)
