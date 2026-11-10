from setuptools import find_packages, setup

package_name = "topping_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/camera.launch.py"]),
        ("share/" + package_name + "/config", ["config/camera_mount.yaml"]),
    ],
    install_requires=["setuptools"],
    entry_points={
        "console_scripts": [
            "laser_enable = topping_perception.laser_enable:main",
        ],
    },
    zip_safe=True,
    maintainer="topping",
    maintainer_email="todo@example.com",
    description="Astra Pro 驱动启动封装与相机外参 TF",
    license="TODO",
)
