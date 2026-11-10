"""Astra Pro 相机启动封装：官方驱动 + base_link→camera_link 静态外参。

生产配置默认只开彩色 + 深度（16UC1 毫米），关闭 IR 与全分辨率点云；
调试可用命令行覆盖，例如：
  ros2 launch topping_perception camera.launch.py enable_ir:=true enable_point_cloud:=true
"""

from __future__ import annotations

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_mount() -> dict:
    share = get_package_share_directory("topping_perception")
    path = os.path.join(share, "config", "camera_mount.yaml")
    with open(path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)["camera_mount"]


def generate_launch_description() -> LaunchDescription:
    astra_share = get_package_share_directory("astra_camera")
    astra_launch = os.path.join(astra_share, "launch", "astra_pro.launch.xml")

    # 透传给官方 launch 的参数（与 astra_pro.launch.xml 的 arg 同名）。
    # serial_number 不直接透传：官方 launch 的默认值是整数 0，
    # XML 的 $(var ...) 会把数字解析成 integer 参数，而驱动声明的是 string，会崩。
    passthrough = [
        "enable_ir",
        "enable_point_cloud",
        "enable_colored_point_cloud",
        "depth_registration",
        "color_qos",
        "depth_qos",
        "color_width",
        "color_height",
        "color_fps",
        "depth_width",
        "depth_height",
        "depth_fps",
    ]

    declarations = [
        DeclareLaunchArgument("serial_number", default_value=""),
        DeclareLaunchArgument("enable_ir", default_value="false"),
        DeclareLaunchArgument("enable_point_cloud", default_value="false"),
        DeclareLaunchArgument("enable_colored_point_cloud", default_value="false"),
        DeclareLaunchArgument("depth_registration", default_value="false"),
        DeclareLaunchArgument("color_qos", default_value="sensor_data"),
        DeclareLaunchArgument("depth_qos", default_value="sensor_data"),
        DeclareLaunchArgument("color_width", default_value="640"),
        DeclareLaunchArgument("color_height", default_value="480"),
        DeclareLaunchArgument("color_fps", default_value="30"),
        DeclareLaunchArgument("depth_width", default_value="640"),
        DeclareLaunchArgument("depth_height", default_value="480"),
        DeclareLaunchArgument("depth_fps", default_value="30"),
        DeclareLaunchArgument("laser_enable", default_value="true"),
    ]

    camera = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(astra_launch),
        launch_arguments={
            "camera_name": "camera",
            **{name: LaunchConfiguration(name) for name in passthrough},
        }.items(),
    )

    mount = _load_mount()
    mount_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=[
            "--x", str(mount["x"]),
            "--y", str(mount["y"]),
            "--z", str(mount["z"]),
            "--roll", str(mount["roll"]),
            "--pitch", str(mount["pitch"]),
            "--yaw", str(mount["yaw"]),
            "--frame-id", "base_link",
            "--child-frame-id", "camera_link",
        ],
    )

    # 深度全 0 多是设备里的红外激光器被关掉；驱动没有参数可设，只能调服务。
    # 该节点等待服务就绪后调用一次即退出，laser_enable:=false 可跳过。
    laser_enable_node = Node(
        package="topping_perception",
        executable="laser_enable",
        name="laser_enable_once",
        output="screen",
        parameters=[
            {
                "service_name": "/camera/set_laser_enable",
                "enable": True,
            }
        ],
        condition=IfCondition(LaunchConfiguration("laser_enable")),
    )

    return LaunchDescription([*declarations, camera, mount_tf, laser_enable_node])
