"""启动 Delta 运动学仿真：robot_state_publisher + IK + 可视化 + 演示。"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    description_share = get_package_share_directory(
        "topping_delta_description"
    )
    sim_share = get_package_share_directory("topping_delta_sim")

    urdf_path = os.path.join(description_share, "urdf", "delta.urdf")
    with open(urdf_path, "r", encoding="utf-8") as stream:
        robot_description = stream.read()

    geometry_file = os.path.join(sim_share, "config", "geometry.yaml")
    constraints_file = os.path.join(
        sim_share, "config", "constraints.yaml"
    )
    rviz_file = os.path.join(description_share, "rviz", "delta.rviz")

    rviz = LaunchConfiguration("rviz")
    demo = LaunchConfiguration("demo")
    demo_z = LaunchConfiguration("demo_z")
    demo_amplitude = LaunchConfiguration("demo_amplitude")
    demo_period = LaunchConfiguration("demo_period")
    constraints_mode = LaunchConfiguration("constraints_mode")
    home_theta_deg = LaunchConfiguration("home_theta_deg")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="是否启动 RViz2",
            ),
            DeclareLaunchArgument(
                "demo",
                default_value="circle",
                description="演示轨迹：none/point/line/circle/sweep",
            ),
            DeclareLaunchArgument(
                "demo_z", default_value="-0.280", description="演示平面高度 m"
            ),
            DeclareLaunchArgument(
                "demo_amplitude",
                default_value="0.035",
                description="演示轨迹半幅 m",
            ),
            DeclareLaunchArgument(
                "demo_period",
                default_value="8.0",
                description="演示周期 s",
            ),
            DeclareLaunchArgument(
                "constraints_mode",
                default_value="",
                description="覆盖 constraints.yaml 的模式：off/shadow/enforce",
            ),
            DeclareLaunchArgument(
                "home_theta_deg",
                default_value="-74.42",
                description="收到目标前的初始关节角",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                output="screen",
                arguments=[
                    "--x", "0",
                    "--y", "0",
                    "--z", "-0.060",
                    "--roll", "0",
                    "--pitch", "0.5236",
                    "--yaw", "0",
                    "--frame-id", "platform",
                    "--child-frame-id", "tool0",
                ],
            ),
            Node(
                package="topping_delta_sim",
                executable="delta_ik_node",
                output="screen",
                parameters=[
                    {
                        "geometry_file": geometry_file,
                        "constraints_file": constraints_file,
                        "constraints_mode_override": constraints_mode,
                        "home_theta_deg": ParameterValue(
                            home_theta_deg, value_type=float
                        ),
                    }
                ],
            ),
            Node(
                package="topping_delta_sim",
                executable="delta_viz_node",
                output="screen",
                parameters=[{"geometry_file": geometry_file}],
            ),
            Node(
                package="topping_delta_sim",
                executable="delta_demo_node",
                output="screen",
                parameters=[
                    {
                        "trajectory_type": demo,
                        "z": ParameterValue(demo_z, value_type=float),
                        "amplitude": ParameterValue(
                            demo_amplitude, value_type=float
                        ),
                        "period": ParameterValue(
                            demo_period, value_type=float
                        ),
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
                arguments=["-d", rviz_file],
                condition=IfCondition(rviz),
            ),
        ]
    )
