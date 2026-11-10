"""由关节角发布平台 TF 和从动臂 Marker。"""

from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import Point, TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from topping_delta_core import (
    DeltaGeometry,
    forward_kinematics_continuous,
)
from topping_delta_core.config import load_geometry


class DeltaVizNode(Node):
    JOINT_NAMES = ("leg_1_joint", "leg_2_joint", "leg_3_joint")

    def __init__(self) -> None:
        super().__init__("delta_viz_node")

        self.declare_parameter("geometry_file", "")
        self.declare_parameter("platform_thickness", 0.010)
        self.declare_parameter("rod_diameter", 0.012)
        self.declare_parameter("fork_width", 0.030)

        geometry_file = str(self.get_parameter("geometry_file").value)
        self.platform_thickness = float(
            self.get_parameter("platform_thickness").value
        )
        self.rod_diameter = float(
            self.get_parameter("rod_diameter").value
        )
        self.fork_width = float(
            self.get_parameter("fork_width").value
        )

        self.geom = (
            load_geometry(geometry_file)
            if geometry_file
            else DeltaGeometry()
        )

        self.br = TransformBroadcaster(self)
        self.marker_pub = self.create_publisher(
            MarkerArray, "/delta/markers", 10
        )
        self.joint_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.on_joints,
            10,
        )
        self.theta: np.ndarray | None = None
        self.prev_P: np.ndarray | None = None
        self.timer = self.create_timer(1.0 / 30.0, self.publish_visualization)

        self.get_logger().info("delta_viz_node ready")

    def on_joints(self, msg: JointState) -> None:
        positions = dict(zip(msg.name, msg.position))
        if not all(name in positions for name in self.JOINT_NAMES):
            return
        self.theta = np.array(
            [positions[name] for name in self.JOINT_NAMES],
            dtype=float,
        )

    @staticmethod
    def point(vec: np.ndarray) -> Point:
        msg = Point()
        msg.x = float(vec[0])
        msg.y = float(vec[1])
        msg.z = float(vec[2])
        return msg

    def publish_visualization(self) -> None:
        if self.theta is None:
            return

        fk = forward_kinematics_continuous(
            self.theta,
            self.geom,
            prev_P=self.prev_P,
        )
        if not bool(np.all(fk.ok)):
            self.get_logger().warning("FK 失败，跳过可视化")
            return

        P = np.asarray(fk.P, dtype=float)
        self.prev_P = P.copy()
        stamp = self.get_clock().now().to_msg()

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = "base_link"
        transform.child_frame_id = "platform"
        transform.transform.translation.x = float(P[0])
        transform.transform.translation.y = float(P[1])
        transform.transform.translation.z = float(P[2])
        transform.transform.rotation.w = 1.0
        self.br.sendTransform(transform)

        markers = MarkerArray()

        platform = Marker()
        platform.header.stamp = stamp
        platform.header.frame_id = "platform"
        platform.ns = "platform"
        platform.id = 0
        platform.type = Marker.CYLINDER
        platform.action = Marker.ADD
        platform.pose.orientation.w = 1.0
        platform.scale.x = 2.0 * self.geom.r
        platform.scale.y = 2.0 * self.geom.r
        platform.scale.z = self.platform_thickness
        platform.color.r = 0.35
        platform.color.g = 0.35
        platform.color.b = 0.40
        platform.color.a = 1.0
        markers.markers.append(platform)

        tool = Marker()
        tool.header.stamp = stamp
        tool.header.frame_id = "tool0"
        tool.ns = "tool"
        tool.id = 0
        tool.type = Marker.CYLINDER
        tool.action = Marker.ADD
        tool.pose.orientation.w = 1.0
        tool.scale.x = 0.030
        tool.scale.y = 0.030
        tool.scale.z = 0.020
        tool.color.r = 0.95
        tool.color.g = 0.45
        tool.color.b = 0.10
        tool.color.a = 1.0
        markers.markers.append(tool)

        rods = Marker()
        rods.header.stamp = stamp
        rods.header.frame_id = "base_link"
        rods.ns = "rods"
        rods.id = 0
        rods.type = Marker.LINE_LIST
        rods.action = Marker.ADD
        rods.pose.orientation.w = 1.0
        rods.scale.x = self.rod_diameter
        rods.color.r = 0.85
        rods.color.g = 0.15
        rods.color.b = 0.15
        rods.color.a = 1.0

        half_fork = 0.5 * self.fork_width
        for i in range(3):
            offset = half_fork * self.geom.t_dir[i]
            for sign in (-1.0, 1.0):
                rods.points.append(
                    self.point(fk.B[i] + sign * offset)
                )
                rods.points.append(
                    self.point(fk.A[i] + sign * offset)
                )
        markers.markers.append(rods)

        elbows = Marker()
        elbows.header.stamp = stamp
        elbows.header.frame_id = "base_link"
        elbows.ns = "elbows"
        elbows.id = 0
        elbows.type = Marker.SPHERE_LIST
        elbows.action = Marker.ADD
        elbows.pose.orientation.w = 1.0
        elbows.scale.x = 0.030
        elbows.scale.y = 0.030
        elbows.scale.z = 0.030
        elbows.color.r = 0.95
        elbows.color.g = 0.75
        elbows.color.b = 0.10
        elbows.color.a = 1.0
        elbows.points = [self.point(fk.B[i]) for i in range(3)]
        markers.markers.append(elbows)

        platform_joints = Marker()
        platform_joints.header.stamp = stamp
        platform_joints.header.frame_id = "base_link"
        platform_joints.ns = "platform_joints"
        platform_joints.id = 0
        platform_joints.type = Marker.SPHERE_LIST
        platform_joints.action = Marker.ADD
        platform_joints.pose.orientation.w = 1.0
        platform_joints.scale.x = 0.020
        platform_joints.scale.y = 0.020
        platform_joints.scale.z = 0.020
        platform_joints.color.r = 0.15
        platform_joints.color.g = 0.80
        platform_joints.color.b = 0.30
        platform_joints.color.a = 1.0
        platform_joints.points = [
            self.point(fk.A[i]) for i in range(3)
        ]
        markers.markers.append(platform_joints)

        self.marker_pub.publish(markers)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DeltaVizNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
