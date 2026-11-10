"""发布演示目标点。"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node


class DemoTargetNode(Node):
    def __init__(self) -> None:
        super().__init__("delta_demo_node")

        self.declare_parameter("trajectory_type", "none")
        self.declare_parameter("amplitude", 0.040)
        self.declare_parameter("z", -0.260)
        self.declare_parameter("period", 8.0)
        self.declare_parameter("rate", 20.0)
        self.declare_parameter("frame_id", "base_link")

        self.trajectory_type = str(
            self.get_parameter("trajectory_type").value
        )
        self.amplitude = float(self.get_parameter("amplitude").value)
        self.z = float(self.get_parameter("z").value)
        self.period = max(0.1, float(self.get_parameter("period").value))
        self.rate = max(1.0, float(self.get_parameter("rate").value))
        self.frame_id = str(self.get_parameter("frame_id").value)

        self.publisher = self.create_publisher(
            PointStamped, "/delta/target_pose", 10
        )
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / self.rate, self.publish_target)

        self.get_logger().info(
            "demo_node ready: type=%s z=%.3f amplitude=%.3f"
            % (self.trajectory_type, self.z, self.amplitude)
        )

    def publish_target(self) -> None:
        if self.trajectory_type == "none":
            return

        now = self.get_clock().now()
        t = (now - self.start_time).nanoseconds * 1e-9
        phase = 2.0 * math.pi * t / self.period

        x = 0.0
        y = 0.0
        if self.trajectory_type == "point":
            x, y = 0.0, 0.0
        elif self.trajectory_type == "line":
            x = self.amplitude * math.sin(phase)
        elif self.trajectory_type == "circle":
            x = self.amplitude * math.cos(phase)
            y = self.amplitude * math.sin(phase)
        elif self.trajectory_type == "sweep":
            x = self.amplitude * math.sin(phase)
            y = (2.0 * self.amplitude / math.pi) * math.asin(
                math.sin(2.0 * phase)
            )
        else:
            self.get_logger().warning(
                "未知轨迹类型: %s" % self.trajectory_type
            )
            return

        msg = PointStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id
        msg.point.x = float(x)
        msg.point.y = float(y)
        msg.point.z = float(self.z)
        self.publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DemoTargetNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
