"""目标点 -> 三个主动臂角度，并执行约束流水线。"""

from __future__ import annotations

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState

from topping_delta_core import (
    C0Reachability,
    ConstraintPipeline,
    DeltaGeometry,
    PoseContext,
    elbow_points,
    inverse_kinematics,
    platform_attachment_points,
)
from topping_delta_core.config import load_constraints, load_geometry


class DeltaIKNode(Node):
    JOINT_NAMES = ("leg_1_joint", "leg_2_joint", "leg_3_joint")

    def __init__(self) -> None:
        super().__init__("delta_ik_node")

        self.declare_parameter("geometry_file", "")
        self.declare_parameter("constraints_file", "")
        self.declare_parameter("constraints_mode_override", "")
        self.declare_parameter("home_theta_deg", -74.42)
        self.declare_parameter("joint_rate", 20.0)
        self.declare_parameter("target_frame", "base_link")

        geometry_file = str(self.get_parameter("geometry_file").value)
        constraints_file = str(self.get_parameter("constraints_file").value)
        mode_override = str(
            self.get_parameter("constraints_mode_override").value
        )
        home_theta_deg = float(
            self.get_parameter("home_theta_deg").value
        )
        joint_rate = float(self.get_parameter("joint_rate").value)
        self.target_frame = str(
            self.get_parameter("target_frame").value
        )

        self.geom = (
            load_geometry(geometry_file)
            if geometry_file
            else DeltaGeometry()
        )
        mode, c0_enabled = (
            load_constraints(constraints_file)
            if constraints_file
            else ("shadow", True)
        )
        if mode_override:
            mode = mode_override

        self.pipeline = ConstraintPipeline(
            [C0Reachability(self.geom, enabled=c0_enabled)],
            mode=mode,
        )
        self.mode = mode

        self.joint_pub = self.create_publisher(
            JointState, "/joint_states", 10
        )
        self.status_pub = self.create_publisher(
            DiagnosticArray, "/delta/status", 10
        )
        self.target_sub = self.create_subscription(
            PointStamped,
            "/delta/target_pose",
            self.on_target,
            10,
        )

        self.prev_theta: np.ndarray | None = None
        self.home_theta = np.radians(
            np.array([home_theta_deg] * 3, dtype=float)
        )
        self.theta = self.home_theta.copy()
        self.joint_timer = self.create_timer(
            1.0 / max(1.0, joint_rate), self.publish_current
        )

        self.get_logger().info(
            "delta_ik_node ready: mode=%s geometry=R%.3f r%.3f L%.3f l%.3f"
            % (mode, self.geom.R, self.geom.r, self.geom.L, self.geom.l)
        )

    def publish_current(self) -> None:
        self.publish_joints(self.theta)

    def publish_joints(self, theta: np.ndarray) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.JOINT_NAMES)
        msg.position = [float(x) for x in np.asarray(theta).reshape(3)]
        self.joint_pub.publish(msg)

    def publish_status(
        self,
        statuses: list[DiagnosticStatus],
    ) -> None:
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = statuses
        self.status_pub.publish(array)

    @staticmethod
    def make_status(
        name: str,
        level: int,
        message: str,
        values: dict | None = None,
    ) -> DiagnosticStatus:
        status = DiagnosticStatus()
        status.name = name
        status.level = level
        status.message = message
        if values:
            status.values = [
                KeyValue(key=str(key), value=str(value))
                for key, value in values.items()
            ]
        return status

    def on_target(self, msg: PointStamped) -> None:
        if msg.header.frame_id and msg.header.frame_id != self.target_frame:
            self.get_logger().warning(
                "目标坐标系 %s 暂不转换，按 %s 处理"
                % (msg.header.frame_id, self.target_frame)
            )

        P = np.array(
            [msg.point.x, msg.point.y, msg.point.z], dtype=float
        )
        ik = inverse_kinematics(P, self.geom, prev=self.prev_theta)
        theta = np.asarray(ik.theta, dtype=float)

        if not bool(np.all(ik.ok)):
            self.publish_status(
                [
                    self.make_status(
                        "delta/ik",
                        DiagnosticStatus.ERROR,
                        "C0_UNREACHABLE",
                        {"P_m": np.round(P, 4).tolist()},
                    )
                ]
            )
            self.get_logger().error(
                "目标不可达: P=%s" % np.round(P, 4).tolist()
            )
            return

        B, _ = elbow_points(theta, self.geom)
        A = platform_attachment_points(P, self.geom)
        b_hat = (A - B) / self.geom.l
        ctx = PoseContext(
            P=P,
            theta=theta,
            B=B,
            A=A,
            b_hat=b_hat,
            ratio=ik.ratio,
            prev_theta=self.prev_theta,
        )
        decision = self.pipeline.evaluate(ctx)

        statuses = [
            self.make_status(
                "delta/ik",
                DiagnosticStatus.OK,
                "IK_OK",
                {
                    "theta_deg": np.round(
                        np.degrees(theta), 3
                    ).tolist()
                },
            )
        ]
        for result in decision.results:
            level = DiagnosticStatus.OK
            if not result.ok:
                level = (
                    DiagnosticStatus.WARN
                    if self.mode == "shadow"
                    else DiagnosticStatus.ERROR
                )
            statuses.append(
                self.make_status(
                    "delta/%s" % result.name,
                    level,
                    result.code,
                    {"margin": round(result.margin, 4)},
                )
            )
        statuses.append(
            self.make_status(
                "delta/pipeline",
                DiagnosticStatus.OK
                if decision.accepted
                else DiagnosticStatus.ERROR,
                "accepted=%s mode=%s"
                % (decision.accepted, self.mode),
            )
        )
        self.publish_status(statuses)

        if not decision.accepted:
            self.get_logger().error(
                "约束拒绝目标: P=%s mode=%s"
                % (np.round(P, 4).tolist(), self.mode)
            )
            return

        self.publish_joints(theta)
        self.prev_theta = theta.copy()
        self.theta = theta.copy()

        self.get_logger().info(
            "IK OK: P=%s theta_deg=%s"
            % (
                np.round(P, 4).tolist(),
                np.round(np.degrees(theta), 3).tolist(),
            )
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DeltaIKNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
