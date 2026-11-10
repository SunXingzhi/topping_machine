"""启动后调用一次红外发射器开关服务。

Astra Pro 的深度是全 0 时，最常见的原因是设备固件里的红外激光器处于关闭
状态（该状态由设备保存，拔插/断电后可能复位）。驱动没有对应的参数，只能
通过 /camera/set_laser_enable 服务设置。本节点等待服务就绪后调用一次并退出。
"""

import sys
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


class LaserEnabler(Node):
    def __init__(self):
        super().__init__("laser_enable_once")
        self.declare_parameter("service_name", "/camera/set_laser_enable")
        self.declare_parameter("enable", True)
        self.declare_parameter("wait_timeout", 10.0)

    def run(self) -> int:
        service_name = self.get_parameter("service_name").value
        enable = self.get_parameter("enable").value
        wait_timeout = self.get_parameter("wait_timeout").value

        client = self.create_client(SetBool, service_name)
        if not self._wait_until_ready(client, wait_timeout):
            self.get_logger().error(
                f"服务 {service_name} 在 {wait_timeout:.0f} s 内未就绪，跳过红外发射器设置"
            )
            return 1

        request = SetBool.Request()
        request.data = enable
        future = client.call_async(request)
        if not self._wait_until_done(future, 5.0):
            self.get_logger().error(f"调用 {service_name} 超时")
            return 1

        response = future.result()
        if not response.success:
            self.get_logger().warning(
                f"驱动返回失败：{response.message}；若深度仍全 0，请检查相机连接"
            )
            return 1

        self.get_logger().info(f"红外发射器已{'开启' if enable else '关闭'}")
        return 0

    def _wait_until_ready(self, client, timeout_sec: float) -> bool:
        """轮询并 spin，等待服务端出现（不 spin 时图更新不会生效）。"""
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if client.service_is_ready():
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return client.service_is_ready()

    def _wait_until_done(self, future, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if future.done():
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return future.done()


def main():
    rclpy.init()
    node = LaserEnabler()
    try:
        exit_code = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
