#!/usr/bin/env python3
"""把 RViz 的 "2D Pose Estimate" 点击转成 rmcs_relocation/Relocalize 服务调用

订阅 /initialpose（PoseWithCovarianceStamped），收到一条就以该位姿作为
initial_guess_world_base 调一次 /rmcs_relocation/relocalize。

参数：
- mode: "local" | "wide" | "initial"（默认 local）
- service_name: 服务名
- pointcloud_topic / collect_duration_sec: 透传给服务请求，可空。

控制台日志会打印每次请求结果，方便对照 RViz 上的 TF 跳变。
"""

import sys

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node

from rmcs_relocation.srv import Relocalize


MODE_MAP = {
    "initial": Relocalize.Request.MODE_INITIAL,
    "local": Relocalize.Request.MODE_LOCAL,
    "wide": Relocalize.Request.MODE_WIDE,
}


class RelocalizeTrigger(Node):
    def __init__(self):
        super().__init__("relocalize_trigger")

        self.declare_parameter("mode", "local")
        self.declare_parameter("service_name", "/rmcs_relocation/relocalize")
        self.declare_parameter("pointcloud_topic", "")
        self.declare_parameter("collect_duration_sec", 0.0)

        mode_str = self.get_parameter("mode").get_parameter_value().string_value.lower()
        if mode_str not in MODE_MAP:
            self.get_logger().error(f"unknown mode '{mode_str}', falling back to 'local'")
            mode_str = "local"
        self.mode = MODE_MAP[mode_str]
        self.mode_str = mode_str
        self.service_name = self.get_parameter("service_name").get_parameter_value().string_value
        self.pointcloud_topic = (
            self.get_parameter("pointcloud_topic").get_parameter_value().string_value
        )
        self.collect_duration_sec = float(
            self.get_parameter("collect_duration_sec").get_parameter_value().double_value
        )

        self.client = self.create_client(Relocalize, self.service_name)
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self.on_initialpose, 10
        )
        self.in_flight = False

        self.get_logger().info(
            f"trigger ready: click 2D Pose Estimate in RViz to call {self.service_name} "
            f"with mode={mode_str}"
        )

    def on_initialpose(self, msg: PoseWithCovarianceStamped):
        if self.in_flight:
            self.get_logger().warn("previous request still in flight, ignoring this click")
            return

        if not self.client.service_is_ready():
            if not self.client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error(f"service {self.service_name} not available")
                return

        request = Relocalize.Request()
        request.mode = self.mode
        request.initial_guess_world_base = msg.pose.pose
        request.pointcloud_topic = self.pointcloud_topic
        request.collect_duration_sec = self.collect_duration_sec

        p = msg.pose.pose.position
        self.get_logger().info(
            f"calling {self.mode_str} at world=({p.x:.2f}, {p.y:.2f}, {p.z:.2f})"
        )

        self.in_flight = True
        future = self.client.call_async(request)
        future.add_done_callback(self.on_response)

    def on_response(self, future):
        self.in_flight = False
        try:
            response = future.result()
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f"service call failed: {error}")
            return

        if response is None:
            self.get_logger().error("response is None")
            return

        if response.success:
            p = response.estimated_world_base.position
            self.get_logger().info(
                f"{self.mode_str} ok: pose=({p.x:.2f},{p.y:.2f},{p.z:.2f}) "
                f"score={response.fitness_score:.4f} conf={response.confidence:.3f} "
                f"in_bounds={response.within_field_bounds} | {response.message}"
            )
        else:
            self.get_logger().warn(
                f"{self.mode_str} rejected: score={response.fitness_score:.4f} "
                f"conf={response.confidence:.3f} | {response.message}"
            )


def main():
    rclpy.init(args=sys.argv)
    node = RelocalizeTrigger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
