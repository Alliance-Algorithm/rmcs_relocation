#!/usr/bin/env python3
"""订阅 TF 链 world→base_link，发布 PoseStamped 和 Path 到 rviz

- PoseStamped → /rmcs_relocation/estimated_pose（当前位姿，rviz 显示为坐标系箭头）
- Path        → /rmcs_relocation/estimated_path（滑窗轨迹，rviz 显示走过的曲线）

不依赖 relocation server 主动发布 pose；只要 TF 在变化就一直更新。
"""

import sys
from collections import deque

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node


class TFToPose(Node):
    def __init__(self):
        super().__init__("tf_to_pose")

        self.declare_parameter("world_frame", "world")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("rate_hz", 10.0)
        # 路径滑窗长度。0 = 关闭 Path 发布。
        self.declare_parameter("path_window", 500)

        self.world_frame = self.get_parameter("world_frame").get_parameter_value().string_value
        self.base_frame_ = self.get_parameter("base_frame").get_parameter_value().string_value
        rate = self.get_parameter("rate_hz").get_parameter_value().double_value
        path_window = self.get_parameter("path_window").get_parameter_value().integer_value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pose_pub = self.create_publisher(PoseStamped, "/rmcs_relocation/estimated_pose", 10)
        self.path_pub = None
        self.path_buffer: deque[PoseStamped] | None = None
        if path_window > 0:
            self.path_pub = self.create_publisher(Path, "/rmcs_relocation/estimated_path", 10)
            self.path_buffer = deque(maxlen=path_window)

        interval = 1.0 / max(0.5, rate)
        self.timer = self.create_timer(interval, self.publish_pose)

        self.get_logger().info(
            f"publishing {self.world_frame}→{self.base_frame_} at {rate}Hz, "
            f"path_window={path_window}"
        )

    def publish_pose(self):
        try:
            stamp = self.tf_buffer.lookup_transform(
                self.world_frame, self.base_frame_, rclpy.time.Time()
            )
        except tf2_ros.TransformException:
            return

        msg = PoseStamped()
        msg.header.frame_id = self.world_frame
        msg.header.stamp = stamp.header.stamp
        msg.pose.position.x = stamp.transform.translation.x
        msg.pose.position.y = stamp.transform.translation.y
        msg.pose.position.z = stamp.transform.translation.z
        msg.pose.orientation = stamp.transform.rotation
        self.pose_pub.publish(msg)

        if self.path_pub is not None and self.path_buffer is not None:
            self.path_buffer.append(msg)
            path = Path()
            path.header.frame_id = self.world_frame
            path.header.stamp = msg.header.stamp
            path.poses = list(self.path_buffer)
            self.path_pub.publish(path)


def main():
    rclpy.init(args=sys.argv)
    node = TFToPose()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
