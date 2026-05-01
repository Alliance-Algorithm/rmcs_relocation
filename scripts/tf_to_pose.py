#!/usr/bin/env python3
"""订阅 TF 链 world→base_link，发布 PoseStamped 到 /rmcs_relocation/estimated_pose

用于 RVIZ 显示机器人实时位姿，不依赖 relocation server 主动发布 pose。
"""

import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import tf2_ros


class TFToPose(Node):
    def __init__(self):
        super().__init__("tf_to_pose")

        self.declare_parameter("world_frame", "world")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("rate_hz", 10.0)

        world = self.get_parameter("world_frame").get_parameter_value().string_value
        base = self.get_parameter("base_frame").get_parameter_value().string_value
        rate = self.get_parameter("rate_hz").get_parameter_value().double_value

        self.world_frame = world
        self.base_frame_ = base

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(PoseStamped, "/rmcs_relocation/estimated_pose", 10)

        interval = 1.0 / max(0.5, rate)
        self.timer = self.create_timer(interval, self.publish_pose)

        self.get_logger().info(f"publishing {world}→{base} at {rate}Hz")

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
        self.pub.publish(msg)


def main():
    rclpy.init(args=sys.argv)
    node = TFToPose()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
