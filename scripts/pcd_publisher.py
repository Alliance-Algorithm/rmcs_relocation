#!/usr/bin/env python3
"""读取 PCD 地图文件，发布为 sensor_msgs/PointCloud2（transient_local）

不依赖 pclpy/pcl，纯 Python 解析 PCD 格式（ascii + binary）。
用于 RVIZ 显示地图背景。
"""

import struct
import sys
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def parse_pcd_header(filepath: str) -> dict:
    """解析 PCD 文件头，返回字段名/类型/大小/点数/数据偏移"""
    info = {"fields": [], "sizes": [], "types": [], "counts": [], "width": 0, "height": 0, "points": 0}
    with open(filepath, "rb") as f:
        for line in f:
            line = line.decode("utf-8", errors="replace").strip()
            if line.startswith("FIELDS"):
                info["fields"] = line.split()[1:]
            elif line.startswith("SIZE"):
                info["sizes"] = [int(x) for x in line.split()[1:]]
            elif line.startswith("TYPE"):
                info["types"] = line.split()[1:]
            elif line.startswith("COUNT"):
                info["counts"] = [int(x) for x in line.split()[1:]]
            elif line.startswith("WIDTH"):
                info["width"] = int(line.split()[1])
            elif line.startswith("HEIGHT"):
                info["height"] = int(line.split()[1])
            elif line.startswith("POINTS"):
                info["points"] = int(line.split()[1])
            elif line.startswith("DATA"):
                info["data_mode"] = line.split()[1]
                # header 结束，余下为数据
                break
    return info


def field_name_to_pcl(name: str) -> int:
    """PCD 字段名 → sensor_msgs/PointField datatype"""
    mapping = {
        "x": PointField.FLOAT32,
        "y": PointField.FLOAT32,
        "z": PointField.FLOAT32,
        "intensity": PointField.FLOAT32,
        "normal_x": PointField.FLOAT32,
        "normal_y": PointField.FLOAT32,
        "normal_z": PointField.FLOAT32,
        "curvature": PointField.FLOAT32,
    }
    return mapping.get(name, PointField.FLOAT32)


class PCDPublisher(Node):
    def __init__(self):
        super().__init__("pcd_publisher")

        self.declare_parameter("map_path", "")
        self.declare_parameter("world_frame", "world")
        map_path = self.get_parameter("map_path").get_parameter_value().string_value

        if not map_path or not Path(map_path).exists():
            self.get_logger().error(f"map_path invalid or not found: {map_path}")
            return

        self.get_logger().info(f"loading PCD: {map_path}")
        cloud_msg = self.load_pcd(map_path)
        if cloud_msg is None:
            self.get_logger().error("failed to load PCD")
            return

        world_frame = self.get_parameter("world_frame").get_parameter_value().string_value
        cloud_msg.header.frame_id = world_frame
        cloud_msg.header.stamp = self.get_clock().now().to_msg()

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.pub = self.create_publisher(PointCloud2, "/rmcs_relocation/map", qos)
        self.pub.publish(cloud_msg)
        self.get_logger().info(f"map published: {cloud_msg.width * cloud_msg.height} points")
        self.create_timer(60.0, lambda: None)

    def load_pcd(self, filepath: str) -> PointCloud2 | None:
        info = parse_pcd_header(filepath)
        if not info["fields"]:
            return None

        n_points = info["points"]
        if n_points <= 0:
            n_points = info["width"] * info["height"]

        data_offset = 0
        with open(filepath, "rb") as f:
            for line in f:
                if line.startswith(b"DATA"):
                    data_offset = f.tell()
                    break

        with open(filepath, "rb") as f:
            f.seek(data_offset)
            raw = f.read()

        if info["data_mode"] == "ascii":
            points = np.loadtxt(raw.decode("utf-8").splitlines(), dtype=np.float32)
            if points.ndim == 1:
                points = points.reshape(1, -1)
        elif info["data_mode"] == "binary":
            row_bytes = sum(size * count for size, count in zip(info["sizes"], info["counts"]))
            dtype_map = {"F": "f", "U": "I", "I": "i"}
            fmt = "".join(dtype_map.get(t, "f") for t in info["types"])
            expected = n_points * row_bytes
            if len(raw) < expected:
                self.get_logger().warn(f"binary data shorter than expected ({len(raw)} < {expected})")
            points = np.frombuffer(raw[:expected], dtype=np.float32).reshape(n_points, -1)
        else:
            return None

        if points.shape[0] < n_points:
            self.get_logger().warn(f"parsed {points.shape[0]} points, expected {n_points}")

        msg = PointCloud2()
        msg.header = Header()
        msg.height = 1
        msg.width = points.shape[0]
        msg.is_bigendian = False
        msg.is_dense = True

        offset = 0
        for i, name in enumerate(info["fields"]):
            pf = PointField()
            pf.name = name
            pf.offset = offset
            pf.datatype = field_name_to_pcl(name)
            pf.count = int(info["counts"][i]) if i < len(info["counts"]) else 1
            msg.fields.append(pf)
            offset += info["sizes"][i]

        msg.point_step = offset
        msg.row_step = msg.point_step * msg.width
        msg.data = points.astype(np.float32).tobytes()
        return msg


def main():
    rclpy.init(args=sys.argv)
    node = PCDPublisher()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
