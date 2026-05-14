"""RViz 可视化测试 launch

启动：
- pcd_publisher：发布静态地图点云（/rmcs_relocation/map）
- tf_to_pose：发布 world→base_link 的 PoseStamped 和 Path
- relocalize_trigger（可选）：把 RViz 的 2D Pose Estimate 转成 relocalize 服务调用
- rviz2（可选）：加载预置的 rviz.rviz 视图

注意：pcd_publisher 和 tf_to_pose 不再共用 location.yaml（命名空间不匹配，参数不会注入）。
本 launch 直接以 launch arg 形式喂参数给这两个 Python 节点。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_path = LaunchConfiguration("map_path")
    world_frame = LaunchConfiguration("world_frame")
    base_frame = LaunchConfiguration("base_frame")
    service_name = LaunchConfiguration("service_name")
    trigger_mode = LaunchConfiguration("trigger_mode")
    pointcloud_topic = LaunchConfiguration("pointcloud_topic")
    collect_duration_sec = LaunchConfiguration("collect_duration_sec")
    path_window = LaunchConfiguration("path_window")
    use_rviz = LaunchConfiguration("use_rviz")
    use_trigger = LaunchConfiguration("use_trigger")
    rviz_config = LaunchConfiguration("rviz_config")

    args = [
        DeclareLaunchArgument(
            "map_path",
            default_value="/rmcs_install/share/rmcs_relocation/maps/my.pcd",
            description="PCD 地图路径，pcd_publisher 用它发布静态地图背景",
        ),
        DeclareLaunchArgument("world_frame", default_value="world"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument(
            "service_name",
            default_value="/rmcs_relocation/relocalize",
            description="relocalize_trigger 调用的服务名",
        ),
        DeclareLaunchArgument(
            "trigger_mode",
            default_value="local",
            description="RViz 2D Pose Estimate 触发的模式：local / wide / initial",
        ),
        DeclareLaunchArgument(
            "pointcloud_topic",
            default_value="",
            description="trigger 透传的话题名；空字符串表示用服务端默认值",
        ),
        DeclareLaunchArgument(
            "collect_duration_sec",
            default_value="0.0",
            description="trigger 透传的采集时长；0 表示用服务端默认值",
        ),
        DeclareLaunchArgument(
            "path_window",
            default_value="500",
            description="tf_to_pose 发布的 Path 滑窗长度；0 关闭 Path",
        ),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument(
            "use_trigger",
            default_value="true",
            description="是否启动 relocalize_trigger（点 RViz 触发 relocalize）",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution(
                [FindPackageShare("rmcs_relocation"), "config", "rviz.rviz"]
            ),
        ),
    ]

    pcd_publisher = Node(
        package="rmcs_relocation",
        executable="pcd_publisher.py",
        name="pcd_publisher",
        output="screen",
        parameters=[{"map_path": map_path, "world_frame": world_frame}],
    )

    tf_to_pose = Node(
        package="rmcs_relocation",
        executable="tf_to_pose.py",
        name="tf_to_pose",
        output="screen",
        parameters=[
            {
                "world_frame": world_frame,
                "base_frame": base_frame,
                "rate_hz": 10.0,
                "path_window": path_window,
            }
        ],
    )

    relocalize_trigger = Node(
        package="rmcs_relocation",
        executable="relocalize_trigger.py",
        name="relocalize_trigger",
        output="screen",
        parameters=[
            {
                "mode": trigger_mode,
                "service_name": service_name,
                "pointcloud_topic": pointcloud_topic,
                "collect_duration_sec": collect_duration_sec,
            }
        ],
        condition=IfCondition(use_trigger),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([*args, pcd_publisher, tf_to_pose, relocalize_trigger, rviz])
