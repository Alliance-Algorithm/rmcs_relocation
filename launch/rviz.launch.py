from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = LaunchConfiguration("config")
    use_rviz = LaunchConfiguration("use_rviz")

    declare_config = DeclareLaunchArgument(
        "config",
        default_value=[FindPackageShare("rmcs-relocation"), "/config/location.yaml"],
        description="Path to rmcs-relocation parameter file",
    )

    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch rviz2",
    )

    pcd_publisher = Node(
        package="rmcs-relocation",
        executable="pcd_publisher.py",
        name="pcd_publisher",
        output="screen",
        parameters=[config],
    )

    tf_to_pose = Node(
        package="rmcs-relocation",
        executable="tf_to_pose.py",
        name="tf_to_pose",
        output="screen",
        parameters=[config],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", [FindPackageShare("rmcs-relocation"), "/config/rviz.rviz"]],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        declare_config,
        declare_use_rviz,
        pcd_publisher,
        tf_to_pose,
        rviz,
    ])
