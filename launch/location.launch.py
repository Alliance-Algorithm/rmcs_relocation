from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = LaunchConfiguration("config")

    declare_config = DeclareLaunchArgument(
        "config",
        default_value=[FindPackageShare("rmcs_relocation"), "/config/location.yaml"],
        description="Path to rmcs_relocation parameter file",
    )

    location_server = Node(
        package="rmcs_relocation",
        executable="rmcs_relocation_server",
        name="rmcs_relocation",
        output="screen",
        parameters=[config],
    )

    return LaunchDescription([declare_config, location_server])
